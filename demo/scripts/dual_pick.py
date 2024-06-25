import sys
import yaml
from yaml.loader import SafeLoader
import rosbag
import os
import copy
import numpy as np
import moveit_commander
import roslaunch
import rospy
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from scipy.spatial.transform import Rotation as R
from std_srvs.srv import Empty
import tf.transformations
import roboticstoolbox as rtb
import socket
import pickle
import time
import argparse
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Vector3
# from moveit_msgs.msg import MTCPlanGoal
from moveit.task_constructor import core, stages

from py_binding_tools import roscpp_init

# in clip frame
clip_size = [0.04, 0.04, 0.06]
hold_x_offset = 0.03
leader_pre_clip = [-(clip_size[0]/2+hold_x_offset), -clip_size[1]/2, clip_size[2]/2]
follower_pre_clip = [clip_size[0]/2+hold_x_offset, -clip_size[1]/2, clip_size[2]/2]
leader_post_clip = [-0.05, 0, 0]  # [0.575, -0.081, 1.128]  # [0.552, 0.069, 1.128]
follower_post_clip = [0.05, 0, 0]  # [0.552, 0.069, 1.128]  # [0.575, -0.081, 1.128]


# Distance from the eef mount to the palm of end effector [x, y, z, r, p, y]
# copied from https://github.com/ros-planning/moveit_grasps/blob/melodic-devel/config_robot/panda_grasp_data.yaml
# z-axis pointing toward object to grasp
# x-axis perp. to movement of grippers
# y-axis parallel to movement of grippers
# t_TCPtoEE =  [0, 0, -0.105, 0, 0, -0.7853]
# EE_in_TCP = np.array([[0], [0], [-0.105], [1]])

# R_RtoW = np.array([[1, 0, 0],
#                    [0, 1, 0],
#                    [0, 0, 1]])

# T_R1toW_3d= np.concatenate((R_RtoW, np.reshape(np.array([0, 0.4, 1.0]), (3, 1))), axis=1)
# T_R1toW_3d = np.concatenate((T_R1toW_3d, np.array([[0, 0, 0, 1]])), axis=0)

# T_R2toW_3d= np.concatenate((R_RtoW, np.reshape(np.array([0.0, -0.4, 1.0]), (3, 1))), axis=1)
# T_R2toW_3d = np.concatenate((T_R2toW_3d, np.array([[0, 0, 0, 1]])), axis=0)


def msg_orientation_to_matrix(orient):
        rotation = R.from_quat([orient.x, orient.y, orient.z, orient.w])
        # print(rotation.as_matrix())
        # print(tf.transformations.quaternion_matrix([orient.x, orient.y, orient.z, orient.w]))
        return rotation.as_matrix()
    
def matrix_to_msg_orientation(orient_matrix):
    rotation = R.from_matrix(orient_matrix)
    quat = rotation.as_quat()
        
    orient = geometry_msgs.msg.Quaternion()
    orient.x = quat[0]
    orient.y = quat[1]
    orient.z = quat[2]
    orient.w = quat[3]
    return orient

def vector_angle(vector):
	'''heading direction of the vector'''
	origin = np.zeros(vector.size)
	origin[0] = 1
	unit_vector = vector/ np.linalg.norm(vector)
	unit_origin = origin/ np.linalg.norm(origin)
	dot_product = np.dot(unit_vector , unit_origin)
	angle = np.arccos(dot_product)  
	return angle


def create_clip_goal(goal_frame, goal_translation_vector):
    goal_pose = PoseStamped()

    goal_pose.pose.position.x = goal_translation_vector[0]  # 0.05
    goal_pose.pose.position.y = goal_translation_vector[1]  # 0.0
    goal_pose.pose.position.z = goal_translation_vector[2]  # 0.0

    # orientation from clip frame to robot ee frame
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.9999997
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 0.0007963

    goal_pose.header.frame_id = goal_frame

    return goal_pose
    

class TaskPlanner():
    def __init__(self, clip_information, clip_path) -> None:        
        # load clip information
        with open(clip_information, 'rb') as clip_file:
            self.clip_info = pickle.load(clip_file)
            
        # initialize todo and passed clip list
        self.clip_path = clip_path
        self.clip_togo = copy.deepcopy(clip_path)
        self.clip_achieved = []

        # publisher to the cpp backend of mtc
        self.publisher = rospy.Publisher('mtc_front2back_chatter', moveit_msgs.msg.MTCPlanGoal, queue_size=10)
        # rospy.init_node('moveit_talker', anonymous=True)
        
        # maker publisher
        self.marker_pub = rospy.Publisher('interactive_robot_markers', Marker, queue_size=1)

        '''task constructor configuration'''
        # groups
        self.dual_arm_group = "dual_arm"
        self.follow_arm_group = "panda_2"
        self.lead_arm_group = "panda_1"
        self.follow_hand_group = "hand_2"
        self.lead_hand_group = "hand_1"
        
        # # planning pipeline
        # self.cartesian_pipeline = core.CartesianPath()
        # self.jointspace_pipeline = core.JointInterpolationPlanner()
        
        # self.lead_pipeline = core.PipelinePlanner()
        # self.lead_pipeline.planner = "RRTConnect"
        
        # self.follow_pipeline = core.PipelinePlanner()
        # self.follow_pipeline.planner = "RRTConnect"
        
        # self.dual_pipeline = core.PipelinePlanner()
        # self.dual_pipeline.planner = "RRTConnect"
        
        # pose and parameters
        self.hand_open_pose_ = "open"
        self.hand_close_pose_ = "close"
    
    def setup_planning_piplines(self):
        # planning pipeline
        self.cartesian_pipeline = core.CartesianPath()
        self.jointspace_pipeline = core.JointInterpolationPlanner()
        
        self.lead_pipeline = core.PipelinePlanner()
        self.lead_pipeline.planner = "RRTConnect"
        
        self.follow_pipeline = core.PipelinePlanner()
        self.follow_pipeline.planner = "RRTConnect"
        
        self.dual_pipeline = core.PipelinePlanner()
        self.dual_pipeline.planner = "RRTConnect"
        
    def create_task(self, goal_frame_name, use_constraint=False):
        t = core.Task()
        t.loadRobotModel()
        
        # current stats
        t.add(stages.CurrentState("current state"))
        
        # CAUTION: planning pipelines setup has to be done after getting current state
        # otherwise, the robot model is not loaded correctly
        self.setup_planning_piplines()
        
        # initialize container
        grasp = core.SerialContainer("pick object")
        
        ''' Close Leader Hand'''
        stage = stages.MoveTo("close hand", self.lead_pipeline)
        stage.group = self.lead_hand_group
        stage.setGoal(self.hand_close_pose_)
        grasp.insert(stage)
        
        ''' Open Follower Hand'''
        stage = stages.MoveTo("open hand", self.follow_pipeline)
        stage.group = self.follow_hand_group
        stage.setGoal(self.hand_open_pose_)
        # pre_move_stage = stage.get()
        grasp.insert(stage)
        
        ''' Connec to Next Stage'''
        connect = stages.Connect("connect", [(self.lead_arm_group, self.lead_pipeline), (self.follow_arm_group, self.follow_pipeline)])
        connect.properties.configureInitFrom(core.Stage.PropertyInitializerSource.PARENT)
        connect.max_distance = 1e-3
        # props.configureInitFrom(core.Stage.PropertyInitializerSource.PARENT, ["target_pose"])   
        grasp.insert(connect)
        
        '''Spawn IK on Fixed Pose for Dual Arm'''
        # target positions in clip frames
        lead_goal_pose = create_clip_goal(goal_frame_name, leader_pre_clip)
        # self.append_frame_markers(lead_goal_pose, "leader_goal_frame")
        follow_goal_pose = create_clip_goal(goal_frame_name, follower_pre_clip)
        # self.append_frame_markers(follow_goal_pose, "follower_goal_frame")

        # Create goal delta vectors
        lead_goal_delta_vector = [lead_goal_pose.pose.position.x, lead_goal_pose.pose.position.y, lead_goal_pose.pose.position.z]
        follow_goal_delta_vector = [follow_goal_pose.pose.position.x, follow_goal_pose.pose.position.y, follow_goal_pose.pose.position.z]
        delta_pairs = {self.lead_arm_group: lead_goal_delta_vector, self.follow_arm_group: follow_goal_delta_vector}
        pre_grasp_pose = {self.lead_arm_group: "close", self.follow_arm_group: "open"}

        # Set IK frame at TCP
        ik_links = {self.lead_arm_group: "panda_1_hand", self.follow_arm_group: "panda_2_hand"}
        ik_frame_1 = PoseStamped()
        ik_frame_1.header.frame_id = ik_links[self.lead_arm_group]
        ik_frame_1.pose = Pose(position=Vector3(z=0.1034))
        ik_frame_2 = PoseStamped()
        ik_frame_2.header.frame_id = ik_links[self.follow_arm_group]
        ik_frame_2.pose = Pose(position=Vector3(z=0.1034))

        # Create lists and dictionaries
        ik_groups = [self.lead_arm_group, self.follow_arm_group]
        ik_endeffectors = {self.lead_arm_group: self.lead_hand_group, self.follow_arm_group: self.follow_hand_group}
        ik_frames = {self.lead_arm_group: ik_frame_1, self.follow_arm_group: ik_frame_2}
        
        # generate grasping cartesian pose, fixed pose for leader arm, random for follower arm
        generator = stages.GenerateGraspPoseDual("generate grasp pose dual", ik_groups)
        generator.eefs = ik_endeffectors
        generator.marker_ns = "grasp_pose"
        generator.explr_axis = "y"
        generator.angle_delta = 0.2
        generator.pregrasps = pre_grasp_pose
        generator.grasp = "close"
        generator.object = goal_frame_name
        generator.target_deltas = delta_pairs
        generator.setMonitoredStage(grasp["open hand"])
        generator.generate_group = self.follow_arm_group
        
        # generate joint positions for both robots from IK solver
        ik_wrapper = stages.ComputeIKMultiple("move IK dual", generator, ik_groups, self.dual_arm_group)
        ik_wrapper.groups = ik_groups
        ik_wrapper.group = self.dual_arm_group
        ik_wrapper.eefs = ik_endeffectors
        ik_wrapper.properties.configureInitFrom(core.Stage.PropertyInitializerSource.INTERFACE, ["target_poses"])
        ik_wrapper.max_ik_solutions = 100
        ik_wrapper.min_solution_distance = 1.0
        ik_wrapper.setIKFrame(ik_frames)
        
        grasp.insert(ik_wrapper)
        
        '''Close Follower Hand'''
        stage = stages.MoveTo("close hand", self.follow_pipeline)
        stage.group = self.follow_hand_group
        stage.setGoal(self.hand_close_pose_)
        grasp.insert(stage)
        
        t.add(grasp)
        
        return t

        
    def create_task_home(self):
        t = core.Task()
        
        # current stats
        t.add(stages.CurrentState("current state"))
        
        '''Homing'''
        stage = stages.MoveTo("back home", self.jointspace_pipeline)
        stage.group = self.dual_arm_group
        stage.setGoal("home")
        t.add(stage)
        
    def publish_goal_marker(self, goal_position_array, id=0, color=[1.0, 0.0, 0,0], timeout=2):
        # remove old marker
        marker_delete = Marker()
        marker_delete.header.stamp = rospy.Time.now()
        marker_delete.id = id
        marker_delete.action = Marker.DELETE
        
        # Publish the marker
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            self.marker_pub.publish(marker_delete)
            rospy.rostime.wallsleep(1.0)
            seconds = rospy.get_time()
        
        # Create a new Marker message
        marker = Marker()
        marker.header.frame_id = "base"  # Set the planning frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_marker"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # Set the scale according to your preference
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.r = color[0]  # Set color (red in this case)
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.5  # Alpha (transparency)
        
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = goal_position_array[0]
        marker.pose.position.y = goal_position_array[1]
        marker.pose.position.z = goal_position_array[2]
        
        marker.lifetime = rospy.Duration()

        # Publish the marker
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            self.marker_pub.publish(marker)
            rospy.rostime.wallsleep(1.0)
            seconds = rospy.get_time()
        
    def append_frame_markers(self, pose_stamped, frame_name, timeout=2):
        # Append frame markers
        object_markers = []

        # rviz_marker_tools.appendFrame is not available in Python. You need to implement it yourself or find a Python equivalent.

        marker_array_msg = MarkerArray()
        id = 0
        for marker in object_markers:
            mutable_marker = Marker()
            mutable_marker = marker  # Make a non-const copy
            mutable_marker.id = id
            id += 1
            marker_array_msg.markers.append(mutable_marker)

        timeout = 2.0
        start = rospy.get_time()
        end = start
        while (end - start) < timeout and not rospy.is_shutdown():
            self.marker_pub.publish(marker_array_msg)
            rospy.sleep(1.0)
            end = rospy.get_time()
        

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Arguments for fixture scene loading')
    parser.add_argument('-c','--clip_file', default='/home/tp2/Documents/mios-wiring/clip_info20240130_transform.pickle')
    # parser.add_argument('--board_base_size', default='store_true', help='Skip unsetting previous environment variables to extend context')
    # parser.add_argument('--board_size', action='store_true', help='Only consider this prefix path and ignore other prefix path in the environment')
    args = parser.parse_args()
    
    roscpp_init("mtc_tutorial")
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    # moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("mtc_task_planner", anonymous=True)
    
        
    # clear previous planning
    rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
    clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
    clear_octomap()
    
    robot1_current_joint = [-1.02680829,  0.44764564,  0.18002864, -2.30666668, -0.24687706,  2.77595507, 1.41033604]
    robot2_current_joint = [-0.03919738,  0.16151273, -0.21564032, -2.65617219,  0.05404735,  2.8080896, 1.55379734]
    
    task_planner = TaskPlanner(clip_information=args.clip_file, clip_path=[7, 6, 9])
    for clip_id in task_planner.clip_togo:
        goal_frame = f"clip{clip_id}"
        task = task_planner.create_task(goal_frame)
        
        # try:
        #     if task.plan():
        #         rospy.logwarn("Executing solution trajectory")
        #         task.publish(task.solutions[0])
        #         execute_result = task.execute(task.solutions[0])
        #         task_planner.clip_togo.remove(clip_id)
        #         task_planner.clip_achieved.append(clip_id)
        #         if execute_result.val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
        #             rospy.logerr("Task execution failed and returned: %s", execute_result.val)
        # except Exception as e:
        #     print("planning failed with exception\n", e)
        #     continue
        
        if task.plan():
            rospy.logwarn("Executing solution trajectory")
            # task.publish(task.solutions[0])
            execute_result = task.execute(task.solutions[0])
    
        # while not rospy.is_shutdown():
        #     rospy.sleep(1)
    
    rospy.spin()
    
    