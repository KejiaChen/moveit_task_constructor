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
import threading
import moveit_task_constructor_msgs.msg
from moveit_msgs.srv import GetMiosPlan
import shape_msgs.msg

from py_binding_tools import roscpp_init

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

def get_cable_grasping_pose(common_frame, cable_start_pose, cable_end_pose):
    tf_listener = tf.TransformListener()

    # Wait for the listener to get the first transform
    tf_listener.waitForTransform(cable_start_pose.header.frame_id, common_frame, rospy.Time(0), rospy.Duration(4.0))
    tf_listener.waitForTransform(cable_end_pose.header.frame_id, common_frame, rospy.Time(0), rospy.Duration(4.0))

    # Transform both poses to the common frame
    pose1_transformed = tf_listener.transformPose(common_frame, cable_start_pose)
    pose2_transformed = tf_listener.transformPose(common_frame, cable_end_pose)
    
    # Calculate midpoint of positions
    mid_x = (pose1_transformed.pose.position.x + pose2_transformed.pose.position.x) / 2
    mid_y = (pose1_transformed.pose.position.y + pose2_transformed.pose.position.y) / 2
    mid_z = (pose1_transformed.pose.position.z + pose2_transformed.pose.position.z) / 2
    
    return [mid_x, mid_y, mid_z]

def construct_plane_constraint(link_name, frame_id):
    constraints = moveit_msgs.msg.Constraints()
    pos_constraint = geometry_msgs.msg.PositionConstraint()
    pos_constraint.link_name = link_name
    pos_constraint.header.frame_id = frame_id
    pos_constraint.target_point_offset.z = 0.1034

    primitive = shape_msgs.msg.SolidPrimitive()
    primitive.type = shape_msgs.msg.SolidPrimitive.BOX
    primitive.dimensions = [0]*3
    primitive.dimensions[shape_msgs.msg.SolidPrimitive.BOX_X] = 3.0
    primitive.dimensions[shape_msgs.msg.SolidPrimitive.BOX_Y] = 3.0
    primitive.dimensions[shape_msgs.msg.SolidPrimitive.BOX_Z] = 0.09

    box_pose = Pose()
    box_pose.orientation.w = 1.0
    box_pose.position.z = 0.0375

    pos_constraint.constraint_region.primitives.append(primitive)
    pos_constraint.constraint_region.primitive_poses.append(box_pose)
    pos_constraint.weight = 1.0

    constraints.position_constraints.append(pos_constraint)

    return constraints

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
        
        '''configuration'''
        # groups
        self.dual_arm_group = "dual_arm"
        self.dual_hand_group = "dual_hand"
        self.follow_arm_group = "panda_2"
        self.lead_arm_group = "panda_1"
        self.follow_hand_group = "hand_2"
        self.lead_hand_group = "hand_1"
        self.follow_end_effector_link = "panda_2_link8"
        self.lead_end_effector_link = "panda_1_link8"
        self.previouse_goal_frame = None
        
        # pose and parameters
        self.hand_open_pose_ = "open"
        self.hand_close_pose_ = "close"
        
        '''move groups initialization'''
        self.follow_arm_move_group = moveit_commander.MoveGroupCommander(self.follow_arm_group)
        self.follow_hand_move_group = moveit_commander.MoveGroupCommander(self.follow_hand_group)
        self.lead_arm_move_group = moveit_commander.MoveGroupCommander(self.lead_arm_group)
        self.lead_hand_move_group = moveit_commander.MoveGroupCommander(self.lead_hand_group)
        
        self.dual_arm_move_group = moveit_commander.MoveGroupCommander(self.dual_arm_group)
        self.dual_hand_move_group = moveit_commander.MoveGroupCommander(self.dual_hand_group)

        '''exchange with mios'''
        # publisher to mios
        self.publisher = rospy.Publisher('moveit2mios_chatter', std_msgs.msg.Bool)
        # rospy.init_node('moveit_talker', anonymous=True)
        
        # Lock for synchronizing access to move group by two subscribers
        self.lock = threading.Lock()
        self.is_planning = False
        self.real_world_joint_positions = None
        self.real_world_gripper_widths = None
        self.keep_running = True
        
        self.execution_thread = threading.Thread(target=self.robot_status_callback_execution)
        self.execution_thread.start()
        
        self.leader_traj_host = '10.157.174.87'
        self.leader_traj_port = 12345
        self.follower_traj_host = '10.157.174.101'
        self.follower_traj_port = 12345
        
        '''trajectory client'''
        # self.client_socket = socket.socket()
        
        '''Initialize Sockets'''
        # self.leader_client = socket.socket()  # instantiate
        # try:
        #     self.leader_client.connect((self.leader_traj_host, self.leader_traj_port))
        #     rospy.loginfo("Connected to leader client")
        # except socket.error as e:
        #     rospy.logerr("Failed to connect leader client: %s", e)
            
        # self.follower_client = socket.socket()
        # try:
        #     self.follower_client.connect((self.follower_traj_host, self.follower_traj_port))
        #     rospy.loginfo("Connected to follower client")
        # except socket.error as e:
        #     rospy.logerr("Failed to connect follower client: %s", e)          
        
    # def __del__(self):
    #     # close sockets
    #     # self.leader_client.close()
    #     # self.follower_client.close()
    #     rospy.loginfo("close sockets")
            
    #     print("task planner is deleted")
    
    def test_dual_joint_goal(self, follow_joint_goal:list, lead_joint_goal:list):
        self.dual_arm_move_group.set_joint_value_target(follow_joint_goal+lead_joint_goal)
        return self.dual_arm_move_group.plan()
    
    def test_dual_pose_goal(self):
        lead_pose_goal =self.lead_arm_move_group.get_current_pose(self.lead_end_effector_link)
        lead_pose_goal.pose.position.z -= 0.15
        lead_pose_goal.pose.position.x += 0.15

        follow_pose_goal = self.follow_arm_move_group.get_current_pose(self.follow_end_effector_link)
        follow_pose_goal.pose.position.z -= 0.15
        follow_pose_goal.pose.position.x -= 0.15
        
        self.dual_arm_move_group.set_start_state_to_current_state()
        self.dual_arm_move_group.set_pose_target(follow_pose_goal, end_effector_link=self.follow_end_effector_link)
        self.dual_arm_move_group.set_pose_target(lead_pose_goal, end_effector_link=self.lead_end_effector_link)
        
        self.dual_arm_move_group.plan()
        self.dual_arm_move_group.go(wait=True)
    
    def test_dual_hand(self, follow_hand_goal:list, lead_hand_goal:list):
        self.dual_hand_move_group.set_joint_value_target(follow_hand_goal+lead_hand_goal)
        self.dual_hand_move_group.go(wait=True)
    
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
    
    def subscribe(self):
        # # subscriber to real-world joint states
        # self.joint_status_subscriber = rospy.Subscriber("mios_combined_joint_positions", moveit_msgs.msg.FromMiosJointStatus, self.joint_status_callback)
        # rospy.loginfo("start subscribe to joint state")
        
        # subscriber to real-world robot states
        self.joint_status_subscriber = rospy.Subscriber("mios_combined_joint_positions", moveit_msgs.msg.FromMiosRobotStatus, self.robot_status_callback)
        rospy.loginfo("start subscribe to robot state")
        
        # subscriber to planning request
        # self.plan_request_subscriber = rospy.Subscriber("mios2moveit_chatter", moveit_msgs.msg.FromMios, self.plan_callback)
        # rospy.loginfo("start subscribe to planning request")
        
        # service to planning request
        self.plan_request_server = rospy.Service('mios_plan_request_service', GetMiosPlan, self.plan_callback)
        rospy.loginfo("start server to handle planning request from mios")
        
        # # subscriber to planning execution
        # self.plan_execution_subscriber = rospy.Subscriber("mios2moveit_execution", std_msgs.msg.Bool, self.plan_execution_callback)
        
    def joint_status_callback_execution(self):
        '''callback function to synchronize move group joint states with real world'''
        # rospy.loginfo("synchronization")
        while self.keep_running and not rospy.is_shutdown():
            if self.real_world_joint_positions and not self.is_planning:
                with self.lock:
                    # self.dual_arm_move_group.set_joint_value_target(lead_current_joint+follow_current_joint)
                    # print("joint states", self.real_world_joint_positions)
                    self.dual_arm_move_group.go(self.real_world_joint_positions, wait=True)
                    self.real_world_joint_positions = None
                    
    def robot_status_callback_execution(self):
        '''callback function to synchronize move group joint states with real world'''
        # rospy.loginfo("synchronization")
        while self.keep_running and not rospy.is_shutdown():
            if self.real_world_joint_positions and not self.is_planning:
                with self.lock:
                    # self.dual_arm_move_group.set_joint_value_target(lead_current_joint+follow_current_joint)
                    # print("joint states", self.real_world_joint_positions)
                    self.dual_arm_move_group.go(self.real_world_joint_positions, wait=True)
                    self.real_world_joint_positions = None
            if self.real_world_gripper_widths and not self.is_planning:
                with self.lock:
                    # print("gripper states", self.real_world_gripper_widths)
                    self.dual_hand_move_group.go(self.real_world_gripper_widths, wait=True)
                    self.real_world_gripper_widths = None
                   
    def joint_status_callback(self, data):
        '''callback function to update joint states from real world'''
        with self.lock:
            lead_current_joint = data.robot1_current_joint
            follow_current_joint = data.robot2_current_joint
            self.real_world_joint_positions = lead_current_joint + follow_current_joint
            
    def robot_status_callback(self, data):
        '''callback function to update joint states from real world'''
        with self.lock:
            lead_current_joint = data.robot1_current_joint
            follow_current_joint = data.robot2_current_joint
            self.real_world_joint_positions = lead_current_joint + follow_current_joint
            self.real_world_gripper_widths = (data.robot1_gripper_width/2, data.robot1_gripper_width/2, data.robot2_gripper_width/2, data.robot2_gripper_width/2)
            
    # def plan_execution_callback(self, data):
    #     '''callback function to execute the planned trajectory'''
    #     with self.lock:
    #         if data == True:
    #             rospy.loginfo("executing finishes")
    #             self.is_planning = False
    
    def plan_callback(self, data):
        with self.lock:
            rospy.loginfo(rospy.get_caller_id() + " I heard %s", data)
            self.is_planning = True
            
            leader_pre_clip_pose = data.mios_plan_request.leader_goal_in_clip
            leader_pre_clip_vec = [leader_pre_clip_pose.pose.position.x, leader_pre_clip_pose.pose.position.y, leader_pre_clip_pose.pose.position.z]
            follower_pre_clip_pose = data.mios_plan_request.follower_goal_in_clip
            follower_pre_clip_vec = [follower_pre_clip_pose.pose.position.x, follower_pre_clip_pose.pose.position.y, follower_pre_clip_pose.pose.position.z]
            goal_frame = f"clip{data.mios_plan_request.clip_id}"
            
            msg_to_mios = moveit_msgs.msg.MiosPlanResponse()
            
            '''Initialize Sockets'''
            # leader_client = socket.socket()  # instantiate
            # leader_client.connect((self.leader_traj_host, self.leader_traj_port))
            # follower_client = socket.socket()
            # follower_client.connect((self.follower_traj_host, self.follower_traj_port))                        
             
            '''Synchronize '''
            # self.synchronize(lead_current_joint, follow_current_joint, fixed_clip)
            # lead_current_pose = self.lead_arm_move_group.get_current_pose().pose
            # rotation_matrix_r1 = msg_orientation_to_matrix(lead_current_pose.orientation)
            
            '''Plan'''
            try: 
                task = self.create_task(goal_frame, leader_pre_clip_pose, follower_pre_clip_pose)
                if task.plan():
                    rospy.logwarn("Executing solution trajectory")
                    # task.publish(task.solutions[0])
                    
                    solution = task.solutions[0]
                    execute_result = task.execute(solution)
                        
                    # execute_goal = moveit_task_constructor_msgs.Solution 
                    execute_goal = task.solutions[0].toMsg(task.getIntrospection())
                    msg_to_mios.success = True
                    traj_collection = execute_goal.sub_trajectory
                    
                    self.is_planning = False
                    
                    for traj_msg in traj_collection:
                        traj_list = []
                        solution_id = traj_msg.info.stage_id
                        joint_names = traj_msg.trajectory.joint_trajectory.joint_names
                        joint_trajectory = traj_msg.trajectory.joint_trajectory.points
                        print(str(solution_id) + ": ")
                        print(' '.join('%s' % x for x in joint_names))
                        if joint_names:
                            if "finger_joint" in joint_names[0]:
                                continue
                            else:
                                real_world_file_path = os.path.join(os.path.dirname(__file__), 'saved_trajectories', 'real_world_traj_task_'+str(solution_id)+'.txt')
                                # with open(real_world_file_path, 'w+') as f:
                                for point in joint_trajectory:
                                    traj_list.append(list(point.positions))
                                #         f.write(' '.join('%s' % x for x in point.positions))
                                #         # f.write(' ')
                                #         # f.write(' '.join('%s' % x for x in point.velocities))
                                #         f.write(' \n')
                                
                                # smoothing
                                traj = np.array(traj_list)
                                smooth_traj = rtb.tools.mstraj(traj, dt=0.001, tacc=0, qdmax=0.5)
                                
                                '''Send trajectories to robots'''
                                # client_socket = socket.socket()  # instantiate
                                
                                # Skip finger joints and Send arm trajectories to robots
                                if "panda_1" in joint_names[0]:
                                    response_robot_id = 1
                                    # client_socket = leader_client
                                    server_host = self.leader_traj_host
                                    server_port = self.leader_traj_port
                                elif "panda_2" in joint_names[0]:
                                    response_robot_id = 2
                                    # client_socket = follower_client
                                    server_host = self.follower_traj_host
                                    server_port = self.follower_traj_port
                                
                                if msg_to_mios.robot_id:
                                    msg_to_mios.robot_id.append(response_robot_id)
                                else:
                                    msg_to_mios.robot_id = [response_robot_id]
                                if msg_to_mios.traj_id:
                                    msg_to_mios.traj_id.append(solution_id)
                                else:
                                    msg_to_mios.traj_id = [solution_id]
                                
                                # # client_socket.connect((server_host, server_port))
                                # # send the write or read command
                                # command = "write"
                                # client_socket.send(f"{command}".encode())
                                # # send the name of the file
                                # smooth_file_path = os.path.join(os.path.dirname(__file__), 'saved_trajectories', 'smooth_real_world_traj_'+str(solution_id)+'.txt')
                                # smooth_file_name = os.path.basename(smooth_file_path)
                                # rospy.loginfo("file name %s", smooth_file_name)
                                # client_socket.send(f"{os.path.basename(smooth_file_path)}".encode())
                                # rospy.loginfo("send file name to mios at %s", server_host)
                                # # send the trajectory
                                # joint_traj_data = pickle.dumps(smooth_traj.q)
                                # client_socket.sendall(joint_traj_data)
                                # rospy.loginfo("send smooth trajectory to mios at %s", server_host)
                                # # client_socket.close()
                    
            except Exception as ex:
                rospy.logerr("planning failed with exception\n%s%s", ex, task)
            
        # close sockets
        # leader_client.close()
        # follower_client.close()
        # rospy.loginfo("close sockets")
        
        rospy.loginfo("response is %s", msg_to_mios)
            
        return msg_to_mios
    
    def create_task_single(self, goal_frame_name, lead_goal_pose, follow_goal_pose, use_constraint=False):
        t = core.Task()
        t.loadRobotModel()
        
        # Set IK frame at TCP
        ik_links = {self.lead_arm_group: "panda_1_hand", self.follow_arm_group: "panda_2_hand"}
        ik_frame_1 = PoseStamped()
        ik_frame_1.header.frame_id = ik_links[self.lead_arm_group]
        ik_frame_1.pose = Pose(position=Vector3(z=0.1034))
        ik_frame_2 = PoseStamped()
        ik_frame_2.header.frame_id = ik_links[self.follow_arm_group]
        ik_frame_2.pose = Pose(position=Vector3(z=0.1034))
        
        # current stats
        current_stage = stages.CurrentState("current state")
        t.add(current_stage)
        
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
        
        '''Move Leader Arm to Pre Clip Position'''
        pre_lead = stages.MoveTo("move leader to clip", self.lead_pipeline)
        pre_lead.group = self.lead_arm_group
        pre_lead.ik_frame = ik_frame_1
        pre_lead.setGoal(lead_goal_pose)
        grasp.insert(pre_lead)
        
        ''' Connec to Next Stage'''
        connect = stages.Connect("connect", [(self.lead_arm_group, self.lead_pipeline), (self.follow_arm_group, self.follow_pipeline)])
        connect.properties.configureInitFrom(core.Stage.PropertyInitializerSource.PARENT)
        connect.max_distance = 1e-3
        # props.configureInitFrom(core.Stage.PropertyInitializerSource.PARENT, ["target_pose"])   
        grasp.insert(connect)
        
        '''Spawn IK on Fixed Pose for Dual Arm'''
        # target positions in clip frames
        # lead_goal_pose = create_clip_goal(goal_frame_name, leader_pre_clip)
        # self.append_frame_markers(lead_goal_pose, "leader_goal_frame")
        # follow_goal_pose = create_clip_goal(goal_frame_name, follower_pre_clip)
        # self.append_frame_markers(follow_goal_pose, "follower_goal_frame")

        # Create goal delta vectors
        lead_goal_delta_vector = [lead_goal_pose.pose.position.x, lead_goal_pose.pose.position.y, lead_goal_pose.pose.position.z]
        follow_goal_delta_vector = [follow_goal_pose.pose.position.x, follow_goal_pose.pose.position.y, follow_goal_pose.pose.position.z]
        delta_pairs = {self.lead_arm_group: lead_goal_delta_vector, self.follow_arm_group: follow_goal_delta_vector}
        pre_grasp_pose = {self.lead_arm_group: "close", self.follow_arm_group: "open"}
        goal_frames = {self.lead_arm_group: lead_goal_pose.header.frame_id, self.follow_arm_group: follow_goal_pose.header.frame_id}

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
        generator.objects = goal_frames
        generator.target_deltas = delta_pairs
        # generator.setMonitoredStage(grasp["open hand"])
        generator.setMonitoredStage(grasp["move leader to clip"])
        generator.generate_group = self.follow_arm_group
        generator.planning_frame = goal_frame_name
        
        # generate joint positions for both robots from IK solver
        ik_wrapper = stages.ComputeIKMultiple("move IK dual", generator, ik_groups, self.dual_arm_group)
        ik_wrapper.groups = ik_groups
        ik_wrapper.group = self.dual_arm_group
        ik_wrapper.eefs = ik_endeffectors
        ik_wrapper.properties.configureInitFrom(core.Stage.PropertyInitializerSource.INTERFACE, ["target_poses"])
        ik_wrapper.max_ik_solutions = 10
        ik_wrapper.min_solution_distance = 1.0
        ik_wrapper.setIKFrame(ik_frames)
        
        # set cost
        cost_cumulative = False
        cost_with_world = False
        cost_group_property = "group"
        cost_mode = core.TrajectoryCostTerm.Mode.AUTO
        cl_cost = core.Clearance(cost_with_world, cost_cumulative,cost_group_property, cost_mode)
        ik_wrapper.setCostTerm(cl_cost)
        
        grasp.insert(ik_wrapper)
        
        '''Close Follower Hand'''
        stage = stages.MoveTo("close hand", self.follow_pipeline)
        stage.group = self.follow_hand_group
        stage.setGoal(self.hand_close_pose_)
        grasp.insert(stage)
        
        t.add(grasp)
        
        return t
    
    def create_task(self, goal_frame_name, lead_goal_pose, follow_goal_pose, use_constraint=False):
        t = core.Task()
        t.loadRobotModel()
        
        # Set IK frame at TCP
        ik_links = {self.lead_arm_group: "panda_1_hand", self.follow_arm_group: "panda_2_hand"}
        ik_frame_1 = PoseStamped()
        ik_frame_1.header.frame_id = ik_links[self.lead_arm_group]
        ik_frame_1.pose = Pose(position=Vector3(z=0.1034))
        ik_frame_2 = PoseStamped()
        ik_frame_2.header.frame_id = ik_links[self.follow_arm_group]
        ik_frame_2.pose = Pose(position=Vector3(z=0.1034))
        
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
        grasp.insert(stage)
        
        # '''Move Leader Arm to Pre Clip Position'''
        # pre_lead = stages.MoveTo("move leader to clip", self.lead_pipeline)
        # pre_lead.group = self.lead_arm_group
        # pre_lead.ik_frame = ik_frame_1
        # pre_lead.setGoal(create_clip_goal(goal_frame_name, leader_pre_clip))
        # grasp.insert(pre_lead)
        
        ''' Connec to Next Stage'''
        connect = stages.Connect("connect", [(self.lead_arm_group, self.lead_pipeline), (self.follow_arm_group, self.follow_pipeline)])
        # connect = stages.Connect("connect",[(self.dual_arm_group, self.dual_pipeline)])
        connect.properties.configureInitFrom(core.Stage.PropertyInitializerSource.PARENT)
        connect.max_distance = 1e-3
        # props.configureInitFrom(core.Stage.PropertyInitializerSource.PARENT, ["target_pose"])   
        grasp.insert(connect)
        
        '''Spawn IK on Fixed Pose for Dual Arm'''
        # target positions in clip frames
        # lead_goal_pose = create_clip_goal(goal_frame_name, leader_pre_clip)
        # self.append_frame_markers(lead_goal_pose, "leader_goal_frame")
        # follow_goal_pose = create_clip_goal(goal_frame_name, follower_pre_clip)
        # self.append_frame_markers(follow_goal_pose, "follower_goal_frame")

        # Create goal delta vectors
        lead_goal_delta_vector = [lead_goal_pose.pose.position.x, lead_goal_pose.pose.position.y, lead_goal_pose.pose.position.z]
        follow_goal_delta_vector = [follow_goal_pose.pose.position.x, follow_goal_pose.pose.position.y, follow_goal_pose.pose.position.z]
        delta_pairs = {self.lead_arm_group: lead_goal_delta_vector, self.follow_arm_group: follow_goal_delta_vector}
        pre_grasp_pose = {self.lead_arm_group: "close", self.follow_arm_group: "open"}
        goal_frames = {self.lead_arm_group: lead_goal_pose.header.frame_id, self.follow_arm_group: follow_goal_pose.header.frame_id}

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
        generator.objects = goal_frames
        generator.target_deltas = delta_pairs
        generator.setMonitoredStage(grasp["open hand"])
        # generator.setMonitoredStage(grasp["move leader to clip"])
        generator.generate_group = self.follow_arm_group
        generator.planning_frame = goal_frame_name
        
        # generate joint positions for both robots from IK solver
        ik_wrapper = stages.ComputeIKMultiple("move IK dual", generator, ik_groups, self.dual_arm_group)
        ik_wrapper.groups = ik_groups
        ik_wrapper.group = self.dual_arm_group
        ik_wrapper.eefs = ik_endeffectors
        ik_wrapper.properties.configureInitFrom(core.Stage.PropertyInitializerSource.INTERFACE, ["target_poses"])
        ik_wrapper.max_ik_solutions = 100
        ik_wrapper.min_solution_distance = 1.0
        ik_wrapper.setIKFrame(ik_frames)
        
        # set cost
        cost_cumulative = False
        cost_with_world = False
        cost_group_property = "group"
        cost_mode = core.TrajectoryCostTerm.Mode.AUTO
        cl_cost = core.Clearance(cost_with_world, cost_cumulative,cost_group_property, cost_mode)
        ik_wrapper.setCostTerm(cl_cost)
        
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
        
def test_shutdown():
    rospy.loginfo("Shutting down")

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
    
    rospy.on_shutdown(test_shutdown)
        
    # clear previous planning
    rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
    clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
    clear_octomap()
    
    robot1_current_joint = [-1.02680829,  0.44764564,  0.18002864, -2.30666668, -0.24687706,  2.77595507, 1.41033604]
    robot2_current_joint = [-0.03919738,  0.16151273, -0.21564032, -2.65617219,  0.05404735,  2.8080896, 1.55379734]
    
    # in clip frame
    clip_size = [0.04, 0.04, 0.06]
    hold_x_offset = 0.03
    leader_pre_clip = [-(clip_size[0]/2+hold_x_offset), -clip_size[1]/2, clip_size[2]/2]
    follower_pre_clip = [clip_size[0]/2+hold_x_offset, -clip_size[1]/2, clip_size[2]/2]
    leader_post_clip = [-0.05, 0, 0]  # [0.575, -0.081, 1.128]  # [0.552, 0.069, 1.128]
    follower_post_clip = [0.05, 0, 0]  # [0.552, 0.069, 1.128]  # [0.575, -0.081, 1.128]
    
    task_planner = TaskPlanner(clip_information=args.clip_file, clip_path=[7, 6, 9]) #7, 6, 9
    
    '''Mios Plan Service'''
    # task_planner.test_dual_hand([0.02, 0.02], [0.02, 0.02])
    # task_planner.subscribe()
    
    # subscriber to real-world robot states
    joint_status_subscriber = rospy.Subscriber("mios_combined_joint_positions", moveit_msgs.msg.FromMiosRobotStatus, task_planner.robot_status_callback)
    rospy.loginfo("start subscribe to robot state")
        
    # # service to planning request
    plan_request_server = rospy.Service('mios_plan_request_service', GetMiosPlan, task_planner.plan_callback)
    rospy.loginfo("start server to handle planning request from mios")
    
    '''MTC Direct Test'''
    # previouse_goal_frame = None
    # for clip_id in task_planner.clip_togo:
    #     goal_frame = f"clip{clip_id}"
    #     if previouse_goal_frame is None:
    #         leader_pre_pose = create_clip_goal(goal_frame, leader_pre_clip)
    #         follower_pre_pose = create_clip_goal(goal_frame, follower_pre_clip)
    #     else:
    #         # follower_pre_clip = get_cable_grasping_pose(goal_frame, create_clip_goal(previouse_goal_frame, leader_pre_clip), create_clip_goal(goal_frame, follower_pre_clip))
    #         follower_pre_pose = create_clip_goal(previouse_goal_frame, leader_pre_clip)
    #         leader_pre_pose = create_clip_goal(goal_frame, follower_pre_clip)
    #         # get_cable_grasping_pose(goal_frame, follower_pre_pose, leader_pre_pose)
    #     task = task_planner.create_task(goal_frame, leader_pre_pose, follower_pre_pose)
        
    #     # try:
    #     #     if task.plan():
    #     #         rospy.logwarn("Executing solution trajectory")
    #     #         task.publish(task.solutions[0])
    #     #         execute_result = task.execute(task.solutions[0])
    #     #         task_planner.clip_togo.remove(clip_id)
    #     #         task_planner.clip_achieved.append(clip_id)
    #     #         if execute_result.val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
    #     #             rospy.logerr("Task execution failed and returned: %s", execute_result.val)
    #     # except Exception as e:
    #     #     print("planning failed with exception\n", e)
    #     #     continue
        
    #     if task.plan():
    #         rospy.logwarn("Executing solution trajectory")
    #         # task.publish(task.solutions[0])
    #         execute_result = task.execute(task.solutions[0])
        
    #     previouse_goal_frame = goal_frame
    
    
    rospy.spin()
    
    