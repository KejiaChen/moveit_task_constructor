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
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MTCPlanGoal


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

    

class TaskPlanner():
    def __init__(self, clip_information, clip_path) -> None:
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("mtc_task_planner", anonymous=True)
        
        # clear previous planning
        rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
        clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        clear_octomap()
        
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
    
    def shape_control(self):
        while len(self.clip_togo) != 0:
            goal_id = self.clip_togo[0]
            self.publish_goal_marker(self.clip_info[goal_id]["W"]["holding_l"], color=[0.0, 0.0, 1.0], id=1)
            self.publish_clip_goal(goal_id)
            
            rospy.loginfo("wait for MTC backend")
            success = rospy.wait_for_message('mtc_back2front_chatter', std_msgs.msg.Bool)
            
            if success.data:
                rospy.loginfo("achieved goal: {}".format(goal_id))
                self.clip_togo = self.clip_togo[1:]
                self.clip_achieved.append(goal_id)
        
        
    def publish_clip_goal(self, goal_id):
        planning_goal = moveit_msgs.msg.MTCPlanGoal()
        leader_goal = planning_goal.robot1_goal_pose
        follower_goal = planning_goal.robot2_goal_pose
        
        leader_goal.header.frame_id = "base"
        leader_goal.pose.orientation.w = 1 # orientation is up-right by default, will be changed later in MTC backend
        leader_goal.pose.position.x = self.clip_info[goal_id]['W']['holding_l'][0]
        leader_goal.pose.position.y = self.clip_info[goal_id]['W']['holding_l'][1]
        leader_goal.pose.position.z = self.clip_info[goal_id]['W']['holding_l'][2]
        
        follower_goal.header.frame_id = "base"
        follower_goal.pose.orientation.w = 1 # orientation is up-right by default, will be changed later in MTC backend
        follower_goal.pose.position.x = self.clip_info[goal_id]['W']['holding_r'][0]
        follower_goal.pose.position.y = self.clip_info[goal_id]['W']['holding_r'][1]
        follower_goal.pose.position.z = self.clip_info[goal_id]['W']['holding_r'][2]
        
        rospy.loginfo("next clip goal: {}".format(goal_id))
        
        rospy.loginfo("leader goal position %.2f", leader_goal.pose.position.x)
        
        rospy.sleep(2) # Attention: this is very necessary for the subscriber to get the message
        self.publisher.publish(planning_goal)
        
        # i = 0
        # while i < 5:
        #     leader_goal.pose.position.x = self.clip_info[goal_id]['W']['holding_l'][0] + i
        #     self.publisher.publish(planning_goal)
        #     rospy.sleep(0.01)
        #     i = i+1
    
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
        

        

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Arguments for fixture scene loading')
    parser.add_argument('-c','--clip_file', default='/home/tp2/Documents/mios-wiring/clip_info20240130_transform.pickle')
    # parser.add_argument('--board_base_size', default='store_true', help='Skip unsetting previous environment variables to extend context')
    # parser.add_argument('--board_size', action='store_true', help='Only consider this prefix path and ignore other prefix path in the environment')
    args = parser.parse_args()
    
    robot1_current_joint = [-1.02680829,  0.44764564,  0.18002864, -2.30666668, -0.24687706,  2.77595507, 1.41033604]
    robot2_current_joint = [-0.03919738,  0.16151273, -0.21564032, -2.65617219,  0.05404735,  2.8080896, 1.55379734]
    
    task_planner = TaskPlanner(clip_information=args.clip_file, clip_path=[7, 6])
    task_planner.shape_control()
    
    