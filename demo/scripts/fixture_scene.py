#!/usr/bin/env python

# Python 2/3 compatibility import
from __future__ import print_function

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
import geometry_msgs.msg
import time
import sys
import copy
import numpy as np
import pickle
import argparse
from scipy.spatial.transform import Rotation as R
from utils.global_variables import *
from utils.mongodb_client import MongoDBClient
from moveit.task_constructor import core, stages
import tf
import tf.transformations as tf_trans

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

def quaternion_between_two_vectors(v1, v2):
    # Normalize the input vectors
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    
    # Calculate the cross product (axis of rotation)
    cross_prod = np.cross(v1, v2)
    
    # Calculate the dot product (cosine of the angle)
    dot_prod = np.dot(v1, v2)
    
    # Compute the angle between the vectors
    angle = np.arccos(dot_prod)
    
    # If the vectors are identical (angle is 0), return the identity quaternion
    if np.isclose(angle, 0.0):
        return [0.0, 0.0, 0.0, 1.0]
    
    # If the vectors are opposite (angle is 180 degrees), we need a special case
    if np.isclose(angle, np.pi):
        # Find an orthogonal vector to v1
        orthogonal_vector = np.array([1, 0, 0]) if not np.isclose(v1[0], 1.0) else np.array([0, 1, 0])
        rotation_axis = np.cross(v1, orthogonal_vector)
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
        return tf_trans.quaternion_about_axis(np.pi, rotation_axis)
    
    # Normalize the cross product to get the rotation axis
    rotation_axis = cross_prod / np.linalg.norm(cross_prod)
    
    # Create the quaternion from the rotation axis and angle
    return tf_trans.quaternion_about_axis(angle, rotation_axis)

def get_holding_orientation(clip_config, default_orientation):
	# default_orientation = [0.0, 0.9999997, 0.0, 0.0007963]
	aruco_fixture_transform = quaternion_between_two_vectors(np.array([1,0,0]), np.array(clip_config['x_axis']))
	transformed_orientation = tf.transformations.quaternion_multiply(aruco_fixture_transform, copy.deepcopy(default_orientation))
	return transformed_orientation

def rotate_pose_stamped_with_quaternion(pose_stamped, rotation_quaternion):
    # Get current orientation as quaternion from PoseStamped
    current_orientation = pose_stamped.pose.orientation
    current_quaternion = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]

    # Multiply the current orientation by the given rotation quaternion
    new_quaternion = tf.transformations.quaternion_multiply(rotation_quaternion, current_quaternion)

    # Update the pose orientation with the new quaternion
    pose_stamped.pose.orientation = geometry_msgs.msg.Quaternion(*new_quaternion)

    return pose_stamped

def convert_db_to_dict(db_data):
    db_dict = {}
    for data in db_data:
        name = int(data["name"])
        del data["name"]
        del data["_id"]
        db_dict[name] = data
    return db_dict


class FixtureScene(object):
    def __init__(self, clip_info_type, clip_info_file, clip_info_num, attachment_info):
        self.scene = PlanningSceneInterface()
        
        # marker publisher
        self.marker_array_pub = rospy.Publisher('interactive_marker_array', MarkerArray, queue_size=1)
        self.marker_pub = rospy.Publisher('interactive_robot_markers', Marker, queue_size=1)
        
        # clear the scene
        self.scene.remove_world_object()
        rospy.loginfo("remove all obejcts from the scene")

        self.robot = RobotCommander()
        
        if clip_info_type == "pickle":
            with open(clip_info_file, 'rb') as clip_file:
                self.clip_infos = pickle.load(clip_file)
        elif clip_info_type == "mongodb":
            self.db_client = MongoDBClient()
            self.clip_data = self.db_client.read("mios_db", "cable_routing_"+str(clip_info_num)+"_clips", {})
            self.clip_config = self.db_client.read("mios_db", "cable_routing_"+str(clip_info_num)+"_configs", {})
            self.clip_infos = convert_db_to_dict(self.clip_data)
            self.clip_setting = convert_db_to_dict(self.clip_config)
        else:
            rospy.logerr("clip info type is not supported")
            
        self.clips = {}
        self.board_height = 0
        
        self.attachment_info = attachment_info
        
    def publish_goal_marker(self, goal_pose_stamped, id=0, color=[1.0, 0.0, 0,0], timeout=2):
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
        # marker = Marker()
        # marker.header.frame_id = "base"  # Set the planning frame
        # marker.header.stamp = rospy.Time.now()
        # marker.ns = "goal_marker"
        # marker.id = id
        # # marker.type = Marker.SPHERE
        # marker.type = Marker.ARROW
        # marker.action = Marker.ADD
        # marker.scale.x = 0.01  # Set the scale according to your preference
        # marker.scale.y = 0.01
        # marker.scale.z = 0.05
        # marker.color.r = color[0]  # Set color (red in this case)
        # marker.color.g = color[1]
        # marker.color.b = color[2]
        # marker.color.a = 0.5  # Alpha (transparency)
        
        # marker.pose.orientation.w = 1.0
        # marker.pose.position.x = goal_position_array[0]
        # marker.pose.position.y = goal_position_array[1]
        # marker.pose.position.z = goal_position_array[2]
        
        # marker.lifetime = rospy.Duration()
        
        marker = self.create_arrow_marker(id, goal_pose_stamped, color)

        # Publish the marker
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            self.marker_pub.publish(marker)
            rospy.rostime.wallsleep(1.0)
            seconds = rospy.get_time()
    
    def create_arrow_marker(self, marker_id, goal_pose_stamped, color=[1.0, 0.0, 0,0]):
        marker = Marker()
        marker.header.frame_id = goal_pose_stamped.header.frame_id # Set the planning frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_marker"
        marker.id = marker_id

        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Set the scale according to your preference
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = color[0]  # Set color (red in this case)
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.5  # Alpha (transparency)
        
        marker.pose = goal_pose_stamped.pose
        
        marker.lifetime = rospy.Duration()
        return marker
    
    def append_frame_markers(self, pose_stamped, timeout=2):
        # Create a MarkerArray to hold all the markers
        marker_array_msg = MarkerArray()

        # Define color for axes
        colors = {
            'x': [1,0,0],  # Red for x-axis
            'y': [0,1,0],  # Green for y-axis
            'z': [0,0,1]   # Blue for z-axis
        }

        # Create markers for x, y, z axes
        # X Axis
        x_marker = self.create_arrow_marker(0, pose_stamped.pose.position, colors['x'])
        marker_array_msg.markers.append(x_marker)
        
        # Y Axis
        y_marker =  self.create_arrow_marker(0, pose_stamped.pose.position, colors['y'])
        marker_array_msg.markers.append(y_marker)
        
        # Z Axis
        z_marker =  self.create_arrow_marker(0, pose_stamped.pose.position, colors['z'])
        marker_array_msg.markers.append(z_marker)

        # Publish the marker array
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            self.marker_array_pub.publish(marker_array_msg)
            rospy.rostime.wallsleep(1.0)
            seconds = rospy.get_time()
    
    def get_all_objects(self):
        # object_ids = self.scene.get_objects().keys()
        object_ids = self.scene.get_known_object_names()

        # Print out the list of object IDs
        rospy.loginfo("List of known object IDs:")
        for object_id in object_ids:
            rospy.loginfo("- %s", object_id)
        
    def wait_for_state_update(
        self, box_name, box_is_known=False, box_is_attached=False, timeout=4
    ):
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def add_qbboard_to_scene(self, expand=True, edge_pos=[0.09, 0, 1], name="qb_board", board_size=BOARD_SIZE):
        box_pose = geometry_msgs.msg.PoseStamped()
        self.board_height = self.board_height + board_size[2]
        if expand:
            board_size = board_size + np.array(EXPAN)
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = edge_pos[0] + board_size[0]/2
        box_pose.pose.position.y = edge_pos[1]
        box_pose.pose.position.z = edge_pos[2] + board_size[2]/2
        self.scene.add_box(name, box_pose, size=board_size)
        result = self.wait_for_state_update(name, box_is_known=True, timeout=8)
        rospy.loginfo("add board %s to the scene: %s", name, result)
        return result
    
    def add_profile_to_scene(self, pose=[0, 0.8, 0], idx=1):
        profile_size = (0.04, 0.04, 2)
        profile_pose = geometry_msgs.msg.PoseStamped()
        profile_pose.header.frame_id = "world"
        profile_pose.pose.orientation.w = 1.0
        profile_pose.pose.position.x = pose[0] + profile_size[0]/2
        profile_pose.pose.position.y = np.sign(pose[1])*(abs(pose[1]) + profile_size[0]/2)
        profile_pose.pose.position.z = pose[2] + profile_size[2]/2
        self.scene.add_box("profile"+str(idx), profile_pose, size=profile_size)
        result = self.wait_for_state_update("profile"+str(idx), box_is_known=True, timeout=8)
        rospy.loginfo("add profile %s to the scene: %s", str(idx), result)
        return result
    
    def add_camera_profile_to_scene(self, pose=[0, 0, 2]):
        profile_size = (0.04, 1.6, 0.04)
        profile_pose = geometry_msgs.msg.PoseStamped()
        profile_pose.header.frame_id = "world"
        profile_pose.pose.orientation.w = 1.0
        profile_pose.pose.position.x = pose[0] + profile_size[0]/2
        profile_pose.pose.position.y = np.sign(pose[1])*(abs(pose[1]) + profile_size[0]/2)
        profile_pose.pose.position.z = pose[2] + profile_size[2]/2
        self.scene.add_box("camera_profile", profile_pose, size=profile_size)
        result = self.wait_for_state_update("profile", box_is_known=True, timeout=8)
        rospy.loginfo("add profile %s to the scene: %s", "camera", result)
        return result
    
    def add_camera_collision_to_scene(self):
        camera_size = (0.02, 0.06, 0.08)
        profile_size = (0.04, 1.6, 0.04)
        camera_pose = geometry_msgs.msg.PoseStamped()
        camera_pose.header.frame_id = "camera_profile"
        camera_pose.pose.orientation.w = 1.0
        camera_pose.pose.position.x = 0
        camera_pose.pose.position.y = 0
        camera_pose.pose.position.z = -camera_size[2]/2-profile_size[2]/2
        self.scene.add_box("camera", camera_pose, size=camera_size)
        result = self.wait_for_state_update("camera", box_is_known=True, timeout=8)
        rospy.loginfo("add camera %s to the scene: %s", "camera", result)
        return result
        
    def add_fixtures_to_scene_backup(self, clip_information, clip_size=X_S_CLIP_SIZE):
        with open(clip_information, 'rb') as clip_file:
            clip_info = pickle.load(clip_file)
            # clip = clip_info[10]
            # clip_position = clip["aruco_W"]
            # clip_size = (0.05, 0.03, 0.04)
            # clip_center_position = np.zeros(3)
            # clip_center_position[0] = clip_position[0] - 0.015 + clip_size[0]/2 # 0.15: dit from aruco center to fixture edge
            # clip_center_position[1] = clip_position[1]
            # clip_center_position[2] = clip_position[2] + clip_size[2]/2
            # result = self.add_one_fixture("clip"+str(10), clip_center_position, clip_size)
            # rospy.loginfo("add clip %s to the scene at %s: %s", str(10), str(clip_center_position), result)
            
            # TODO@Kejia: clip type should also be included in clip.pickle
            for key, clip in clip_info.items():
                clip_position = clip["center_W"]
                current_clip_size = copy.copy(clip_size)
                current_clip_size[2] = current_clip_size[2] + clip["height"]
                # clip_size = (0.05, 0.03, 0.04)
                clip_center_position = np.zeros(3)
                clip_center_position[0] = clip_position[0] - 0.02 + current_clip_size[0]/2 # 0.15: distance from fixture center to fixture edge
                clip_center_position[1] = clip_position[1]
                # put on the board
                # clip_center_position[2] = clip_position[2] + clip_size[2]/2
                clip_center_position[2] = 1 + self.board_height + current_clip_size[2]/2
                
                clip_rot_vec_Kp = R.from_rotvec(clip["rot_vec"])
                clip_rot_euler_Kp = list(clip_rot_vec_Kp.as_euler('zxy', degrees=True))
                print("kp angles", clip_rot_euler_Kp)
                clip_rot_euler_W = copy.deepcopy(clip_rot_euler_Kp)
                clip_rot_euler_W[1] = 0
                clip_rot_euler_W[2] = 0
                clip_orientation = R.from_euler('zyx', clip_rot_euler_W, degrees=True)
                print("quat", list(clip_orientation.as_quat()))
                self.clips["clip"+str(key)] = {"center": clip_center_position, "size": current_clip_size, "orientation": list(clip_orientation.as_quat())}
                result = self.add_one_fixture("clip"+str(key), clip_center_position, current_clip_size, clip_orientation, expand=False)
                rospy.loginfo("add clip %s to the scene at %s: %s", str(key), str(clip_center_position), result)
    
    def add_fixtures_to_scene(self, clip_size=X_S_CLIP_SIZE):
        # TODO@Kejia: clip type should also be included in clip.pickle
        for clip_number, clip_info in self.clip_infos.items():
            clip_position = clip_info["W"]["center"]
            # current_clip_size = copy.copy(clip_size)
            # current_clip_size[2] = current_clip_size[2] + clip_info["W"]["height"]
            # # clip_size = (0.05, 0.03, 0.04)
            clip_config = self.clip_setting[clip_number]
            config_clip_size = clip_config["fixture_size"]
            current_clip_size = np.array(clip_config['x_axis'])*config_clip_size[0] + np.array(clip_config['y_axis'])*config_clip_size[1] + np.array(clip_config['z_axis'])*config_clip_size[2]
            clip_center_position = np.zeros(3)
            clip_center_position[0] = clip_position[0] - 0.02 + current_clip_size[0]/2 # 0.15: distance from fixture center to fixture edge
            clip_center_position[1] = clip_position[1]
            # put on the board
            # clip_center_position[2] = clip_position[2] + clip_size[2]/2
            clip_center_position[2] = 1 + self.board_height + current_clip_size[2]/2
                
            clip_rot_vec_Kp = R.from_rotvec(clip_info["Kp"]["rot_vec"])
            clip_rot_euler_Kp = list(clip_rot_vec_Kp.as_euler('zxy', degrees=True))
            print("kp angles", clip_rot_euler_Kp)
            clip_rot_euler_W = copy.deepcopy(clip_rot_euler_Kp) # only apply rotation around world z axis
            clip_rot_euler_W[1] = 0
            clip_rot_euler_W[2] = 0
            clip_orientation = R.from_euler('zyx', clip_rot_euler_W, degrees=True)
            clip_orientation = get_holding_orientation(clip_config,list(clip_orientation.as_quat()))
            # print("quat", list(clip_orientation.as_quat()))
            print("quat", clip_orientation)
            self.clips["clip"+str(clip_number)] = {"center": clip_center_position, "size": current_clip_size, "orientation": list(clip_orientation)}
            result = self.add_one_fixture("clip"+str(clip_number), clip_center_position, current_clip_size, clip_orientation, expand=False)
            # self.add_fixture_axes("clip"+str(clip_number))
            rospy.loginfo("add clip %s to the scene at %s: %s", str(clip_number), str(clip_center_position), result)            
            
            # add attachment
            # result = self.add_one_attachment("clip"+str(clip_number), current_clip_size)
            # rospy.loginfo("add attachment %s to the scene at %s: %s", str(clip_number), str(clip_center_position), result)            
        
    def add_one_fixture(self, clip_name, clip_pos, clip_size, clip_orientation, expand=True):
        box_pose = geometry_msgs.msg.PoseStamped()
        
        # quaternion = list(clip_orientation.as_quat())
        quaternion = clip_orientation
        
        if expand:
            clip_size = clip_size + EXPAN
        box_pose.header.frame_id = "world"
        # box_pose.pose.orientation.w = 1.0
        box_pose.header.stamp = rospy.Time.now()
        box_pose.pose.orientation.x = quaternion[0]
        box_pose.pose.orientation.y = quaternion[1]
        box_pose.pose.orientation.z = quaternion[2]
        box_pose.pose.orientation.w = quaternion[3]
        box_pose.pose.position.x = clip_pos[0]
        box_pose.pose.position.y = clip_pos[1]
        box_pose.pose.position.z = clip_pos[2] 
        self.scene.add_box(clip_name, box_pose, size=clip_size)
        x_pose = copy.deepcopy(box_pose)
        # y_pose = copy.deepcopy(box_pose)  
        y_pose = rotate_pose_stamped_with_quaternion(copy.deepcopy(box_pose), [0, 0, 0.7071068, 0.7071068])
        # z_pose = rotate_pose_stamped_with_quaternion(copy.deepcopy(box_pose), [0, 0, 0.7071068, 0.7071068])
        self.publish_goal_marker(x_pose, id=1, color=[1.0, 0.0, 0.0])
        self.publish_goal_marker(y_pose, id=2, color=[0.0, 1.0, 0.0])
        # self.publish_goal_marker(z_pose, id=3, color=[0.0, 0.0, 1.0])
        return self.wait_for_state_update(clip_name, box_is_known=True, timeout=8)
    
    def add_one_attachment(self, clip_name, clip_size):
        attachment_size = self.attachment_info[clip_name]
        
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = clip_name
        # box_pose.pose.orientation.w = 1.0
        box_pose.pose.orientation.x = 0
        box_pose.pose.orientation.y = 0
        box_pose.pose.orientation.z = 0
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.x = -clip_size[0]/4 
        box_pose.pose.position.y = -clip_size[1]/4
        box_pose.pose.position.z = clip_size[2]/2 + attachment_size[2]/20.7071068, 0, 0, 0.7071068
        return self.wait_for_state_update(clip_name, box_is_known=True, timeout=8)
    
    def remove_all_objects(self):
        for object in self.scene.get_known_object_names():
            self.remove_one_object(object)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Arguments for fixture scene loading')
    parser.add_argument('-c','--clip_file', default='/home/tp2/Documents/mios-wiring/clip_info_20240710_transform.pickle')
    parser.add_argument('-n','--config_num', default=4)
    parser.add_argument('-f','--data_type', default='mongodb')
    # parser.add_argument('--board_base_size', default='store_true', help='Skip unsetting previous environment variables to extend context')
    # parser.add_argument('--board_size', action='store_true', help='Only consider this prefix path and ignore other prefix path in the environment')
    args = parser.parse_args()
    
    attachment = {'clip8':[0.02, 0.02, 0.04], 
                  'clip5':[0.02, 0.02, 0.04], 
                  'clip0':[0.02, 0.02, 0.04]}
    
    rospy.init_node("fixture_scene")
    while (
        not rospy.search_param("robot_description_semantic") and not rospy.is_shutdown()
    ):
        time.sleep(0.5)
    load_scene = FixtureScene(clip_info_type=args.data_type, clip_info_file=args.clip_file, clip_info_num=args.config_num, attachment_info=attachment)
    
    load_scene.add_qbboard_to_scene(expand=False, edge_pos=[0.18, 0, 1], name="qb_board_base", board_size=BOARD_BASE_SIZE)
    load_scene.add_qbboard_to_scene(expand=True, edge_pos=[0.18, 0, 1+BOARD_BASE_SIZE[2]], name="qb_board", board_size=BOARD_SIZE)

    # add profiles
    load_scene.add_profile_to_scene(pose=[0.18, 0.8, 0], idx=1)
    load_scene.add_profile_to_scene(pose=[0.18, -0.8, 0], idx=2)
    load_scene.add_camera_profile_to_scene(pose=[0.18, 0, 1.96])
    load_scene.add_camera_collision_to_scene()
        
    # add fixtures to scene        
    load_scene.add_fixtures_to_scene(clip_size=X_S_CLIP_SIZE)
    
    # load_scene.get_all_objects()