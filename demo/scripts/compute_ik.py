#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from std_msgs.msg import Header
import time
import moveit_commander
from visualization_msgs.msg import Marker
import rospy

from py_binding_tools import roscpp_init

def publish_marker(pose, frame_id, marker_publisher, timeout=2):
    """
    Publish a marker to represent the pose in RViz.
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.05  # Length of the arrow
    marker.scale.y = 0.05  # Arrow width
    marker.scale.z = 0.05  # Arrow height
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.pose = pose
    
    marker.lifetime = rospy.Duration()

     # Publish the marker
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        marker_publisher.publish(marker)
        rospy.rostime.wallsleep(1.0)
        seconds = rospy.get_time()
        
# CAUTION: only "mtc_tutorial" is allowed as node name
roscpp_init("mtc_tutorial")
rospy.init_node("mtc_task_planner", anonymous=True)


# Specify the planning group
group = "panda_arm"

# Create a task
task = core.Task()

marker_pub = rospy.Publisher('interactive_robot_markers', Marker, queue_size=1)

# Add a stage to retrieve the current state
task.add(stages.CurrentState("current state"))

# Add a planning stage connecting the generator stages
planner = core.PipelinePlanner()  # create default planning pipeline
connect = stages.Connect("connect", [(group, planner)])
connect.max_distance = 1e-3
task.add(connect)  # operate on group
del planner  # Delete PipelinePlanner when not explicitly needed anymore

# [propertyTut12]
# Add a Cartesian pose generator
generator = stages.GeneratePose("cartesian pose")
# [propertyTut12]
# Inherit PlanningScene state from "current state" stage
generator.setMonitoredStage(task["current state"])
# Configure target pose
# [propertyTut13]
pose = Pose(position=Vector3(z=0.2))
publish_marker(pose, "panda_hand", marker_pub)
generator.pose = PoseStamped(header=Header(frame_id="panda_hand"), pose=pose)
# [propertyTut13]

# [initAndConfigComputeIk]
# Wrap Cartesian generator into a ComputeIK stage to yield a joint pose
computeIK = stages.ComputeIK("compute IK", generator)
computeIK.group = group  # Use the group-specific IK solver
# Which end-effector frame should reach the target?
ik_frame = Pose(position=Vector3())
computeIK.ik_frame = PoseStamped(header=Header(frame_id="panda_hand"), pose=ik_frame)
computeIK.max_ik_solutions = 10  # Limit the number of IK solutions
# [propertyTut14]
props = computeIK.properties
# derive target_pose from child's solution
props.configureInitFrom(core.Stage.PropertyInitializerSource.INTERFACE, ["target_pose"])
# [propertyTut14]

# Add the stage to the task hierarchy
task.add(computeIK)
# [initAndConfigComputeIk]

if task.plan():
    task.publish(task.solutions[0])

# time.sleep(100)  # sleep some time to allow C++ threads to publish their messages

while not rospy.is_shutdown():
    rospy.sleep(1)
    
rospy.spin()