/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Robert Haschke, Michael Goerner
   Desc:    Move to joint-state or Cartesian goal pose
*/

#include <moveit/task_constructor/stages/cartesian_waypoints.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/utils.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace moveit {
namespace task_constructor {
namespace stages {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("CartesianWaypointStage");

CartesianWaypointStage::CartesianWaypointStage(const std::string& name, const solvers::PlannerInterfacePtr& planner, const moveit::planning_interface::MoveGroupInterfacePtr& move_group)
  :PropagatingEitherWay(name), planner_(planner), move_group_(move_group) {
  setCostTerm(std::make_unique<cost::PathLength>());
}

void CartesianWaypointStage::init(const moveit::core::RobotModelConstPtr& robot_model) {
  PropagatingEitherWay::init(robot_model);
  planner_->init(robot_model);
}

void CartesianWaypointStage::setWaypoints(const std::vector<geometry_msgs::msg::Pose>& waypoints) {
  waypoints_ = waypoints;
}

void CartesianWaypointStage::setGroup(const std::string& group) {
  group_name_ = group;
}

void CartesianWaypointStage::setIKFrame(const Eigen::Isometry3d& pose, const std::string& link) {
	geometry_msgs::msg::PoseStamped pose_msg;
	pose_msg.header.frame_id = link;
	pose_msg.pose = tf2::toMsg(pose);
	setIKFrame(pose_msg);
}

void CartesianWaypointStage::setIKFrame(const geometry_msgs::msg::PoseStamped& ik_frame) {
  ik_frame_ = ik_frame;
}

void CartesianWaypointStage::setPathConstraints(const moveit_msgs::msg::Constraints& constraints) {
  path_constraints_ = constraints;
}

bool CartesianWaypointStage::compute(const InterfaceState& state, planning_scene::PlanningScenePtr& scene,
                                      SubTrajectory& solution, Interface::Direction dir) {
  scene = state.scene()->diff();
  const moveit::core::RobotModelConstPtr& robot_model = scene->getRobotModel();
  assert(robot_model);

  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name_);
  if (!jmg) {
    solution.markAsFailure("Invalid joint model group: " + group_name_);
    return false;
  }

  // Plan Cartesian path
// Set up MoveGroupInterface for Cartesian Path
  move_group_->setPoseReferenceFrame(scene->getPlanningFrame());
  move_group_->setPathConstraints(path_constraints_);
  move_group_->setStartState(scene->getCurrentState());

  // Plan Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  move_group_->setPoseReferenceFrame("world");
  double fraction = move_group_->computeCartesianPath(
      waypoints_,  // Waypoints
      0.01,        // EEF step (meters)
      0.0,         // Jump threshold
      trajectory,  // Result trajectory
      path_constraints_,
      true
    );

  if (fraction < 1.0) {
    RCLCPP_WARN(LOGGER, "Cartesian path incomplete. Achieved fraction: %f", fraction);
  }

  // Validate trajectory
  if (trajectory.joint_trajectory.points.empty()) {
    solution.markAsFailure("No valid trajectory generated.");
    return false;
  }

  // Store result
  robot_trajectory::RobotTrajectoryPtr robot_trajectory =
      std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, jmg);
  robot_trajectory->setRobotTrajectoryMsg(scene->getCurrentState(), trajectory);
  solution.setTrajectory(robot_trajectory);

  return true;
}

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
