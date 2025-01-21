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

/* Authors: Michael Goerner, Robert Haschke
   Desc:    Connect arbitrary states by motion planning
*/

#include <moveit/task_constructor/stages/connect_master_follower.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>



using namespace trajectory_processing;

namespace moveit {
namespace task_constructor {
namespace stages {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ConnectMF");

ConnectMF::ConnectMF(const std::string& name, const GroupPlannerVector& planners, const moveit::planning_interface::MoveGroupInterfacePtr& move_group_follow) 
    : Connect(name, planners), move_group_follow_(move_group_follow){
	// setTimeout(1.0);
	// setCostTerm(std::make_unique<cost::PathLength>());

	auto& p = properties();
	// p.declare<MergeMode>("merge_mode", WAYPOINTS, "merge mode");
	// p.declare<double>("max_distance", 1e-2, "maximally accepted distance between end and goal sate");
	// p.declare<moveit_msgs::msg::Constraints>("path_constraints", moveit_msgs::msg::Constraints(),
	//                                          "constraints for the first arm to maintain during trajectory");
	// properties().declare<TimeParameterizationPtr>("merge_time_parameterization",
	//                                               std::make_shared<TimeOptimalTrajectoryGeneration>());

    p.declare<std::string>("lead_group", "right_panda_arm", "Group name of the leader.");
    p.declare<std::string>("follow_group", "left_panda_arm", "Group name of the follower.");
    p.declare<GroupStringDict>("eefs", "vector of names of end-effector group");
}

void ConnectMF::compute(const InterfaceState& from, const InterfaceState& to) {
	const auto& props = properties();
	double timeout = this->timeout();
	MergeMode mode = props.get<MergeMode>("merge_mode");
	double max_distance = props.get<double>("max_distance");

    RCLCPP_INFO_STREAM(LOGGER, "Computing dual-arm trajectory");

    planning_scene::PlanningScenePtr intermediate_scene;
    robot_trajectory::RobotTrajectoryPtr leader_trajectory;
    robot_trajectory::RobotTrajectoryPtr follower_trajectory;
    intermediate_scene = from.scene()->diff();
    moveit::core::RobotState intermediate_state = intermediate_scene->getCurrentStateNonConst();

     // Step 1: Compute trajectory for the first arm
    if (!computeFirstArmTrajectory(from, to, leader_trajectory, intermediate_scene)) {
        auto failed_solution = std::make_shared<SubTrajectory>();
        failed_solution->markAsFailure("Leader arm trajectory planning failed.");
        connect(from, to, failed_solution);
        return;
    }

    // Update the leader arm's state in the intermediate scene
    // Update only the leader arm's joints
    const moveit::core::JointModelGroup* leader_jmg = leader_trajectory->getGroup();
    const moveit::core::RobotState& leader_final_state = leader_trajectory->getLastWayPoint();
    std::vector<double> leader_joint_positions;
    leader_final_state.copyJointGroupPositions(leader_jmg, leader_joint_positions);
    intermediate_state.setJointGroupPositions(leader_jmg, leader_joint_positions);
    // Update the rest of the robot state (e.g., follower arm) remains unchanged
    intermediate_state.update();

    intermediate_scene->setCurrentState(intermediate_state);
	intermediate_scene->getCurrentStateNonConst().update();

    // Step 2: Compute trajectory for the second arm based on the first arm's Cartesian trajectory
    // planning_scene::PlanningScenePtr final_scene = intermediate_scene->diff();
    RCLCPP_INFO_STREAM(LOGGER, "Intermediate scene created");
    if (!computeSecondArmTrajectory(leader_trajectory, from, to, follower_trajectory, intermediate_scene)) {
        auto failed_solution = std::make_shared<SubTrajectory>();
        failed_solution->markAsFailure("Follower arm trajectory planning failed.");
        connect(from, to, failed_solution);
        return;
    }

    // Combine trajectories into a valid dual-arm solution
    robot_trajectory::RobotTrajectoryPtr dual_arm_trajectory =
        std::make_shared<robot_trajectory::RobotTrajectory>(intermediate_scene->getRobotModel(), merged_jmg_.get());
    dual_arm_trajectory->append(*leader_trajectory, 0.0);
    dual_arm_trajectory->append(*follower_trajectory, 0.0);
    auto solution = std::make_shared<SubTrajectory>(dual_arm_trajectory, 0.0, "connect_master_follower");

	// SolutionBasePtr solution;
	// if (success && mode != SEQUENTIAL)  // try to merge
	// 	solution = merge(sub_trajectories, intermediate_scenes, from.scene()->getCurrentState());
	// if (!solution)  // success == false or merging failed: store sequentially
	// 	solution = makeSequential(sub_trajectories, intermediate_scenes, from, to);
	// if (!success)  // error during sequential planning
	// 	solution->markAsFailure(comment);
    
	connect(from, to, solution);
}

bool ConnectMF::computeSecondArmTrajectory(const robot_trajectory::RobotTrajectoryPtr& leader_trajectory,
                                                const InterfaceState& from, const InterfaceState& to,
                                                robot_trajectory::RobotTrajectoryPtr& follower_trajectory,
                                                planning_scene::PlanningScenePtr& intermediate_scene) {

  const auto& props = properties();
  const moveit::core::RobotState& start_state = from.scene()->getCurrentState();
  const moveit::core::RobotState& final_goal_state = to.scene()->getCurrentState();
  planning_scene::PlanningScenePtr final_scene = intermediate_scene->diff();

  RCLCPP_INFO_STREAM(LOGGER, "Computing trajectory for the second arm");
  // Define the transform from the "leader_ee_link" to the actual end-effector frame
  Eigen::Isometry3d lead_grasp_frame_transform = Eigen::Isometry3d::Identity();
  lead_grasp_frame_transform.translation().z() = 0.1034;  // Offset along the Z-axis
  
  // Extract the Cartesian trajectory of the first arm
  std::vector<geometry_msgs::msg::Pose> leader_cartesian_trajectory;
  for (size_t i = 0; i < leader_trajectory->getWayPointCount(); ++i) {
    const auto& point = leader_trajectory->getWayPoint(i);
    // Get the pose of the "leader_ee_link" in the world frame
    // const GroupStringDict& eefs = props.get<GroupStringDict>("eefs");
    // const std::string lead_group_name = props.get<std::string>("lead_group");
    // const std::string leader_ee_link = eefs.at(lead_group_name);
    const std::string leader_ee_link = "right_panda_hand";
    if (!final_goal_state.knowsFrameTransform(leader_ee_link)) {
        RCLCPP_ERROR(LOGGER, "Link '%s' not found in model '%s'", leader_ee_link.c_str(), final_goal_state.getRobotModel()->getName().c_str());
        return false;
    }
    Eigen::Isometry3d ee_link_pose = point.getGlobalLinkTransform(leader_ee_link);
    // Apply the transform to get the pose of the actual end-effector
    Eigen::Isometry3d end_effector_pose = ee_link_pose * lead_grasp_frame_transform;

    // Convert to geometry_msgs::Pose
    geometry_msgs::msg::Pose pose;
    tf2::convert(end_effector_pose, pose);
    leader_cartesian_trajectory.push_back(pose);
  }

  if (leader_cartesian_trajectory.empty()) {
    RCLCPP_ERROR(LOGGER, "Leader arm trajectory is empty.");
    return false;
  }

  /* Step 1: Move second arm to the first arm's starting position */
  bool success=false;
  robot_trajectory::RobotTrajectoryPtr to_start_trajectory;
  // Plan joint trajectory for the leader arm
  for (const auto& pair : planner_) {
    if (pair.first == props.get<std::string>("follow_group")) {
      planning_scene::PlanningSceneConstPtr start = intermediate_scene;
      const moveit::core::JointModelGroup* jmg = final_goal_state.getJointModelGroup(pair.first);
      planning_scene::PlanningScenePtr end = start->diff();
      moveit::core::RobotState& goal_state = end->getCurrentStateNonConst();

      // Set the goal pose for the second arm's end effector
      geometry_msgs::msg::Pose leader_start_pose = leader_cartesian_trajectory.front();
      Eigen::Quaterniond follower_start_orientation(start_state.getGlobalLinkTransform("left_panda_hand").rotation());
      tf2::convert(follower_start_orientation, leader_start_pose.orientation);

      Eigen::Isometry3d follower_start_transform;
      tf2::fromMsg(leader_start_pose, follower_start_transform);
      goal_state.setFromIK(jmg, follower_start_transform, "left_panda_hand");

      // Validate goal state
      RCLCPP_INFO_STREAM(LOGGER, "Follower arm goal state position: " << follower_start_transform.translation().transpose()
                                << " orientation: " << Eigen::Quaterniond(follower_start_transform.rotation()).coeffs().transpose());

      // Plan trajectory
      auto result = pair.second->plan(from.scene(), end, jmg, props.get<double>("timeout"), to_start_trajectory);
      success = bool(result);

      if (!success) {
        RCLCPP_ERROR_STREAM(LOGGER, "Follower arm planning to start failed: " << result.message);
        break;
      }
      
      RCLCPP_INFO_STREAM(LOGGER, "Follower arm planning to start result: " << success);
      
      return true;
    }
  }

  /* Step 2: Follow the first arm's trajectory with a 0.01m offset in EE frame*/
  std::vector<geometry_msgs::msg::Pose> follower_cartesian_trajectory;
  for (const auto& pose : leader_cartesian_trajectory) {
    // Adjust the pose to create the offset
    Eigen::Isometry3d original_pose;
    tf2::fromMsg(pose, original_pose);

    // Define the offset in the ee frame
    Eigen::Vector3d offset_ee(-0.1, 0.0, 0.0);  // 0.1m offset along the X-axis of the ee frame
    // Transform the offset to the world frame
    Eigen::Vector3d offset_world = original_pose.rotation() * offset_ee;
    // Apply the transformed offset to the original pose
    Eigen::Isometry3d offset_pose = original_pose;
    offset_pose.translation() += offset_world;

    geometry_msgs::msg::Pose adjusted_pose;
    tf2::convert(offset_pose, adjusted_pose);
    follower_cartesian_trajectory.push_back(adjusted_pose);
  }

  moveit_msgs::msg::RobotTrajectory follow_trajectory;
  double fraction_follow = move_group_follow_->computeCartesianPath(
      follower_cartesian_trajectory, 0.01, 0.0, follow_trajectory, true);

  if (fraction_follow < 1.0) {
    RCLCPP_WARN(LOGGER, "Follower arm failed to follow the first arm's trajectory. Fraction: %f", fraction_follow);
    return false;
  }

  // Combine the trajectories of two steps
  follower_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(
      final_scene->getRobotModel(), final_scene->getRobotModel()->getJointModelGroup(props.get<std::string>("follow_group")));
  moveit_msgs::msg::RobotTrajectory to_start_trajectory_msg;
  to_start_trajectory->getRobotTrajectoryMsg(to_start_trajectory_msg);
  follower_trajectory->setRobotTrajectoryMsg(final_scene->getCurrentState(), to_start_trajectory_msg);

  robot_trajectory::RobotTrajectory follow_robot_trajectory(final_scene->getRobotModel(), final_scene->getRobotModel()->getJointModelGroup("follower"));
  follow_robot_trajectory.setRobotTrajectoryMsg(final_scene->getCurrentState(), follow_trajectory);
  follower_trajectory->append(follow_robot_trajectory, 0.0);

  return true;
}

bool ConnectMF::computeFirstArmTrajectory(const InterfaceState& from, const InterfaceState& to,
                                               robot_trajectory::RobotTrajectoryPtr& leader_trajectory,
                                               planning_scene::PlanningScenePtr& intermediate_scene) {
  const auto& props = properties();
  const moveit::core::RobotState& final_goal_state = to.scene()->getCurrentState();
  const auto& path_constraints = props.get<moveit_msgs::msg::Constraints>("path_constraints");

  RCLCPP_INFO(LOGGER, "Computing trajectory for the leader arm.");

  bool success=false;

  // Plan joint trajectory for the leader arm
  for (const auto& pair : planner_) {
    if (pair.first == props.get<std::string>("lead_group")) {
      planning_scene::PlanningSceneConstPtr start = from.scene();
      const moveit::core::JointModelGroup* jmg = final_goal_state.getJointModelGroup(pair.first);
      intermediate_scene = start->diff();
      moveit::core::RobotState& goal_state = intermediate_scene->getCurrentStateNonConst();

      // Set the joint group goal
      std::vector<double> positions;
      final_goal_state.copyJointGroupPositions(jmg, positions);
      goal_state.setJointGroupPositions(jmg, positions);
      goal_state.update();

      // Plan trajectory
      auto result = pair.second->plan(start, intermediate_scene, jmg, props.get<double>("timeout"),
                                      leader_trajectory, path_constraints);
      success = bool(result);

      if (!success) {
        RCLCPP_ERROR_STREAM(LOGGER, "Leader arm trajectory planning failed: " << result.message);
        break;
      }
      
      RCLCPP_INFO_STREAM(LOGGER, "Leader arm trajectory planning result: " << success);
      
      return true;
    }
  }
  return false;
}

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
