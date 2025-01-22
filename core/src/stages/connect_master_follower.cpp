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

    std::vector<planning_scene::PlanningSceneConstPtr> intermediate_scenes;
    planning_scene::PlanningSceneConstPtr start = from.scene();
    intermediate_scenes.push_back(start);

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
    // Update only the leader arm's joints in the intermediate scene
    const moveit::core::JointModelGroup* leader_jmg = leader_trajectory->getGroup();
    RCLCPP_INFO_STREAM(LOGGER, "leader group name: " << leader_jmg->getName());
    const moveit::core::RobotState& leader_final_state = leader_trajectory->getLastWayPoint();
    std::vector<double> leader_joint_positions;
    leader_final_state.copyJointGroupPositions(leader_jmg, leader_joint_positions);
    intermediate_state.setJointGroupPositions(leader_jmg, leader_joint_positions);
    intermediate_state.update();  // Ensure consistency

    intermediate_scenes.push_back(intermediate_scene->diff());

    // Step 2: Compute trajectory for the second arm based on the first arm's Cartesian trajectory
    // planning_scene::PlanningScenePtr final_scene = intermediate_scene->diff();
    RCLCPP_INFO_STREAM(LOGGER, "Intermediate scene update");
    if (!computeSecondArmTrajectory(leader_trajectory, to, follower_trajectory, intermediate_scene)) {
        auto failed_solution = std::make_shared<SubTrajectory>();
        failed_solution->markAsFailure("Follower arm trajectory planning failed.");
        connect(from, to, failed_solution);
        return;
    }

    intermediate_scenes.push_back(intermediate_scene->diff());

    // Combine trajectories into a valid dual-arm solution
    RCLCPP_INFO_STREAM(LOGGER, "Combining leader and follower arm trajectories");

    std::vector<PlannerIdTrajectoryPair> sub_trajectories;
    sub_trajectories.push_back({ "leader_arm", leader_trajectory });
    sub_trajectories.push_back({ "follower_arm", follower_trajectory });

    // robot_trajectory::RobotTrajectoryPtr dual_arm_trajectory =
    //     std::make_shared<robot_trajectory::RobotTrajectory>(intermediate_scene->getRobotModel(), merged_jmg_.get());
    // dual_arm_trajectory->append(*leader_trajectory, 0.0);
    // dual_arm_trajectory->append(*follower_trajectory, 0.0);
    // auto solution = std::make_shared<SubTrajectory>(dual_arm_trajectory, 0.0, "connect_master_follower");

	SolutionBasePtr solution;
	if (mode != SEQUENTIAL)  // try to merge
		solution = merge(sub_trajectories, intermediate_scenes, from.scene()->getCurrentState());
	if (!solution)  // success == false or merging failed: store sequentially
		solution = makeSequential(sub_trajectories, intermediate_scenes, from, to);
	
    RCLCPP_INFO_STREAM(LOGGER, "ConnectMF solution computed");

	connect(from, to, solution);
}

bool ConnectMF::computeSecondArmTrajectory(const robot_trajectory::RobotTrajectoryPtr& leader_trajectory,
                                                const InterfaceState& to,
                                                robot_trajectory::RobotTrajectoryPtr& follower_trajectory,
                                                planning_scene::PlanningScenePtr& intermediate_scene) {

  const auto& props = properties();
  const moveit::core::RobotState& final_goal_state = to.scene()->getCurrentState();
  const moveit::core::JointModelGroup* follow_jmg;

  double start_offset = 0.15;
  double track_offset = 0.1;

//   // validate the updated state
//   moveit::core::RobotState intermediate_state = final_scene->getCurrentStateNonConst();
//   const std::string leader_ee_link = "right_panda_hand";
//   if (intermediate_state.knowsFrameTransform(leader_ee_link)) {
//       Eigen::Isometry3d ee_transform = intermediate_state.getGlobalLinkTransform(leader_ee_link);
//       RCLCPP_INFO_STREAM(LOGGER, "Leader arm end-effector pose in intermediate_scene: "
//                             << "Position: " << ee_transform.translation().transpose()
//                             << ", Orientation (quaternion): " << Eigen::Quaterniond(ee_transform.rotation()).coeffs().transpose());
//   } else {
//       RCLCPP_ERROR_STREAM(LOGGER, "End-effector link '" << leader_ee_link << "' not found in the updated state.");
//   }

  RCLCPP_INFO_STREAM(LOGGER, "Computing trajectory for the second arm");

  // Define the transform from the "leader_ee_link" to the actual end-effector frame
  Eigen::Isometry3d lead_grasp_frame_transform = Eigen::Isometry3d::Identity();
  lead_grasp_frame_transform.translation().z() = 0.1034;  // Offset along the Z-axis
  
  // Extract the Cartesian trajectory of the first arm
  std::vector<geometry_msgs::msg::Pose> leader_tip_trajectory;
  geometry_msgs::msg::Pose leader_start_hand_pose_msg;
  Eigen::Isometry3d leader_start_tip_pose;
//   geometry_msgs::msg::Pose follower_start_hand_pose_msg;
//   bool follower_start_hand_pose_set = false;

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
    Eigen::Isometry3d ee_link_pose = point.getGlobalLinkTransform(leader_ee_link); // hand pose
    // Apply the transform to get the pose of the actual end-effector
    Eigen::Isometry3d tip_pose = ee_link_pose * lead_grasp_frame_transform;

    // leader start pose
    if (i == 0) {
      tf2::convert(ee_link_pose, leader_start_hand_pose_msg);
      leader_start_tip_pose = tip_pose;
      continue;
    }
    
    if (i>0){
      // check distance to the leader start point to decide the follower start point
      double distance_from_start  = (tip_pose.translation() - leader_start_tip_pose.translation()).norm();
      // Instead of strating from the first pose, start from the closest to the current pose of the follower arm
      if (distance_from_start < start_offset) {
        RCLCPP_WARN(LOGGER, "Distance from the starting point is too narrow: %f", distance_from_start);
        continue;
      }
    }
    
    // Convert to geometry_msgs::Pose
    geometry_msgs::msg::Pose pose;
    tf2::convert(tip_pose, pose);
    leader_tip_trajectory.push_back(pose);

    // if (!follower_start_hand_pose_set){
    //     tf2::convert(ee_link_pose, follower_start_hand_pose_msg);
    //     RCLCPP_INFO_STREAM(LOGGER, "Follower arm start pose: " << follower_start_hand_pose_msg.position.x << ", " << follower_start_hand_pose_msg.position.y << ", " << follower_start_hand_pose_msg.position.z);
    //     follower_start_hand_pose_set = true;
    // }
  }

  if (leader_tip_trajectory.empty()) {
    RCLCPP_ERROR(LOGGER, "Leader arm trajectory is empty.");
    return false;
  }

  // Obtain the Cartesian path for the follower from the leader's Cartesian path
  std::vector<geometry_msgs::msg::Pose> follower_tip_path;
  Eigen::Isometry3d follower_hand_start_pose;

  // Define the transform from the tip frame to "follower_ee_link"
  Eigen::Isometry3d follow_hand_frame_transform = Eigen::Isometry3d::Identity();
  follow_hand_frame_transform.translation().z() = -0.1034;  // Offset along the Z-axis

  for (size_t i = 0; i < leader_tip_trajectory.size(); ++i) {
    const auto& pose_msg = leader_tip_trajectory[i];
    Eigen::Isometry3d original_pose;
    tf2::fromMsg(pose_msg, original_pose);

    // Define the offset in the ee frame
    Eigen::Vector3d offset_ee(-track_offset, 0.0, 0.0);  // offset along the X-axis of the ee frame
    // Transform the offset to the world frame
    Eigen::Vector3d offset_world = original_pose.rotation() * offset_ee;
    Eigen::Isometry3d offset_pose = original_pose;
    offset_pose.translation() += offset_world;

    // TODO: Set the orientation 
    // Eigen::Quaterniond follower_orientation(final_goal_state.getGlobalLinkTransform("left_panda_hand").rotation());
    // offset_pose.linear() = follower_orientation.toRotationMatrix();

    geometry_msgs::msg::Pose adjusted_pose;
    tf2::convert(offset_pose, adjusted_pose);
    follower_tip_path.push_back(adjusted_pose);

    if (i == 0){
        follower_hand_start_pose = offset_pose*follow_hand_frame_transform;
    }
  }

  /********************************************************************/
  /*** Step 1: Move second arm to the first arm's starting position ***/
  /********************************************************************/
  bool success=false;
  robot_trajectory::RobotTrajectoryPtr to_start_trajectory;
  geometry_msgs::msg::Pose follower_start_tip_pose_msg = follower_tip_path[0];


  // Plan joint trajectory for the leader arm
  for (const auto& pair : planner_) {
    if (pair.first == props.get<std::string>("follow_group")) {
      planning_scene::PlanningSceneConstPtr start = intermediate_scene;
      follow_jmg = final_goal_state.getJointModelGroup(pair.first);
      planning_scene::PlanningScenePtr end = start->diff();
      moveit::core::RobotState& goal_state = end->getCurrentStateNonConst();

      // Set the goal pose for the second arm's end effector
    //   Eigen::Quaterniond follower_start_orientation(start_state.getGlobalLinkTransform("left_panda_hand").rotation());
    //   tf2::convert(follower_start_orientation, leader_start_tip_pose.orientation);

    //   Eigen::Isometry3d follower_hand_start_pose;
    //   tf2::fromMsg(follower_start_hand_pose_msg, follower_hand_start_pose);

      goal_state.setFromIK(follow_jmg, follower_hand_start_pose, "left_panda_hand");

      RCLCPP_INFO_STREAM(LOGGER, "Follower arm goal state position: " << follower_hand_start_pose.translation().transpose()
                                << " orientation: " << Eigen::Quaterniond(follower_hand_start_pose.rotation()).coeffs().transpose());

    //   // Validate goal state
    //   std::vector<double> follower_joint_positions;
    //   goal_state.copyJointGroupPositions(jmg, follower_joint_positions);

    //   RCLCPP_INFO_STREAM(LOGGER, "Follower arm goal state joint positions: " << follower_joint_positions[0] << ", " << follower_joint_positions[1] << ", " 
    //                             << follower_joint_positions[2] << ", " << follower_joint_positions[3] << ", " << follower_joint_positions[4] 
    //                             << ", " << follower_joint_positions[5] << ", " << follower_joint_positions[6]);

      // Plan trajectory
      auto result = pair.second->plan(start, end, follow_jmg, props.get<double>("timeout"), to_start_trajectory);
      success = bool(result);

      if (!success) {
        RCLCPP_ERROR_STREAM(LOGGER, "Follower arm planning to start failed: " << result.message);
        break;
      }
      
      RCLCPP_INFO_STREAM(LOGGER, "Follower arm planning to start succeeded with " << to_start_trajectory->getWayPointCount() << " waypoints.");
    //   return true;
    }
  }

  // Validate and update follower arm state in intermediate_scene
  if (to_start_trajectory) {
    // const moveit::core::JointModelGroup* follow_jmg = to_start_trajectory->getGroup();
    const moveit::core::RobotState& follower_final_state = to_start_trajectory->getLastWayPoint();
    std::vector<double> follower_joint_positions;
    follower_final_state.copyJointGroupPositions(follow_jmg, follower_joint_positions);

    moveit::core::RobotState& state = intermediate_scene->getCurrentStateNonConst();
    state.setJointGroupPositions(follow_jmg, follower_joint_positions);
    state.update();  // Ensure consistency

    RCLCPP_INFO_STREAM(LOGGER, "Follower arm state updated in intermediate_scene.");
  }

//   /*********************************************************************************/
//   /*** Step 2: Follow the first arm's trajectory with an offset in EE frame ***/
//   /*********************************************************************************/
//   move_group_follow_->setPoseReferenceFrame("world");
//   move_group_follow_->setStartState(intermediate_scene->getCurrentState());

//   // compute joint trajectory from the cartesian path
//   moveit_msgs::msg::RobotTrajectory follow_trajectory_msg;
//   double fraction_follow = move_group_follow_->computeCartesianPath(
//       follower_tip_path, 0.01, 0.0, follow_trajectory_msg, true);

//   robot_trajectory::RobotTrajectoryPtr follow_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(
//       final_scene->getRobotModel(), final_scene->getRobotModel()->getJointModelGroup(props.get<std::string>("follow_group")));
//   follow_trajectory->setRobotTrajectoryMsg(final_scene->getCurrentState(), follow_trajectory_msg);

//   if (fraction_follow < 1.0) {
//     RCLCPP_WARN(LOGGER, "Follower arm failed to follow the first arm's trajectory. Fraction: %f", fraction_follow);
//     return false;
//   }else{
//     RCLCPP_INFO(LOGGER, "Follower arm successfully followed the first arm's cartesian path.");

//     // Update intermediate scene
//     const moveit::core::JointModelGroup* follow_jmg = follow_trajectory->getGroup();
//     const moveit::core::RobotState& follower_final_state = follow_trajectory->getLastWayPoint();
//     std::vector<double> follower_joint_positions;
//     follower_final_state.copyJointGroupPositions(follow_jmg, follower_joint_positions);

//     moveit::core::RobotState& state = intermediate_scene->getCurrentStateNonConst();
//     state.setJointGroupPositions(follow_jmg, follower_joint_positions);
//     state.update();  // Ensure consistency

//     RCLCPP_INFO_STREAM(LOGGER, "Follower arm state updated in intermediate_scene.");
//   }

 /*****************************************/
 /*** Step 3: Connect to the 'to' scene ***/
 /*****************************************/
 // Plan joint trajectory for the follower arm
 robot_trajectory::RobotTrajectoryPtr to_end_trajectory;

 for (const auto& pair : planner_) {
    if (pair.first == props.get<std::string>("follow_group")) {
    planning_scene::PlanningSceneConstPtr start = intermediate_scene;
    // const moveit::core::JointModelGroup* jmg = final_goal_state.getJointModelGroup(pair.first);
    planning_scene::PlanningScenePtr end = start->diff();
    moveit::core::RobotState& goal_state = end->getCurrentStateNonConst();

    // Set the joint group goal
    std::vector<double> positions;
    final_goal_state.copyJointGroupPositions(follow_jmg, positions);
    goal_state.setJointGroupPositions(follow_jmg, positions);
    goal_state.update();

    // Plan trajectory
    auto result = pair.second->plan(start, end, follow_jmg, props.get<double>("timeout"), to_end_trajectory);
    success = bool(result);

    if (!success) {
        RCLCPP_ERROR_STREAM(LOGGER, "Follower arm trajectory planning to end failed: " << result.message);
        break;
    }
    
    RCLCPP_INFO_STREAM(LOGGER, "Follower arm trajectory planning to end result: " << success);
    
    // return true;
    }
 }

 // Combine the trajectories of two steps
 follower_trajectory = to_start_trajectory;
//  follower_trajectory->append(*follow_trajectory, 0.01);
 follower_trajectory->append(*to_end_trajectory, 0.01);

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
