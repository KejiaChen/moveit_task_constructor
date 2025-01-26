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
#include <moveit/task_constructor/merge.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>



using namespace trajectory_processing;

namespace moveit {
namespace task_constructor {
namespace stages {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ConnectMF");

ConnectMF::ConnectMF(const std::string& name, const GroupPlannerVector& planners, 
                    const moveit::planning_interface::MoveGroupInterfacePtr& move_group_follow,
                    moveit_visual_tools::MoveItVisualTools visual_tools) 
    : Connect(name, planners), move_group_follow_(move_group_follow), visual_tools_(visual_tools){
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

    RCLCPP_INFO_STREAM(LOGGER, "Leader arm trajectory computed with " << leader_trajectory->getWayPointCount() << " waypoints");
    RCLCPP_INFO_STREAM(LOGGER, "Leader arm trajectory duration: " << leader_trajectory->getDuration());

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
		solution = mergeIgnoreCollision(sub_trajectories, intermediate_scenes, from.scene()->getCurrentState());
	if (!solution)  // success == false or merging failed: store sequentially
		solution = makeSequential(sub_trajectories, intermediate_scenes, from, to);
	
    RCLCPP_INFO_STREAM(LOGGER, "ConnectMF solution computed");

	connect(from, to, solution);
}

bool ConnectMF::ExtractFirstArmCartesianTrajectory(const robot_trajectory::RobotTrajectoryPtr& leader_trajectory,
                                                   const moveit::core::RobotState& final_goal_state,
                                                   std::vector<geometry_msgs::msg::Pose>& leader_tip_path,
                                                   std::vector<double>& path_time,
                                                   int& start_index,
                                                   double start_offset) {
  // Define the transform from the "leader_ee_link" to the actual end-effector frame
  Eigen::Isometry3d lead_grasp_frame_transform = Eigen::Isometry3d::Identity();
  lead_grasp_frame_transform.translation().z() = 0.1034;  // Offset along the Z-axis
  
  // std::vector<geometry_msgs::msg::Pose> leader_tip_path;
  geometry_msgs::msg::Pose leader_start_hand_pose_msg;
  Eigen::Isometry3d leader_start_tip_pose;

  bool start = false;
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
        // RCLCPP_WARN(LOGGER, "Distance from the starting point is too narrow: %f", distance_from_start);
        continue;
      }
    }

    if (!start) {
      start_index = i;
      RCLCPP_INFO_STREAM(LOGGER, "Follower starts tracking from waypoint index: " << start_index);
      start = true;
    }
    
    // Convert to geometry_msgs::Pose
    geometry_msgs::msg::Pose pose;
    tf2::convert(tip_pose, pose);
    leader_tip_path.push_back(pose);

    // log the time for each point
    path_time.push_back(leader_trajectory->getWayPointDurationFromStart(i));

  }

  if (leader_tip_path.empty()) {
    RCLCPP_ERROR(LOGGER, "Leader arm trajectory is empty.");
    return false;
  }

  return true;
}

bool ConnectMF::computeSecondArmTrajectory(robot_trajectory::RobotTrajectoryPtr& leader_trajectory,
                                          const InterfaceState& to,
                                          robot_trajectory::RobotTrajectoryPtr& follower_trajectory,
                                          planning_scene::PlanningScenePtr& intermediate_scene,
                                          bool reverse) {

  const auto& props = properties();
  const moveit::core::RobotState& initial_state = intermediate_scene->getCurrentState();
  const moveit::core::RobotState& final_goal_state = to.scene()->getCurrentState();
  // const moveit::core::JointModelGroup* follow_jmg_;

  Eigen::Quaterniond follower_initial_orientation(initial_state.getGlobalLinkTransform("left_panda_hand").rotation());

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

  /* Extract the Cartesian trajectory of the first arm */ 
  std::vector<geometry_msgs::msg::Pose> leader_tip_path;
  std::vector<double> path_time_sequnce;
  int leader_start_index = 0;

  double leader_duration_original = leader_trajectory->getDuration();
  RCLCPP_INFO_STREAM(LOGGER, "Leader arm trajectory duration: " << leader_duration_original);
  
  if (!ExtractFirstArmCartesianTrajectory(leader_trajectory, final_goal_state, leader_tip_path, path_time_sequnce, 
                                          leader_start_index, start_offset)) {
    RCLCPP_INFO_STREAM(LOGGER, "Failed to extract leader arm Cartesian trajectory.");
    return false;
  }

  /* Obtain the Cartesian path for the follower from the leader's Cartesian path */
  std::vector<geometry_msgs::msg::Pose> follower_tip_path;
  std::vector<geometry_msgs::msg::Pose> follower_hand_path;

  Eigen::Isometry3d follower_hand_start_pose;
  Eigen::Isometry3d follower_tip_start_pose;
  Eigen::Quaterniond follower_start_orientation; //TODO@KejiaChen: align y rotation to initial_state, the other to leader
  Eigen::Quaterniond follower_final_orientation(final_goal_state.getGlobalLinkTransform("left_panda_hand").rotation());
  // RCLCPP_INFO_STREAM(LOGGER, "Follower arm final orientation: " << follower_final_orientation.coeffs().transpose());

  // Define the transform from the tip frame to "follower_ee_link"
  Eigen::Isometry3d follow_hand_frame_transform = Eigen::Isometry3d::Identity();
  follow_hand_frame_transform.translation().z() = -0.1034;  // Offset along the Z-axis
  // Eigen::Matrix3d follower_ee_orientation;
  // follower_ee_orientation << 0.7071, -0.7071, 0,
  //                           0.7071, 0.7071, 0,
  //                           0, 0, 1;
  // follow_hand_frame_transform.linear() = follower_ee_orientation;
  

  int length = leader_tip_path.size(); // plan until the last 10 waypoints
  for (size_t i = 0; i < length; ++i) {
    double percentage = (double)i / (double)length;

    const auto& pose_msg = leader_tip_path[i];
    Eigen::Isometry3d original_pose;
    tf2::fromMsg(pose_msg, original_pose);
    
    // Define the offset in the ee frame
    Eigen::Vector3d offset_ee(-track_offset, 0.0, 0.0);  // offset along the X-axis of the ee frame
    // Transform the offset to the world frame
    Eigen::Vector3d offset_in_world = original_pose.rotation() * offset_ee;
    Eigen::Isometry3d follow_tip_pose = original_pose;
    follow_tip_pose.translation() += offset_in_world;

    // Interpolate the orientation
    if (i > 0){
      follower_start_orientation.normalize();
      follower_final_orientation.normalize();

      // if (follower_start_orientation.dot(follower_final_orientation) < 0.0) {
      //     follower_final_orientation.coeffs() = -follower_final_orientation.coeffs();
      // }

      Eigen::Quaterniond interpolated_orientation = follower_start_orientation.slerp(percentage, follower_final_orientation);
      // interpolated_orientation.normalize();
      follow_tip_pose.linear() = interpolated_orientation.toRotationMatrix();

      // RCLCPP_INFO_STREAM(LOGGER, "Follower arm orientation at " << percentage << ": " << Eigen::Quaterniond(follow_tip_pose.rotation()).coeffs().transpose());
    }

    // set start tip orientation
    if (i == 0){
      Eigen::Quaterniond lead_hand_start_orientation = Eigen::Quaterniond(original_pose.rotation());
      
      // Extract the rotation components
      Eigen::Matrix3d initial_rotation_matrix = follower_initial_orientation.toRotationMatrix();
      Eigen::Matrix3d start_rotation_matrix = lead_hand_start_orientation.toRotationMatrix();

      // Extract Euler angles (Roll, Pitch, Yaw) from the matrices
      Eigen::Vector3d initial_euler_angles = initial_rotation_matrix.eulerAngles(0, 1, 2);  // XYZ convention
      Eigen::Vector3d start_euler_angles = start_rotation_matrix.eulerAngles(0, 1, 2);

      // Extract roll (X) and pitch (Y) from the initial orientation
      double initial_roll = initial_euler_angles[0];
      double initial_pitch = initial_euler_angles[1];
      // Extract yaw (Z) from the start orientation
      double start_yaw = start_euler_angles[2];

      // Create a combined rotation with roll and pitch from the initial orientation and yaw from the start orientation
      Eigen::Quaterniond combined_orientation =
          Eigen::AngleAxisd(initial_roll, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(initial_pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(start_yaw, Eigen::Vector3d::UnitZ());

      // Set the follower's start orientation
      follow_tip_pose.linear() = combined_orientation.toRotationMatrix();
    }

    // pose at hand
    Eigen::Isometry3d follow_hand_pose = follow_tip_pose*follow_hand_frame_transform;

    if (i == 0){
      follower_hand_start_pose = follow_hand_pose;
      follower_tip_start_pose = follow_tip_pose;

      follower_start_orientation = Eigen::Quaterniond(follow_hand_pose.rotation());
      
      // RCLCPP_INFO_STREAM(LOGGER, "Follower arm start orientation: " << follower_final_orientation.coeffs().transpose());
    }

    geometry_msgs::msg::Pose adjusted_tip_pose;
    tf2::convert(follow_tip_pose, adjusted_tip_pose);
    follower_tip_path.push_back(adjusted_tip_pose);

    geometry_msgs::msg::Pose adjusted_hand_pose;
    tf2::convert(follow_hand_pose, adjusted_hand_pose);
    follower_hand_path.push_back(adjusted_hand_pose);
  }

  // Test the last orientation of cartesian wapyoints
  // geometry_msgs::msg::Pose last_pose = follower_hand_path.back();
  // Eigen::Isometry3d last_pose_transformed;
  // tf2::fromMsg(last_pose, last_pose_transformed);
  // Eigen::Quaterniond last_orientation(last_pose_transformed.rotation());
  // RCLCPP_INFO_STREAM(LOGGER, "Follower arm last orientation: " << last_orientation.coeffs().transpose());

  // Test if position follower_hand_start_pose == follower_hand_path[0]
  Eigen::Vector3d start_hand_position = follower_hand_start_pose.translation();
  Eigen::Quaterniond start_hand_orientation(follower_hand_start_pose.rotation());
  start_hand_orientation.normalize();
  RCLCPP_INFO_STREAM(LOGGER, "Follower arm first step should end at hand position: " << start_hand_position.transpose());
  RCLCPP_INFO_STREAM(LOGGER, "Follower arm first step should end at hand orientation: " << start_hand_orientation.coeffs().transpose());

  Eigen::Vector3d path_hand_position(follower_hand_path[0].position.x, follower_hand_path[0].position.y, follower_hand_path[0].position.z);
  Eigen::Quaterniond path_hand_orientation(follower_hand_path[0].orientation.w, follower_hand_path[0].orientation.x, 
                                            follower_hand_path[0].orientation.y, follower_hand_path[0].orientation.z);
  path_hand_orientation.normalize();
  RCLCPP_INFO_STREAM(LOGGER, "Follower arm second step shoud start from hand position: " << path_hand_position.transpose());
  RCLCPP_INFO_STREAM(LOGGER, "Follower arm second step shoud start from hand orientation: " << path_hand_orientation.coeffs().transpose());

  // Test if position follower_hand_start_pose == follower_hand_path[0]
  Eigen::Vector3d start_tip_position = follower_tip_start_pose.translation();
  Eigen::Quaterniond start_tip_orientation(follower_tip_start_pose.rotation());
  RCLCPP_INFO_STREAM(LOGGER, "Follower arm first step should end at tcp position: " << start_tip_position.transpose());
  RCLCPP_INFO_STREAM(LOGGER, "Follower arm first step should end at tcp orientation: " << start_tip_orientation.coeffs().transpose());

  Eigen::Vector3d path_tip_position(follower_tip_path[0].position.x, follower_tip_path[0].position.y, follower_tip_path[0].position.z);
  Eigen::Quaterniond path_tip_orientation(follower_tip_path[0].orientation.w, follower_tip_path[0].orientation.x, follower_tip_path[0].orientation.y, 
                                          follower_tip_path[0].orientation.z);
  RCLCPP_INFO_STREAM(LOGGER, "Follower arm second step shoud start from tcp position: " << path_tip_position.transpose());
  RCLCPP_INFO_STREAM(LOGGER, "Follower arm second step shoud start from tcp orientation: " << path_tip_orientation.coeffs().transpose());

  // visual_tools_.publishPath(follower_tip_path, rviz_visual_tools::YELLOW, rviz_visual_tools::MEDIUM);
  // visual_tools_.trigger();

  /********************************************************************/
  /*** Step 1: Move second arm to the first arm's starting position ***/
  /********************************************************************/
  bool success=false;
  robot_trajectory::RobotTrajectoryPtr to_start_trajectory;

  // Plan joint trajectory for the leader arm
  for (const auto& pair : planner_) {
    if (pair.first == props.get<std::string>("follow_group")) {
      planning_scene::PlanningSceneConstPtr start = intermediate_scene;
      follow_jmg_ = final_goal_state.getJointModelGroup(pair.first);
      planning_scene::PlanningScenePtr end = start->diff();
      moveit::core::RobotState& goal_state = end->getCurrentStateNonConst();

    // Set the goal pose for the second arm's end effector
    //   Eigen::Quaterniond follower_start_orientation(initial_state.getGlobalLinkTransform("left_panda_hand").rotation());
    //   tf2::convert(follower_start_orientation, leader_start_tip_pose.orientation);

    //   Eigen::Isometry3d follower_hand_start_pose;
    //   tf2::fromMsg(follower_start_hand_pose_msg, follower_hand_start_pose);

      goal_state.setFromIK(follow_jmg_, follower_hand_start_pose, "left_panda_hand");

      RCLCPP_INFO_STREAM(LOGGER, "Follower arm first step goal hand position: " << follower_hand_start_pose.translation().transpose()
                                << " orientation: " << Eigen::Quaterniond(follower_hand_start_pose.rotation()).coeffs().transpose());

    //   // Validate goal state
    //   std::vector<double> follower_joint_positions;
    //   goal_state.copyJointGroupPositions(jmg, follower_joint_positions);

    //   RCLCPP_INFO_STREAM(LOGGER, "Follower arm goal state joint positions: " << follower_joint_positions[0] << ", " << follower_joint_positions[1] << ", " 
    //                             << follower_joint_positions[2] << ", " << follower_joint_positions[3] << ", " << follower_joint_positions[4] 
    //                             << ", " << follower_joint_positions[5] << ", " << follower_joint_positions[6]);

      // Plan trajectory
      auto result = pair.second->plan(start, end, follow_jmg_, props.get<double>("timeout"), to_start_trajectory);
      success = bool(result);

      if (!success) {
        RCLCPP_ERROR_STREAM(LOGGER, "Follower arm planning to start failed: " << result.message);
        return false;
      }
      
      RCLCPP_INFO_STREAM(LOGGER, "Follower arm planning to start succeeded with " << to_start_trajectory->getWayPointCount() << " waypoints.");
    //   return true;
    }
  }

  // Validate and update follower arm state in intermediate_scene
  if (to_start_trajectory) {
    // const moveit::core::JointModelGroup* follow_jmg_ = to_start_trajectory->getGroup();
    const moveit::core::RobotState& follower_final_state = to_start_trajectory->getLastWayPoint();
    std::vector<double> follower_joint_positions;
    follower_final_state.copyJointGroupPositions(follow_jmg_, follower_joint_positions);

    moveit::core::RobotState& state = intermediate_scene->getCurrentStateNonConst();
    state.setJointGroupPositions(follow_jmg_, follower_joint_positions);
    state.update();  // Ensure consistency

    RCLCPP_INFO_STREAM(LOGGER, "Follower arm state updated in intermediate_scene.");
  }

  // follower_trajectory = to_start_trajectory;

  // Get current position and orientation
  Eigen::Isometry3d left_panda_hand_transform = intermediate_scene->getCurrentState().getGlobalLinkTransform("left_panda_hand");
  // Define the Z-offset in the left_panda_hand frame
  Eigen::Isometry3d tcp_offset = Eigen::Isometry3d::Identity();
  tcp_offset.translation().z() = 0.1034; // Set the Z-offset value
  // Define the transform from the "leader_ee_link" to the actual end-effector frame
  // Eigen::Matrix3d follower_ee_orientation;
  // follower_ee_orientation << 0.7071, 0.7071, 0,
  //                           -0.7071, 0.7071, 0,
  //                           0, 0, 1;
  // tcp_offset.linear() = follower_ee_orientation;
  Eigen::Isometry3d tcp_transform = left_panda_hand_transform * tcp_offset;
  // Get the translation of the TCP
  Eigen::Vector3d start_position_updated = tcp_transform.translation();

  /* Time Adjustment */
  auto delayed_to_start_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(to_start_trajectory->getRobotModel(), to_start_trajectory->getGroup());
  // Get the time when finishing the first step
  double first_step_end_time = to_start_trajectory->getWayPointDurationFromStart(to_start_trajectory->getWayPointCount());
  double leader_first_step_end_time = leader_trajectory->getWayPointDurationFromStart(leader_start_index);

  double leader_second_step_start_time = leader_first_step_end_time;
  double leader_second_step_end_time = leader_duration_original;
  if (to_start_trajectory){
    // Force the leader_trajectory to wait for the follower_trajectory to finish the first step
    // double pause_duration = first_step_end_time - leader_trajectory->getWayPointDurationFromStart(leader_start_index);
    double pause_duration = first_step_end_time;
    RCLCPP_INFO_STREAM(LOGGER, "Pause duration: " << pause_duration);
    auto leader_trajectory_with_pause = std::make_shared<robot_trajectory::RobotTrajectory>(leader_trajectory->getRobotModel(), leader_trajectory->getGroup());
    if (!splitTrajectoryWithPause(leader_trajectory, pause_duration, leader_start_index, leader_trajectory_with_pause)) {
      RCLCPP_ERROR(LOGGER, "Failed to split the leader trajectory.");
      return false;
    }
    leader_trajectory = leader_trajectory_with_pause;

    leader_second_step_start_time = leader_second_step_start_time + pause_duration;
    leader_second_step_end_time = leader_second_step_end_time + pause_duration;

    // Force the follower_trajectory to start after the leader_trajectory finishes the first step
    if (!splitTrajectoryWithPause(to_start_trajectory, leader_first_step_end_time, 0, delayed_to_start_trajectory)) {
      RCLCPP_ERROR(LOGGER, "Failed to delay the follower trajectory.");
      return false;
    }
  }
  double leader_second_duration = leader_second_step_end_time - leader_second_step_start_time;
  RCLCPP_INFO_STREAM(LOGGER, "Leader arm second duration: " << leader_second_duration);

  follower_trajectory = delayed_to_start_trajectory;

  // // Log the hand position
  // RCLCPP_INFO_STREAM(LOGGER, "Follower arm hand position after first step: " << left_panda_hand_transform.translation().transpose());
  // Eigen::Quaterniond first_reached_hand_orientaiton(left_panda_hand_transform.rotation());
  // RCLCPP_INFO_STREAM(LOGGER, "Follower arm hand orientation after first step: " << first_reached_hand_orientaiton.coeffs().transpose());

  // // Log the TCP position
  // RCLCPP_INFO_STREAM(LOGGER, "Follower arm TCP position after first step: " << start_position_updated.transpose());
  // Eigen::Quaterniond first_reached_orientaiton(intermediate_scene->getCurrentState().getGlobalLinkTransform("left_panda_hand").rotation());
  // RCLCPP_INFO_STREAM(LOGGER, "Follower arm reached orientation after first step: " << first_reached_orientaiton.coeffs().transpose());

  /*********************************************************************************/
  /*** Step 2: Follow the first arm's trajectory with an offset in EE frame ***/
  /*********************************************************************************/
  robot_trajectory::RobotTrajectoryPtr follow_trajectory;

  double fraction_follow = SecondArmFollow(intermediate_scene, follower_tip_path, follow_trajectory);

  if (fraction_follow < 1.0) {
    RCLCPP_WARN(LOGGER, "Follower arm failed to follow the first arm's trajectory. Fraction: %f", fraction_follow);
    return false;
  }else{
    RCLCPP_INFO(LOGGER, "Follower arm successfully followed the first arm's cartesian path.");

    // Update intermediate scene
    // const moveit::core::JointModelGroup* follow_jmg_ = follow_trajectory->getGroup();
    const moveit::core::RobotState& follower_final_state = follow_trajectory->getLastWayPoint();
    std::vector<double> follower_joint_positions;
    follower_final_state.copyJointGroupPositions(follow_jmg_, follower_joint_positions);

    moveit::core::RobotState& state = intermediate_scene->getCurrentStateNonConst();
    state.setJointGroupPositions(follow_jmg_, follower_joint_positions);
    state.update();  // Ensure consistency

    RCLCPP_INFO_STREAM(LOGGER, "Follower arm state updated in intermediate_scene.");
  }

  // Perform time parameterization for velocity consistency
  trajectory_processing::IterativeParabolicTimeParameterization time_param;

  robot_trajectory::RobotTrajectory scaled_trajectory(follow_trajectory->getRobotModel(), follow_trajectory->getGroup());
  try {
      scaled_trajectory = reinterpolateTrajectory(follow_trajectory, leader_second_duration, 0.1);
      
      // The new trajectory is now ready for execution or further processing
  } catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "Error during trajectory re-interpolation: %s", e.what());
  }

  follower_trajectory->append(scaled_trajectory, 0.0);

  // follower_trajectory->append(*follow_trajectory, 0.0);

  // Get current orientation
  Eigen::Quaterniond next_reached_orientaiton(intermediate_scene->getCurrentState().getGlobalLinkTransform("left_panda_hand").rotation());
  RCLCPP_INFO_STREAM(LOGGER, "Follower arm reached orientation after second step: " << next_reached_orientaiton.coeffs().transpose());

  // /*****************************************/
  // /*** Step 3: Connect to the 'to' scene ***/
  // /*****************************************/
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
    final_goal_state.copyJointGroupPositions(follow_jmg_, positions);
    goal_state.setJointGroupPositions(follow_jmg_, positions);
    goal_state.update();

    // Plan trajectory
    auto result = pair.second->plan(start, end, follow_jmg_, props.get<double>("timeout"), to_end_trajectory);
    success = bool(result);

    if (!success) {
        RCLCPP_ERROR_STREAM(LOGGER, "Follower arm trajectory planning to end failed: " << result.message);
        break;
    }
    
    RCLCPP_INFO_STREAM(LOGGER, "Follower arm trajectory planning to end result: " << success);
    
    // return true;
    }
  }

  follower_trajectory->append(*to_end_trajectory, 0.0);

  // Update intermediate scene
  if (to_end_trajectory){
    // const moveit::core::JointModelGroup* follow_jmg_ = to_end_trajectory->getGroup();
    const moveit::core::RobotState& follower_final_state = to_end_trajectory->getLastWayPoint();
    std::vector<double> follower_joint_positions;
    follower_final_state.copyJointGroupPositions(follow_jmg_, follower_joint_positions);

    moveit::core::RobotState& state = intermediate_scene->getCurrentStateNonConst();
    state.setJointGroupPositions(follow_jmg_, follower_joint_positions);
    state.update();  // Ensure consistency

    RCLCPP_INFO_STREAM(LOGGER, "Follower arm state updated in intermediate_scene.");
  }

  // Get current orientation
  Eigen::Quaterniond final_reached_orientaiton(intermediate_scene->getCurrentState().getGlobalLinkTransform("left_panda_hand").rotation());
  RCLCPP_INFO_STREAM(LOGGER, "Follower arm reached final orientation: " << final_reached_orientaiton.coeffs().transpose());

  /* Smoothing */

  // Perform time parameterization for velocity consistency
  // trajectory_processing::IterativeParabolicTimeParameterization time_param;

  // if (!time_param.computeTimeStamps(*follower_trajectory)) {
  //     RCLCPP_ERROR(LOGGER, "Time parameterization failed for the follower arm's trajectory.");
  //     return false;
  // }

  trajectory_processing::RuckigSmoothing ruckig_smoother;
    
  // Apply Ruckig smoothing to the trajectory
  if (!ruckig_smoother.applySmoothing(*follower_trajectory)) {
      RCLCPP_ERROR(LOGGER, "Ruckig smoothing failed to smooth trajectory.");
  } else {
      RCLCPP_INFO(LOGGER, "Ruckig smoothing successfully applied.");
  }

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

double ConnectMF::SecondArmFollow(planning_scene::PlanningScenePtr& intermediate_scene,
                                std::vector<geometry_msgs::msg::Pose> follower_tip_path,
                                robot_trajectory::RobotTrajectoryPtr& follow_trajectory) {

  const auto& props = properties();
  move_group_follow_->setPoseReferenceFrame("world");
  move_group_follow_->setStartState(intermediate_scene->getCurrentState());

  auto follow_scene = intermediate_scene->diff();

  // Define the transform from the "leader_ee_link" to the actual end-effector frame
  Eigen::Matrix3d follower_ee_orientation;
  follower_ee_orientation << 0.7071, 0.7071, 0,
                            -0.7071, 0.7071, 0,
                            0, 0, 1;
  Eigen::Isometry3d follow_grasp_frame_transform = Eigen::Isometry3d::Identity();
  follow_grasp_frame_transform.translation().z() = 0.1034;  // Offset along the Z-axis
  follow_grasp_frame_transform.linear() = follower_ee_orientation;


  // compute joint trajectory from the cartesian path
  moveit_msgs::msg::RobotTrajectory follow_trajectory_msg;
  double fraction_follow = move_group_follow_->computeCartesianPath(follower_tip_path, 0.01, 2.0, follow_trajectory_msg, true,
                                                                    nullptr, follow_grasp_frame_transform);
  // follower_cartesian_planner_.plan(follow_scene, follow_jmg_->getLinkModel("left_panda_hand"),
  //                                follow_grasp_frame_transform, 
  // follower_hand_path, 0.01, 0.0, follow_trajectory_msg);

  follow_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(
      follow_scene->getRobotModel(), follow_scene->getRobotModel()->getJointModelGroup(props.get<std::string>("follow_group")));
  follow_trajectory->setRobotTrajectoryMsg(follow_scene->getCurrentState(), follow_trajectory_msg);

  return fraction_follow;
}

SubTrajectoryPtr ConnectMF::mergeIgnoreCollision(const std::vector<PlannerIdTrajectoryPair>& sub_trajectories,
                                  const std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes,
                                  const moveit::core::RobotState& state) {
	// no need to merge if there is only a single sub trajectory
	if (sub_trajectories.size() == 1)
		return std::make_shared<SubTrajectory>(sub_trajectories.at(0).trajectory, 0.0, std::string(""),
		                                       sub_trajectories.at(0).planner_id);

	// split sub_trajectories into trajectories and joined planner_ids
	std::string planner_ids;
	std::vector<robot_trajectory::RobotTrajectoryConstPtr> subs;
	subs.reserve(sub_trajectories.size());
	for (auto it = sub_trajectories.begin(); it != sub_trajectories.end(); ++it) {
		subs.push_back(it->trajectory);
		if (it != sub_trajectories.begin())
			planner_ids += ", ";
		planner_ids += it->planner_id;
	}

	RCLCPP_INFO(LOGGER, "Merge trajectories from planners: %s", planner_ids.c_str());

	auto jmg = merged_jmg_.get();
	assert(jmg);
	auto timing = properties().get<TimeParameterizationPtr>("merge_time_parameterization");
	robot_trajectory::RobotTrajectoryPtr trajectory = task_constructor::merge(subs, state, jmg, *timing);
	if (!trajectory){
		RCLCPP_INFO(LOGGER, "Failed to merge trajectories");
		return SubTrajectoryPtr();
	}

	// // check merged trajectory for collisions
	// if (!intermediate_scenes.front()->isPathValid(*trajectory,
	//                                               properties().get<moveit_msgs::msg::Constraints>("path_constraints"))){
	// 	RCLCPP_INFO(LOGGER, "Collision detected in merged trajectory");
	// 	return SubTrajectoryPtr();
	// }

	return std::make_shared<SubTrajectory>(trajectory, 0.0, std::string(""), planner_ids);
}

bool ConnectMF::splitTrajectoryWithPause(const robot_trajectory::RobotTrajectoryPtr& trajectory,
                                          const double pause_duration,
                                          const int split_index,
                                          robot_trajectory::RobotTrajectoryPtr& split_trajectory){
  auto first_part = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory->getRobotModel(), trajectory->getGroup());
  auto second_part = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory->getRobotModel(), trajectory->getGroup());

  RCLCPP_INFO_STREAM(LOGGER, "Splitting the trajectory at index: " << split_index);

  // Add waypoints to the first part
  for (size_t i = 0; i <= split_index; ++i) {
      first_part->addSuffixWayPoint(trajectory->getWayPoint(i), trajectory->getWayPointDurationFromStart(i));
  }

  // Add waypoints to the second part
  for (size_t i = split_index; i < trajectory->getWayPointCount(); ++i) {
      second_part->addSuffixWayPoint(trajectory->getWayPoint(i), trajectory->getWayPointDurationFromStart(i) - trajectory->getWayPointDurationFromStart(split_index));
  }

 // Ensure zero velocity at the split points
  moveit::core::RobotState& first_part_last_state = *first_part->getLastWayPointPtr();
  first_part_last_state.zeroVelocities();

  moveit::core::RobotState& second_part_first_state = *second_part->getFirstWayPointPtr();
  second_part_first_state.zeroVelocities();
  
  // create a pause part
  auto pause_part = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory->getRobotModel(), trajectory->getGroup());
  moveit::core::RobotState pause_state = first_part->getLastWayPoint();
  // Add duplicate waypoints for the pause duration
  for (double t = 0.1; t <= pause_duration; t += 0.1) {
      pause_part->addSuffixWayPoint(pause_state, t);
  }

  trajectory_processing::IterativeParabolicTimeParameterization time_param;
  // Smooth the first part
  if (!time_param.computeTimeStamps(*first_part)) {
      RCLCPP_ERROR(LOGGER, "Time parameterization failed for the first part.");
  }
  // Smooth the pause part (no motion, so time stamps remain consistent)
  if (!time_param.computeTimeStamps(*pause_part)) {
      RCLCPP_ERROR(LOGGER, "Time parameterization failed for the pause part.");
  }
  // Smooth the second part
  if (!time_param.computeTimeStamps(*second_part)) {
      RCLCPP_ERROR(LOGGER, "Time parameterization failed for the second part.");
  }

  // auto split_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory->getRobotModel(), trajectory->getGroup());

  // Add waypoints from the first part
  for (size_t i = 0; i < first_part->getWayPointCount(); ++i) {
      split_trajectory->addSuffixWayPoint(first_part->getWayPoint(i), first_part->getWayPointDurationFromStart(i));
  }

  // Add waypoints from the pause part
  double pause_time_offset = split_trajectory->getWayPointDurationFromStart(split_trajectory->getWayPointCount() - 1);
  for (size_t i = 0; i < pause_part->getWayPointCount(); ++i) {
      split_trajectory->addSuffixWayPoint(pause_part->getWayPoint(i), pause_time_offset + pause_part->getWayPointDurationFromStart(i));
  }

  // Add waypoints from the second part
  double second_part_time_offset = split_trajectory->getWayPointDurationFromStart(split_trajectory->getWayPointCount() - 1);
  for (size_t i = 0; i < second_part->getWayPointCount(); ++i) {
      split_trajectory->addSuffixWayPoint(second_part->getWayPoint(i), second_part_time_offset + second_part->getWayPointDurationFromStart(i));
  }

  return true;
}

robot_trajectory::RobotTrajectory ConnectMF::reinterpolateTrajectory(const robot_trajectory::RobotTrajectoryPtr& original_trajectory, 
                                                                      double total_time, 
                                                                      double waypoint_interval) {
    if (original_trajectory->empty()) {
        throw std::runtime_error("Input trajectory is empty.");
    }

    // Get the total duration of the original trajectory
    double original_duration = original_trajectory->getWayPointDurationFromStart(original_trajectory->getWayPointCount() - 1);
    double scaling_factor = total_time / original_duration;

    // Create a new trajectory with re-interpolated waypoints
    robot_trajectory::RobotTrajectory reinterpolated_trajectory(original_trajectory->getRobotModel(), original_trajectory->getGroup());

    // Iterate over the desired time points
    double current_time = 0.0;
    while (current_time <= total_time) {
        // Get the interpolated robot state at `current_time`
        auto interpolated_state = std::make_shared<moveit::core::RobotState>(original_trajectory->getRobotModel());
        original_trajectory->getStateAtDurationFromStart(current_time / scaling_factor, interpolated_state);

        // Add the waypoint to the new trajectory
        reinterpolated_trajectory.addSuffixWayPoint(interpolated_state, waypoint_interval);

        // Move to the next time point
        current_time += waypoint_interval;
    }

    return reinterpolated_trajectory;
}

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
