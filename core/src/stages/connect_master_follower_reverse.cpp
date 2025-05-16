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

/* Authors: Kejia Chen
   Desc:    Connect states in a master-follower manner
*/

#include <moveit/task_constructor/stages/connect_master_follower_reverse.h>
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ConnectMFReverse");

ConnectMFReverse::ConnectMFReverse(const std::string& name, const GroupPlannerVector& planners,
                    const GroupPlannerVector& interpolation_planners,
                    const GroupCartPlannerVector& cartesian_planners,
                    const moveit::planning_interface::MoveGroupInterfacePtr& move_group_lead,
                    moveit_visual_tools::MoveItVisualTools visual_tools) 
    :Connect(name, planners), 
     move_group_lead_(move_group_lead), 
     visual_tools_(visual_tools),
     interpolation_planner_(interpolation_planners), 
     cartesian_planner_(cartesian_planners),
     lead_hand_to_tcp_transform_(Eigen::Isometry3d::Identity()),
     follow_hand_to_tcp_transform_(Eigen::Isometry3d::Identity()) 
{
	// setTimeout(1.0);
	setCostTerm(std::make_unique<cost::PathLength>());

	auto& p = properties();
	// p.declare<MergeMode>("merge_mode", WAYPOINTS, "merge mode");
	// p.declare<double>("max_distance", 1e-2, "maximally accepted distance between end and goal sate");
	// p.declare<moveit_msgs::msg::Constraints>("path_constraints", moveit_msgs::msg::Constraints(),
	//                                          "constraints for the first arm to maintain during trajectory");
	// properties().declare<TimeParameterizationPtr>("merge_time_parameterization",
	//                                               std::make_shared<TimeOptimalTrajectoryGeneration>());

    p.declare<std::string>("lead_group", "right_panda_arm", "Group name of the leader.");
    p.declare<std::string>("follow_group", "left_panda_arm", "Group name of the follower.");
    p.declare<std::string>("dual_group", "dual_arm", "Group name of the dual arm.");
    p.declare<GroupStringDict>("eefs", "vector of names of end-effector group");
    p.declare<double>("follow_grasp_offset", 0.15, "offset of the follower's grasping point in clip frame");
    p.declare<double>("track_offset", 0.1, "offset between leader and follower in leader's TCP frame");
    p.declare<geometry_msgs::msg::PoseStamped>("lead_grasp_pose", geometry_msgs::msg::PoseStamped(),
                                        "grasp pose of the leader arm in world frame");
    p.declare<geometry_msgs::msg::PoseStamped>("follow_grasp_pose", geometry_msgs::msg::PoseStamped(),
                                        "grasp pose of the leader arm in world frame");

    Eigen::Isometry3d default_hand_to_tcp_transform = Eigen::Isometry3d::Identity();
    default_hand_to_tcp_transform.translation().z() = 0.1034; // default z offset of the hand to TCP

    p.declare<Eigen::Isometry3d>("lead_hand_to_tcp_transform", default_hand_to_tcp_transform, "transform from lead hand to TCP");
    p.declare<Eigen::Isometry3d>("follow_hand_to_tcp_transform", default_hand_to_tcp_transform, "transform from follow hand to TCP");
}

void ConnectMFReverse::compute(const InterfaceState& from, const InterfaceState& to) {
	const auto& props = properties();
	double timeout = this->timeout();
	MergeMode mode = props.get<MergeMode>("merge_mode");
	double max_distance = props.get<double>("max_distance");
    lead_hand_to_tcp_transform_ = props.get<Eigen::Isometry3d>("lead_hand_to_tcp_transform");
    follow_hand_to_tcp_transform_ = props.get<Eigen::Isometry3d>("follow_hand_to_tcp_transform");

    RCLCPP_INFO_STREAM(LOGGER, "Computing dual-arm trajectory");

    std::vector<planning_scene::PlanningSceneConstPtr> intermediate_scenes;
    planning_scene::PlanningSceneConstPtr to_scene = to.scene();
    planning_scene::PlanningSceneConstPtr from_scene = from.scene();
    
    // intermediate_scenes.push_back(from_scene);

    planning_scene::PlanningScenePtr follower_intermediate_scene = from_scene->diff();
    planning_scene::PlanningScenePtr follower_final_scene = from_scene->diff();
    robot_trajectory::RobotTrajectoryPtr reversed_leader_trajectory;
    robot_trajectory::RobotTrajectoryPtr reversed_follower_trajectory;
    // moveit::core::RobotState intermediate_state = intermediate_scene->getCurrentStateNonConst();

    // Step 1: Compute trajectory for the first arm
    if (!computeSecondArmTrajectoryReverse(from, to, reversed_follower_trajectory, follower_intermediate_scene, follower_final_scene)) {
        auto failed_solution = std::make_shared<SubTrajectory>();
        failed_solution->markAsFailure("Follower arm trajectory planning failed.");
        connect(from, to, failed_solution);
        return;
    }

    // intermediate_scenes.push_back(follower_intermediate_scene);
    // intermediate_scenes.push_back(follower_final_scene);

    RCLCPP_INFO_STREAM(LOGGER, "Reversed Follower arm trajectory computed with " << reversed_follower_trajectory->getWayPointCount() << " waypoints");
    RCLCPP_INFO_STREAM(LOGGER, "Reversed Follower arm trajectory duration: " << reversed_follower_trajectory->getDuration());
    // intermediate_scenes.push_back(intermediate_scene->diff());

    // Step 2: Compute trajectory for the second arm based on the first arm's Cartesian trajectory
    RCLCPP_INFO_STREAM(LOGGER, "Intermediate scene update");
    std::vector<PlannerIdTrajectoryPair> reversed_leader_trajectories;
    std::vector<PlannerIdTrajectoryPair> reversed_follower_trajectories;
    // if (!computeSecondArmTrajectory(leader_trajectory, leader_trajectories ,to, follower_trajectory, follower_trajectories, leader_intermediate_scene, 
    //                               leader_final_scene, intermediate_scenes)) 
    if (!computeFirstArmTrajectoryReverse(reversed_follower_trajectory, reversed_follower_trajectories, from_scene, reversed_leader_trajectory, reversed_leader_trajectories, 
                                          follower_intermediate_scene, follower_final_scene, intermediate_scenes)) 
    {
        auto failed_solution = std::make_shared<SubTrajectory>();        
        failed_solution->markAsFailure("Leader arm trajectory planning failed.");
        connect(from, to, failed_solution);
        return;
    }

    // Step 3: Reverse the trajectories and connect them sequentially
    std::vector<PlannerIdTrajectoryPair> leader_trajectories;
    std::vector<PlannerIdTrajectoryPair> follower_trajectories;
    // Reverse the trajectory vectors
    for (auto& reversed_trajectory_pair: reversed_leader_trajectories) {
        auto& reversed_trajectory = reversed_trajectory_pair.trajectory;
        auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(*reversed_trajectory);
        trajectory->reverse();
        leader_trajectories.push_back({reversed_trajectory_pair.planner_id, trajectory});
    }
    for (auto& reversed_trajectory_pair: reversed_follower_trajectories) {
        auto& reversed_trajectory = reversed_trajectory_pair.trajectory;
        auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(*reversed_trajectory);
        trajectory->reverse();
        follower_trajectories.push_back({reversed_trajectory_pair.planner_id, trajectory});
    }
    // Reverse the trajectory
    auto leader_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(*reversed_leader_trajectory);
    leader_trajectory->reverse();
    auto follower_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(*reversed_follower_trajectory);
    follower_trajectory->reverse();
    // Reverse the order of scenes in intermediate_scenes
    std::reverse(intermediate_scenes.begin(), intermediate_scenes.end());

    // Combine trajectories into a valid dual-arm solution
    RCLCPP_INFO_STREAM(LOGGER, "Combining leader and follower arm trajectories");
    RCLCPP_INFO_STREAM(LOGGER, "Leader arm trajectory computed with " << leader_trajectory->getWayPointCount() << " waypoints");
    RCLCPP_INFO_STREAM(LOGGER, "Follower arm trajectory computed with " << follower_trajectory->getWayPointCount() << " waypoints");

//   Option 2: Merge each subtrajectory sequentially
    std::vector<PlannerIdTrajectoryPair> dual_sub_trajectories;
    for (int i = 0; i < leader_trajectories.size(); ++i) {    
        std::vector<PlannerIdTrajectoryPair> sub_trajectories;
        sub_trajectories.push_back({"leader_arm", leader_trajectories[i].trajectory});
        sub_trajectories.push_back({"follower_arm", follower_trajectories[i].trajectory});

        SubTrajectoryPtr dual_sub_trajectory = mergeIgnoreCollision(sub_trajectories, from.scene()->getCurrentState());
        if (!dual_sub_trajectory) {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to merge sub trajectories of phase " << i);
        continue;
        }

        dual_sub_trajectories.push_back({"dual_arm", dual_sub_trajectory->trajectory()});
        RCLCPP_INFO_STREAM(LOGGER, "Merge trajectory of phase " << i << " with " << dual_sub_trajectory->trajectory()->getWayPointCount() << " waypoints");
    }

    SolutionBasePtr solution;
    solution = makeSequential(dual_sub_trajectories, intermediate_scenes, from, to);
  
    // SolutionBasePtr solution;
    // solution = std::make_shared<SubTrajectory>(reversed_follower_trajectory, 0.0, "connect_master_follower");
  
    connect(from, to, solution);
}

bool ConnectMFReverse::ExtractSecondArmCartesianTrajectory(const robot_trajectory::RobotTrajectoryPtr& follower_trajectory,
                                                   const moveit::core::RobotState& final_goal_state,
                                                   std::vector<geometry_msgs::msg::Pose>& follower_tip_path,
                                                   std::vector<double>& path_time,
                                                  //  int& start_index,
                                                   double start_offset,
                                                //    std::vector<PlannerIdTrajectoryPair>& follower_trajectories,
                                                //    robot_trajectory::RobotTrajectoryPtr& follower_grasp_trajectory,
                                                   robot_trajectory::RobotTrajectoryPtr& follower_track_trajectory,
                                                   bool reverse)
{  
  // std::vector<geometry_msgs::msg::Pose> follower_tip_path;
  geometry_msgs::msg::Pose follower_start_hand_pose_msg;
  Eigen::Isometry3d follower_start_tip_pose;
 
  int start_index = follower_start_index_;

  bool track_start = false;

  // bool start = false;
  for (size_t i = 0; i < follower_trajectory->getWayPointCount(); ++i) {
    const auto& point = follower_trajectory->getWayPoint(i);
    double delta_time = (i == 0) ? 0.0 : follower_trajectory->getWayPointDurationFromStart(i) - follower_trajectory->getWayPointDurationFromStart(i - 1);    

    const std::string follower_ee_link = "left_panda_hand";
    if (!final_goal_state.knowsFrameTransform(follower_ee_link)) {
        RCLCPP_ERROR(LOGGER, "Link '%s' not found in model '%s'", follower_ee_link.c_str(), final_goal_state.getRobotModel()->getName().c_str());
        return false;
    }
    Eigen::Isometry3d ee_link_pose = point.getGlobalLinkTransform(follower_ee_link); // hand pose
    // Apply the transform to get the pose of the actual end-effector
    Eigen::Isometry3d tip_pose = ee_link_pose * follow_hand_to_tcp_transform_;
    if (start_index== -1){
        // find follower start index based on the distance
        // follower start pose
        if (i == 0) {
            tf2::convert(ee_link_pose, follower_start_hand_pose_msg);
            follower_start_tip_pose = tip_pose;
            continue;
        }
        
        if (i > 0){
            // check distance to the follower start point to decide the leader start point
            double distance_from_start  = (tip_pose.translation() - follower_start_tip_pose.translation()).norm();
            // Instead of strating from the first pose, start from the closest to the current pose of the leader arm
            if (distance_from_start < start_offset) {
            // RCLCPP_WARN(LOGGER, "Distance from the starting point is too narrow: %f", distance_from_start);
            continue;
            }
        }

        start_index = i;
    }else{
        if (i < start_index){
            if (reverse){
                if (!track_start) {
                    track_start = true;
                    RCLCPP_INFO_STREAM(LOGGER, "leader starts tracking from 0 until waypoint index: " << start_index);
                }
        
                // RCLCPP_INFO(LOGGER, "Waypoint %zu: Current Time: %f, Previous Time: %f, Delta Time: %f", i, 
                //                     follower_trajectory->getWayPointDurationFromStart(i), follower_trajectory->getWayPointDurationFromStart(i-1), delta_time);
        
                follower_track_trajectory->addSuffixWayPoint(point, delta_time);
            
                // Convert to geometry_msgs::Pose
                geometry_msgs::msg::Pose pose;
                tf2::convert(tip_pose, pose);
                follower_tip_path.push_back(pose);
        
                // log the time for each point
                path_time.push_back(follower_trajectory->getWayPointDurationFromStart(i));
            }
                // else{
            //     follower_grasp_trajectory->addSuffixWayPoint(point, delta_time);
            // }
        }else{
            if (reverse){
                // follower_grasp_trajectory->addSuffixWayPoint(point, delta_time);
            }else{
                if (!track_start) {
                    track_start = true;
                    RCLCPP_INFO_STREAM(LOGGER, "leader starts tracking from waypoint index: " << start_index << "until " << follower_trajectory->getWayPointCount());
                }
                follower_track_trajectory->addSuffixWayPoint(point, delta_time);
                // Convert to geometry_msgs::Pose
                geometry_msgs::msg::Pose pose;
                tf2::convert(tip_pose, pose);
                follower_tip_path.push_back(pose);
        
                // log the time for each point
                path_time.push_back(follower_trajectory->getWayPointDurationFromStart(i));
            }
        }
    } 
    
  }

  if (follower_tip_path.empty()) {
    RCLCPP_ERROR(LOGGER, "follower arm trajectory is empty.");
    return false;
  }

  return true;
}

bool ConnectMFReverse::computeFirstArmTrajectoryReverse(robot_trajectory::RobotTrajectoryPtr& follower_trajectory,
                                                  std::vector<PlannerIdTrajectoryPair>& follower_trajectories,
                                                  planning_scene::PlanningSceneConstPtr& goal_scene,
                                                  robot_trajectory::RobotTrajectoryPtr& leader_trajectory,
                                                  std::vector<PlannerIdTrajectoryPair>& leader_trajectories,
                                                  // robot_trajectory::RobotTrajectoryPtr& dual_trajectory,
                                                  planning_scene::PlanningScenePtr& follow_intermediate_scene,
                                                  planning_scene::PlanningScenePtr& follow_final_scene,
                                                  std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes) 
{

  const auto& props = properties();
  const moveit::core::RobotState& initial_state = follow_final_scene->getCurrentState();
  Eigen::Quaterniond leader_initial_orientation(initial_state.getGlobalLinkTransform("right_panda_hand").rotation());
  leader_jmg_ = initial_state.getJointModelGroup(props.get<std::string>("lead_group"));

  const moveit::core::RobotState& final_goal_state = goal_scene->getCurrentState();
  intermediate_scenes.push_back(goal_scene);

  // double start_offset = 0.15;
  double track_offset = props.get<double>("track_offset");
  double grasp_follower_offset = props.get<double>("follow_grasp_offset");
  double grasp_leader_offset = track_offset + grasp_follower_offset; // not used if leader_start_index_ is already set

  RCLCPP_INFO_STREAM(LOGGER, "Computing trajectory for the fisrt arm");

  /* Extract the Cartesian trajectory of the first arm */ 
  std::vector<geometry_msgs::msg::Pose> follower_tip_path;
  std::vector<double> path_time_sequnce;
  // int follower_start_index = 0;

  // reverse the follower trajectory
//   auto reversed_follower_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(*follower_trajectory);
//   reversed_follower_trajectory->reverse();
  // std::cout << "Reversed follower trajectory:" << std::endl;
  // reversed_follower_trajectory->print(std::cout);
  // follower_trajectory = reversed_follower_trajectory;

//   double follower_duration_original = reversed_follower_trajectory->getDuration();
//   RCLCPP_INFO_STREAM(LOGGER, "follower arm trajectory duration: " << follower_duration_original);

//   auto reversed_follower_track_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(reversed_follower_trajectory->getRobotModel(), follow_jmg_);
  auto follower_track_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(follower_trajectory->getRobotModel(), follow_jmg_);
//   auto follower_to_goal_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(follower_trajectory->getRobotModel(), follow_jmg_);
  if (!ExtractSecondArmCartesianTrajectory(follower_trajectory, final_goal_state, follower_tip_path, path_time_sequnce, 
                                          grasp_follower_offset, follower_track_trajectory, true)) {
    RCLCPP_INFO_STREAM(LOGGER, "Failed to extract follower arm Cartesian trajectory.");
    return false;
  }

  RCLCPP_INFO_STREAM(LOGGER, "Follower arm trajectory with " << follower_trajectory->getWayPointCount() << " waypoints, " 
                              << follower_track_trajectory->getWayPointCount() << " for tracking");
                            //   << follower_to_goal_trajectory->getWayPointCount() << " for goal");
//   for (size_t i = 0; i < follower_tip_path.size(); ++i) {
//     const auto& pose_msg = follower_tip_path[i];
//     Eigen::Isometry3d original_pose;
//     tf2::fromMsg(pose_msg, original_pose);
//     std::cout << "follower tip position: " << original_pose.translation().transpose() << std::endl;
//     // std::cout << "leader tip orientation: " << Eigen::Quaterniond(original_pose.rotation()).coeffs().transpose() << std::endl;
//   }


  /* Obtain the Cartesian path for the leader from the follower's Cartesian path */
  std::vector<geometry_msgs::msg::Pose> leader_tip_path;
//   std::vector<geometry_msgs::msg::Pose> leader_hand_path;

  Eigen::Isometry3d leader_hand_grasp_pose;
  Eigen::Isometry3d leader_tip_grasp_pose;
  Eigen::Quaterniond follow_hand_start_orientation;
  Eigen::Quaterniond leader_start_orientation(leader_initial_orientation);
  RCLCPP_INFO_STREAM(LOGGER, "Leader arm start orientation: " << leader_start_orientation.coeffs().transpose());

  // set lead final orientation
  geometry_msgs::msg::PoseStamped leader_grasp_pos_msg = props.get<geometry_msgs::msg::PoseStamped>("lead_grasp_pose");
  Eigen::Isometry3d leader_grasp_pose_tcp;
  tf2::fromMsg(leader_grasp_pos_msg.pose, leader_grasp_pose_tcp);
  Eigen::Quaterniond grasp_orientation(leader_grasp_pose_tcp.rotation());
  RCLCPP_INFO_STREAM(LOGGER, "Leader arm grasp orientation: " << grasp_orientation.coeffs().transpose());
  // grasp's yaw and start's pitch and roll
  Eigen::Quaterniond combined_orientation = combineRotations(grasp_orientation, leader_start_orientation);
  Eigen::Quaterniond leader_final_orientation = combined_orientation;

//   Eigen::Quaterniond leader_final_orientation(leader_start_orientation);
  RCLCPP_INFO_STREAM(LOGGER, "Leader arm final orientation: " << leader_final_orientation.coeffs().transpose());

  // Define the transform from the tip frame to "leader_ee_link"
//   Eigen::Isometry3d lead_hand_frame_transform = Eigen::Isometry3d::Identity();
//   lead_hand_frame_transform.translation().z() = -lead_hand_to_tcp_transform_.translation().z();  // Offset along the Z-axis

  int length = follower_tip_path.size();
  for (size_t i = 0; i < length; ++i) {
    double percentage = (double)i / (double)length;

    const auto& pose_msg = follower_tip_path[i];
    Eigen::Isometry3d original_pose;
    tf2::fromMsg(pose_msg, original_pose);

    // preserve only yaw rotation
    Eigen::Matrix3d original_rot = original_pose.rotation();
    double original_yaw = std::atan2(original_rot(1, 0), original_rot(0, 0));  // equivalent to yaw from rotation matrix
    Eigen::AngleAxisd yaw_rotation(original_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond yaw_quat(yaw_rotation);
    Eigen::Matrix3d yaw_rot_matrix = yaw_quat.toRotationMatrix();

    // Define the offset in the ee frame
    Eigen::Vector3d offset_ee(track_offset, 0.0, 0.0);  // offset along the X-axis of the follower's EEF frame
    // Transform the offset to the world frame
    Eigen::Vector3d offset_in_world =  yaw_rot_matrix * offset_ee;
    Eigen::Isometry3d lead_tip_pose;
    lead_tip_pose.translation() = original_pose.translation() + offset_in_world;

    leader_start_orientation.normalize();
    leader_final_orientation.normalize();

    // interpolate the orientation
    Eigen::Quaterniond interpolated_orientation = leader_start_orientation.slerp(percentage, leader_final_orientation);
    // interpolated_orientation.normalize();
    lead_tip_pose.linear() = interpolated_orientation.toRotationMatrix();

    // pose at hand
    // Eigen::Isometry3d lead_hand_pose = lead_tip_pose*lead_hand_frame_transform;

    geometry_msgs::msg::Pose lead_tip_pose_msg;
    tf2::convert(lead_tip_pose, lead_tip_pose_msg);
    leader_tip_path.push_back(lead_tip_pose_msg);

    // geometry_msgs::msg::Pose lead_hand_pose_msg;
    // tf2::convert(lead_hand_pose, lead_hand_pose_msg);
    // leader_hand_path.push_back(lead_hand_pose_msg);
  }

//   Test the last orientation of cartesian wapyoints
//   geometry_msgs::msg::Pose last_pose_hand = leader_hand_path.back();
//   tf2::fromMsg(last_pose_hand, leader_hand_grasp_pose);
  geometry_msgs::msg::Pose last_pose_tcp = leader_tip_path.back();
  tf2::fromMsg(last_pose_tcp, leader_tip_grasp_pose);

  Eigen::Vector3d grasp_hand_position = leader_hand_grasp_pose.translation();
  Eigen::Quaterniond grasp_hand_orientation(leader_hand_grasp_pose.rotation());
  grasp_hand_orientation.normalize();
  RCLCPP_INFO_STREAM(LOGGER, "leader arm first step should end at hand position: " << grasp_hand_position.transpose());
  RCLCPP_INFO_STREAM(LOGGER, "leader arm first step should end at hand orientation: " << grasp_hand_orientation.coeffs().transpose());

//   Eigen::Vector3d path_hand_position(leader_hand_path[0].position.x, leader_hand_path[0].position.y, leader_hand_path[0].position.z);
//   Eigen::Quaterniond path_hand_orientation(leader_hand_path[0].orientation.w, leader_hand_path[0].orientation.x, 
//       leader_hand_path[0].orientation.y, leader_hand_path[0].orientation.z);
//   path_hand_orientation.normalize();
//   RCLCPP_INFO_STREAM(LOGGER, "leader arm second step shoud start from hand position: " << path_hand_position.transpose());
//   RCLCPP_INFO_STREAM(LOGGER, "leader arm second step shoud start from hand orientation: " << path_hand_orientation.coeffs().transpose());

  // Test if position leader_hand_grasp_pose == leader_hand_path[0]
  Eigen::Vector3d grasp_tip_position = leader_tip_grasp_pose.translation();
  Eigen::Quaterniond grasp_tip_orientation(leader_tip_grasp_pose.rotation());
  RCLCPP_INFO_STREAM(LOGGER, "leader arm first step should end at tcp position: " << grasp_tip_position.transpose());
  RCLCPP_INFO_STREAM(LOGGER, "leader arm first step should end at tcp orientation: " << grasp_tip_orientation.coeffs().transpose());

//   Eigen::Vector3d path_tip_position(leader_tip_path[0].position.x, leader_tip_path[0].position.y, leader_tip_path[0].position.z);
//   Eigen::Quaterniond path_tip_orientation(leader_tip_path[0].orientation.w, leader_tip_path[0].orientation.x, leader_tip_path[0].orientation.y, 
//     leader_tip_path[0].orientation.z);
//   RCLCPP_INFO_STREAM(LOGGER, "leader arm second step shoud start from tcp position: " << path_tip_position.transpose());
//   RCLCPP_INFO_STREAM(LOGGER, "leader arm second step shoud start from tcp orientation: " << path_tip_orientation.coeffs().transpose());

  visual_tools_.publishPath(leader_tip_path, rviz_visual_tools::YELLOW, rviz_visual_tools::MEDIUM);
  visual_tools_.trigger();

//   check lead_tip_path
  for (size_t i = 0; i < leader_tip_path.size(); ++i) {
    const auto& pose_msg = leader_tip_path[i];
    Eigen::Isometry3d original_pose;
    tf2::fromMsg(pose_msg, original_pose);
    Eigen::Quaterniond original_orientation(original_pose.rotation());
    std::cout << "leader tip position: " << original_pose.translation().transpose() << " orientation: " << original_orientation.coeffs().transpose() << std::endl;
  }

  /******************************************************************/
  /*** Step 1: BACKWARD Planning: move second arm starting from "to" to track follower until grasping point ***/
  /*****************************************************************/
  robot_trajectory::RobotTrajectoryPtr leader_track_trajectory;

  planning_scene::PlanningSceneConstPtr start = follow_intermediate_scene;
  planning_scene::PlanningScenePtr intermediate_scene = start->diff();
//   planning_scene::PlanningScenePtr intermediate_scene = follow_intermediate_scene->diff();
// //   update the scene with the follower arm's grasp state
//   const moveit::core::RobotState& follower_grasp_state = follower_track_trajectory->getLastWayPoint();
// //   const moveit::core::RobotState& follower_grasp_state = follow_intermediate_scene->getCurrentState();
//   std::vector<double> follower_joint_positions;
//   follower_grasp_state.copyJointGroupPositions(follow_jmg_, follower_joint_positions);

//   moveit::core::RobotState& state = intermediate_scene->getCurrentStateNonConst();
//   state.setJointGroupPositions(follow_jmg_, follower_joint_positions);
//   state.update();  //

  double fraction_lead = FirstArmFollow(intermediate_scene, leader_tip_path, leader_track_trajectory);

  if (fraction_lead < 1.0) {
    RCLCPP_ERROR(LOGGER, "leader arm failed to track the second arm's trajectory. Fraction: %f", fraction_lead);
    return false;
  }else{
    RCLCPP_INFO(LOGGER, "leader arm successfully tracked the second arm's cartesian path with %zu waypoints", leader_track_trajectory->getWayPointCount());

    std::cout << "leader track trajectory:" << std::endl;
    leader_track_trajectory->print(std::cout);

    // // // Update intermediate scene
    // const moveit::core::RobotState& leader_final_state = leader_track_trajectory->getLastWayPoint();
    // std::vector<double> leader_joint_positions;
    // leader_final_state.copyJointGroupPositions(leader_jmg_, leader_joint_positions);

    // moveit::core::RobotState& state = intermediate_scene->getCurrentStateNonConst();
    // state.setJointGroupPositions(leader_jmg_, leader_joint_positions);
    // state.update(true);  // Ensure consistency

    // RCLCPP_INFO_STREAM(LOGGER, "leader arm state updated in intermediate_scene.");
  }

  // set tip_link to get cumulative arc length
  if (!leader_track_trajectory->setTipLink("right_panda_hand")) {
    RCLCPP_ERROR(LOGGER, "Failed to set tip link for leader trajectory.");
    return false;
  }

  if (!follower_track_trajectory->setTipLink("left_panda_hand")) {
    RCLCPP_ERROR(LOGGER, "Failed to set tip link for follower trajectory.");
    return false;
  }

  double follower_total_distance = follower_track_trajectory->getWayPointDistanceFromStart(follower_track_trajectory->getWayPointCount());
  std::cout << "follower track trajectory total distance: " << follower_total_distance << std::endl;

  double leader_total_distance = leader_track_trajectory->getWayPointDistanceFromStart(leader_track_trajectory->getWayPointCount());
  std::cout << "leader track trajectory total distance: " << leader_total_distance << std::endl;

  // resample the follower trajectory to match the leader trajectory
  robot_trajectory::RobotTrajectoryPtr follower_track_resample_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(follower_trajectory->getRobotModel(), follow_jmg_);

  // Find the corresponding follower trajectory point for each leader trajectory point with desried distance plus/minus tolerance
  double start_arc_percent = 0.0;
  double end_arc_percent = 1.0;
  double tolerance = 0.005; // 5mm
  for (size_t i = 0; i < leader_track_trajectory->getWayPointCount(); ++i) {
    double lead_arc_length = leader_track_trajectory->getWayPointDistanceFromStart(i);
    const moveit::core::RobotState& leader_state = leader_track_trajectory->getWayPoint(i);
    Eigen::Isometry3d leader_tip_transform = leader_state.getGlobalLinkTransform("right_panda_hand") * lead_hand_to_tcp_transform_;
    // std::cout << "leader tip: "<< leader_tip_transform.translation().transpose()<<std::endl;

    bool match_found = false;  // Track if a match is found

    // search for the follower trajectory point with incremental of 0.002
    for (double s=start_arc_percent; s<=end_arc_percent; s+=0.001){
      double follower_arc_length = s * follower_total_distance;
      auto follower_interpolated_state = std::make_shared<moveit::core::RobotState>(follower_trajectory->getRobotModel());
      follower_track_trajectory->getStateAtArcDistanceFromStart(follower_arc_length, follower_interpolated_state);

      // check the distance between the follower and leader trajectory
      Eigen::Isometry3d follower_tip_transform = follower_interpolated_state->getGlobalLinkTransform("left_panda_hand") * follow_hand_to_tcp_transform_;
      double distance = (follower_tip_transform.translation() - leader_tip_transform.translation()).norm();
    //   std::cout << "follower tip: "<< follower_tip_transform.translation().transpose() << " distance: " << distance << std::endl;

      if (abs(distance - track_offset) < tolerance) {
        RCLCPP_INFO_STREAM(LOGGER, "Point "<< i<<" Arc length: " << follower_arc_length << ", distance: " << distance);
        follower_track_resample_trajectory->addSuffixWayPoint(follower_interpolated_state, 0.1);

        start_arc_percent = s;
        match_found = true;
        break;
      }
    }

    if (!match_found) {
      RCLCPP_WARN_STREAM(LOGGER, "No matching point found for leader trajectory at index: " << i);
    }
  }
  
  if (!follower_track_resample_trajectory->setTipLink("left_panda_hand")) {
    RCLCPP_ERROR(LOGGER, "Failed to set tip link for follower trajectory.");
    return false;
  }

  std::cout << "follower track resampled trajectory:" << std::endl;
  follower_track_resample_trajectory->print(std::cout);

  if (follower_track_resample_trajectory->getWayPointCount() != leader_track_trajectory->getWayPointCount()) {
    RCLCPP_ERROR(LOGGER, "follower and leader TRACK trajectories have different number of waypoints! follower: %zu, leader: %zu",
    follower_track_resample_trajectory->getWayPointCount(), leader_track_trajectory->getWayPointCount());
  }

  // phase_1_scene: scene after tracking
  planning_scene::PlanningScenePtr phase_1_scene; // TODO@KejiaChen: is phase_3_scene the same as intermediate_scene?
  planning_scene::PlanningScenePtr temp_start = start->diff();
  updateDualIntermediateState(leader_track_trajectory->getLastWayPoint(), follower_track_resample_trajectory->getLastWayPoint(), temp_start, phase_1_scene);

//   // replace the first part of the follower trajectory with the resampled trajectory
//   robot_trajectory::RobotTrajectoryPtr temp_follower_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(follower_trajectory->getRobotModel(), follow_jmg_);
//   follower_trajectory = follower_track_resample_trajectory;
  
//   for (size_t i = 0; i > follower_start_index_; ++i) {
//     follower_trajectory->addPrefixWayPoint(temp_follower_trajectory->getWayPoint(i), temp_follower_trajectory->getWayPointDurationFromStart(i));
//   }
//   follower_start_index_ = follower_trajectory->getWayPointCount();

  robot_trajectory::RobotTrajectoryPtr original_follower_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(follower_trajectory->getRobotModel(), follow_jmg_);
  // Append them to trajectory
  leader_trajectory = leader_track_trajectory;
  follower_trajectory = follower_track_resample_trajectory;
  // Add them to the vector
  leader_trajectories.push_back({"leader_arm", leader_track_trajectory});
  follower_trajectories.push_back({"follower_arm", follower_track_resample_trajectory});
  
  // intermediate secene is added in a reverse order
  intermediate_scenes.push_back(phase_1_scene);

  // dual_trajectory->append(*dual_track_trajectory, 0.0);

  // // Get current orientation
  // Eigen::Quaterniond next_reached_orientaiton(intermediate_scene->getCurrentState().getGlobalLinkTransform("left_panda_hand").rotation());
  // RCLCPP_INFO_STREAM(LOGGER, "leader arm reached orientation after second step: " << next_reached_orientaiton.coeffs().transpose());


  /***********************************************************************************/
  /*** Step 2: BACKWARD Planning: Move both arms from grasping point to the start ***/
  /**********************************************************************************/
  bool success=false;
  robot_trajectory::RobotTrajectoryPtr leader_to_goal_trajectory;
  robot_trajectory::RobotTrajectoryPtr follower_to_goal_trajectory;
  std::vector<robot_trajectory::RobotTrajectoryPtr> follower_to_goal_trajectories;
  std::vector<robot_trajectory::RobotTrajectoryPtr> leader_to_goal_trajectories;

  // Plan joint trajectory for the follower arm
  // tension_scene: leader tensioned, follower moved to the goal
  planning_scene::PlanningScenePtr tension_scene = phase_1_scene->diff();
  // update the scene with the follower arm's goal state
  std::vector<double> follower_joint_positions;
  final_goal_state.copyJointGroupPositions(follow_jmg_, follower_joint_positions);
  moveit::core::RobotState& state = tension_scene->getCurrentStateNonConst();
  state.setJointGroupPositions(follow_jmg_, follower_joint_positions);  
  state.update(true);

  for (const auto& pair : planner_) {
    if (pair.first == props.get<std::string>("follow_group")) {
      planning_scene::PlanningSceneConstPtr end = tension_scene;
      planning_scene::PlanningSceneConstPtr start = phase_1_scene;
      // Plan trajectory
      auto result = pair.second->plan(start, end, follow_jmg_, props.get<double>("timeout"), follower_to_goal_trajectory);
      success = bool(result);

      if (!success) {
      RCLCPP_ERROR_STREAM(LOGGER, "follower arm planning to goal failed: " << result.message);
      return false;
      }

      RCLCPP_INFO_STREAM(LOGGER, "follower arm planning to goal succeeded with " << follower_to_goal_trajectory->getWayPointCount() << " waypoints.");
      //   return true;
    }
  }

  // Plan joint trajectory for the leader arm
  for (const auto& pair : planner_) {
    if (pair.first == props.get<std::string>("lead_group")) {
      planning_scene::PlanningSceneConstPtr end = goal_scene;
      planning_scene::PlanningSceneConstPtr start = tension_scene;
      // Plan trajectory
      auto result = pair.second->plan(start, end, leader_jmg_, props.get<double>("timeout"), leader_to_goal_trajectory);
      success = bool(result);

      if (!success) {
      RCLCPP_ERROR_STREAM(LOGGER, "leader arm planning to goal failed: " << result.message);
      return false;
      }

      RCLCPP_INFO_STREAM(LOGGER, "leader arm planning to goal succeeded with " << leader_to_goal_trajectory->getWayPointCount() << " waypoints.");
      //   return true;
    }
  }

  /* Time Adjustment */
  RCLCPP_INFO_STREAM(LOGGER, "Adding pause for the leader and follower arm trajectories");
  auto delayed_leader_to_goal_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(leader_to_goal_trajectory->getRobotModel(), leader_jmg_);
  // Get the time when finishing the first step
  double leader_to_goal_duration = leader_to_goal_trajectory->getWayPointDurationFromStart(leader_to_goal_trajectory->getWayPointCount());
  double follower_to_goal_duration = follower_to_goal_trajectory->getWayPointDurationFromStart(follower_to_goal_trajectory->getWayPointCount());

//   double follower_second_step_start_time = follower_to_goal_duration;
//   double follower_second_step_end_time = follower_duration_original;
  if (leader_to_goal_trajectory){
    // Force the leader_trajectory to start after the follower_trajectory finishes the first step
    if (!splitTrajectoryWithPause(leader_to_goal_trajectory, follower_to_goal_duration, 0, delayed_leader_to_goal_trajectory, leader_to_goal_trajectories, true)) {
        RCLCPP_ERROR(LOGGER, "Failed to delay the leader trajectory.");
        return false;
    }
    // leader_trajectory = delayed_leader_to_goal_trajectory;

    // Force the follower_trajectory to wait for the leader_trajectory to finish the first step
    // double pause_duration = leader_to_goal_duration - follower_trajectory->getWayPointDurationFromStart(follower_start_index);
    double pause_duration = leader_to_goal_duration;
    RCLCPP_INFO_STREAM(LOGGER, "Follower pause duration: " << pause_duration);
    auto follower_to_goal_trajectory_with_pause = std::make_shared<robot_trajectory::RobotTrajectory>(follower_trajectory->getRobotModel(), follow_jmg_);
    if (!splitTrajectoryWithPause(follower_to_goal_trajectory, leader_to_goal_duration, follower_to_goal_trajectory->getWayPointCount(), follower_to_goal_trajectory_with_pause, follower_to_goal_trajectories, false)) {
        RCLCPP_ERROR(LOGGER, "Failed to extend the follower trajectory.");
        return false;
    }
    // follower_trajectory = follower_to_goal_trajectory_with_pause; // follower trajectory until dual arm tracking starts

    // follower_second_step_start_time = follower_second_step_start_time + pause_duration;
    // follower_second_step_end_time = follower_second_step_end_time + pause_duration;

   
  }

  // Phase 1: follower moving to the first position while the leader remains still
  if (follower_to_goal_trajectories[0]->getWayPointCount() != leader_to_goal_trajectories[0]->getWayPointCount()) {
    RCLCPP_ERROR(LOGGER, "follower and leader trajectories have different number of waypoints for phase 2! follower: %zu, leader: %zu",
    follower_to_goal_trajectories[0]->getWayPointCount(), leader_to_goal_trajectories[0]->getWayPointCount());
    int add_count = follower_to_goal_trajectories[0]->getWayPointCount() - leader_to_goal_trajectories[0]->getWayPointCount();
    for (size_t i = 0; i < add_count; ++i) {
      leader_to_goal_trajectories[0]->addSuffixWayPoint(leader_to_goal_trajectories[0]->getLastWayPoint(), 0.1);
    }
    RCLCPP_INFO_STREAM(LOGGER, "leader arm trajectory updated with additional pause to " << leader_to_goal_trajectories[0]->getWayPointCount() << " waypoints");
  }
  follower_trajectory->append(*follower_to_goal_trajectories[0], 0.1);
  leader_trajectory->append(*leader_to_goal_trajectories[0], 0.1);
  follower_trajectories.push_back({"follower_arm", follower_to_goal_trajectories[0]});
  leader_trajectories.push_back({"leader_arm", leader_to_goal_trajectories[0]});
  
  planning_scene::PlanningScenePtr phase_2_scene;
  updateDualIntermediateState(leader_to_goal_trajectories[0]->getLastWayPoint(), follower_to_goal_trajectories[0]->getLastWayPoint(), phase_1_scene, phase_2_scene);
  intermediate_scenes.push_back(phase_2_scene);

  // Phase 2: leader moving to the first position while the follower remains still
  if (follower_to_goal_trajectories[1]->getWayPointCount() != leader_to_goal_trajectories[1]->getWayPointCount()) {
    RCLCPP_ERROR(LOGGER, "follower and leader trajectories have different number of waypoints for phase 3! follower: %zu, leader: %zu",
    follower_to_goal_trajectories[1]->getWayPointCount(), leader_to_goal_trajectories[1]->getWayPointCount());
    int add_count = leader_to_goal_trajectories[1]->getWayPointCount() - follower_to_goal_trajectories[1]->getWayPointCount();
    for (size_t i = 0; i < add_count; ++i) {
      follower_to_goal_trajectories[1]->addSuffixWayPoint(follower_to_goal_trajectories[1]->getLastWayPoint(), 0.1);
    }
    RCLCPP_INFO_STREAM(LOGGER, "follower arm trajectory updated with additional pause to " << follower_to_goal_trajectories[1]->getWayPointCount() << " waypoints");
  }
  follower_trajectory->append(*follower_to_goal_trajectories[1], 0.1);
  leader_trajectory->append(*leader_to_goal_trajectories[1], 0.1);
  follower_trajectories.push_back({"follower_arm",follower_to_goal_trajectories[1]});
  leader_trajectories.push_back({"leader_arm",leader_to_goal_trajectories[1]});

//   planning_scene::PlanningScenePtr phase_3_scene;
//   updateDualIntermediateState(leader_to_goal_trajectories[1]->getLastWayPoint(), follower_to_goal_trajectories[1]->getLastWayPoint(), phase_2_scene, phase_3_scene);
//   intermediate_scenes.push_back(phase_3_scene);
  intermediate_scenes.push_back(goal_scene);

  return true;
}

void ConnectMFReverse::updateDualIntermediateState(const moveit::core::RobotState& leader_state,
                                            const moveit::core::RobotState& follower_state,
                                            planning_scene::PlanningScenePtr& start,
                                            planning_scene::PlanningScenePtr& end) 
{
  end = start->diff();
  moveit::core::RobotState& dual_state = end->getCurrentStateNonConst();

  std::vector<double> leader_joint_positions;
  leader_state.copyJointGroupPositions(leader_jmg_, leader_joint_positions);
  dual_state.setJointGroupPositions(leader_jmg_, leader_joint_positions);
  dual_state.update();  // Ensure consistency

  std::vector<double> follower_joint_positions;
  follower_state.copyJointGroupPositions(follow_jmg_, follower_joint_positions);
  dual_state.setJointGroupPositions(follow_jmg_, follower_joint_positions);

  dual_state.update();  // Ensure consistency
}

bool ConnectMFReverse::computeSecondArmTrajectoryReverse(const InterfaceState& from, const InterfaceState& to,
                                                        robot_trajectory::RobotTrajectoryPtr& follower_trajectory,
                                                        planning_scene::PlanningScenePtr& intermediate_scene,
                                                        planning_scene::PlanningScenePtr& final_scene) 
{
  const auto& props = properties();
  const moveit::core::RobotState& final_state = from.scene()->getCurrentState();
  const auto& path_constraints = props.get<moveit_msgs::msg::Constraints>("path_constraints");

  RCLCPP_INFO(LOGGER, "Computing trajectory for the follower arm.");

  double track_offset = props.get<double>("track_offset");
  double grasp_follower_offset = props.get<double>("follow_grasp_offset");
//   double grasp_leader_offset = track_offset + grasp_follower_offset;

  /* Option 1: Direct planning to the final scene*/
  // // Plan joint trajectory for the leader arm
  // for (const auto& pair : planner_) {
  //   if (pair.first == props.get<std::string>("lead_group")) {
  //     planning_scene::PlanningSceneConstPtr start = from.scene();
  //     leader_jmg_ = final_state.getJointModelGroup(pair.first);
  //     RCLCPP_INFO_STREAM(LOGGER, "leader group name: " << leader_jmg_->getName());
  //     final_scene = start->diff();
  //     moveit::core::RobotState& goal_state = final_scene->getCurrentStateNonConst();

  //     // Set the joint group goal
  //     std::vector<double> positions;
  //     final_state.copyJointGroupPositions(leader_jmg_, positions);
  //     goal_state.setJointGroupPositions(leader_jmg_, positions);
  //     goal_state.update();

  //     // Plan trajectory
  //     auto result = pair.second->plan(start, final_scene, leader_jmg_, props.get<double>("timeout"),
  //                                     leader_trajectory, path_constraints);

  //     if (!result) {
  //       RCLCPP_ERROR(LOGGER, "Leader arm planning from intermediate to final failed.");
  //       return false;
  //     }

  //     RCLCPP_INFO_STREAM(LOGGER, "Leader arm trajectory planning success ");

  //     return true;
  //   }
  // }
  // return false;

  /* Option 2: Passing intermediate waypoint*/
  planning_scene::PlanningSceneConstPtr start = to.scene();
  follower_trajectory.reset(new robot_trajectory::RobotTrajectory(start->getRobotModel(), follow_jmg_));
  
  robot_trajectory::RobotTrajectoryPtr traj_1;
                                            
  for (const auto& pair: cartesian_planner_) {
    if (pair.first == props.get<std::string>("follow_group")) {

      follow_jmg_ = final_state.getJointModelGroup(pair.first);

      const moveit::core::RobotState& start_state = start->getCurrentState();
      const moveit::core::LinkModel* eef_link = start_state.getLinkModel("left_panda_hand");

      // Get current pose of EEF in world frame
      Eigen::Isometry3d current_pose = start_state.getGlobalLinkTransform(eef_link);
      Eigen::Quaterniond clip_orientation(current_pose.rotation());
      RCLCPP_INFO_STREAM(LOGGER, "Follower arm current position: " << current_pose.translation().transpose());
      RCLCPP_INFO_STREAM(LOGGER, "Follower arm current orientation: " << clip_orientation.coeffs().transpose());

      // Offset from current link frame to ik_frame (used by solver)
      Eigen::Isometry3d offset = current_pose.inverse() * current_pose;  // Identity in this case, because ik_pose_world = current

      // Option 1: Get the follower_grasp_pose in world frame as target
      geometry_msgs::msg::PoseStamped follower_grasp_pos_msg = props.get<geometry_msgs::msg::PoseStamped>("follow_grasp_pose");
      Eigen::Isometry3d follower_grasp_pose_tcp;
      tf2::fromMsg(follower_grasp_pos_msg.pose, follower_grasp_pose_tcp);
      Eigen::Quaterniond grasp_orientation(follower_grasp_pose_tcp.rotation());
      
      Eigen::Quaterniond combined_orientation = combineRotations(grasp_orientation, clip_orientation);

      follower_grasp_pose_tcp.linear() = combined_orientation.toRotationMatrix();
      Eigen::Isometry3d follower_grasp_pose_eef = follower_grasp_pose_tcp * (follow_hand_to_tcp_transform_).inverse();

      Eigen::Isometry3d target_pose = follower_grasp_pose_eef;
      Eigen::Quaterniond target_orientation(target_pose.rotation());
      
      // // Option 2: Get the follower_grasp_pose in EEF frame as target
      // // Apply local +X offset
      // Eigen::Isometry3d target_pose = current_pose * Eigen::Translation3d(grasp_follower_offset, 0.0, 0.0);

      // TODO@KejiaChen: check if the target pose is valid
      
      RCLCPP_INFO_STREAM(LOGGER, "Follower arm grasp position: " << target_pose.translation().transpose());
      RCLCPP_INFO_STREAM(LOGGER, "Follower arm grasp orientation: " << target_orientation.coeffs().transpose());

      // Call the correct `plan()` method
      auto result_1 = pair.second->plan(start, *eef_link, offset, target_pose, follow_jmg_,
                                      props.get<double>("timeout"), traj_1, path_constraints);

      if (!result_1) {
        RCLCPP_ERROR(LOGGER, "Follower arm planning to intermediate pose failed.");
        return false;
      }

    }
  }

  if (!traj_1->empty()) {
    intermediate_scene = start->diff();
    moveit::core::RobotState& intermediate_state = intermediate_scene->getCurrentStateNonConst();
    const moveit::core::RobotState& traj1_final_state = traj_1->getLastWayPoint();
    std::vector<double> intermediate_positions;
    traj1_final_state.copyJointGroupPositions(follow_jmg_, intermediate_positions);
    intermediate_state.setJointGroupPositions(follow_jmg_, intermediate_positions);
    intermediate_state.update();

    // print the trajectory
    // std::cout << "follower arm trajectory:" << std::endl;
    // traj_1->print(std::cout);

    follower_trajectory = traj_1;
    follower_start_index_ = follower_trajectory->getWayPointCount();
    RCLCPP_INFO_STREAM(LOGGER, "follower arm trajectory start index: " << follower_start_index_);
  } else {
    RCLCPP_WARN(LOGGER, "Cartesian trajectory is empty.");
  }
  
  // Plan joint trajectory for the follower arm
  for (const auto& pair : planner_) {
    if (pair.first == props.get<std::string>("follow_group")) {

      // ---- Step 3: Plan from intermediate to final ----
      final_scene = start->diff();
      // update only leader_jmg_
      moveit::core::RobotState& goal_state = final_scene->getCurrentStateNonConst();
      std::vector<double> goal_positions;
      final_state.copyJointGroupPositions(follow_jmg_, goal_positions);
      goal_state.setJointGroupPositions(follow_jmg_, goal_positions);
      goal_state.update();

      // Plan trajectory
      robot_trajectory::RobotTrajectoryPtr traj_2(new robot_trajectory::RobotTrajectory(start->getRobotModel(), follow_jmg_));
      auto result_2 = pair.second->plan(intermediate_scene, final_scene, follow_jmg_, props.get<double>("timeout"),
                                      traj_2, path_constraints);
      if (!result_2) {
        RCLCPP_ERROR(LOGGER, "follower arm planning from intermediate to final failed.");
        return false;
      }
      
      // ---- Step 4: Concatenate and assign ----
      follower_trajectory->append(*traj_2, 0.0);

      return true;
    }
  }
  return false;
}

double ConnectMFReverse::FirstArmFollow(planning_scene::PlanningScenePtr& intermediate_scene,
                                std::vector<geometry_msgs::msg::Pose> leader_tip_path,
                                robot_trajectory::RobotTrajectoryPtr& lead_trajectory) {

  const auto& props = properties();
  move_group_lead_->setPoseReferenceFrame("world");
  move_group_lead_->setStartState(intermediate_scene->getCurrentState());

  auto lead_scene = intermediate_scene->diff();

  // Define the transform from the "leader_ee_link" to the actual end-effector frame
  Eigen::Matrix3d leader_flange_to_ee_rotation;
  leader_flange_to_ee_rotation << 0.7071, 0.7071, 0,
                            -0.7071, 0.7071, 0,
                            0, 0, 1;
  Eigen::Isometry3d lead_flange_to_tcp_transform = lead_hand_to_tcp_transform_;
  // TODO@KejiaChen: AUTO CORRECT
  lead_flange_to_tcp_transform.translation().z() = lead_flange_to_tcp_transform.translation().z() + 0.036;
  lead_flange_to_tcp_transform.linear() = leader_flange_to_ee_rotation*lead_hand_to_tcp_transform_.linear();
  RCLCPP_INFO_STREAM(LOGGER, "Leader arm flange to tcp translation: " << lead_flange_to_tcp_transform.translation().transpose() << 
                             " orientation: " << Eigen::Quaterniond(lead_flange_to_tcp_transform.rotation()).coeffs().transpose());

  // compute joint trajectory from the cartesian path
  moveit_msgs::msg::RobotTrajectory lead_trajectory_msg;

  std::cout<<"leader arm tip path has " << leader_tip_path.size() << " waypoints." << std::endl;

  // TODO@KejiaChen: set time_parameterization to false will lead to problems
  double fraction_lead = move_group_lead_->computeCartesianPath(leader_tip_path, 0.0005, 5.0, lead_trajectory_msg, true,
                                                                    nullptr, lead_flange_to_tcp_transform, true);
  // leader_cartesian_planner_.plan(lead_scene, leader_jmg_->getLinkModel("left_panda_hand"),
  //                                lead_grasp_frame_transform, 
  // leader_hand_path, 0.01, 0.0, lead_trajectory_msg);

  lead_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(
      lead_scene->getRobotModel(), lead_scene->getRobotModel()->getJointModelGroup(props.get<std::string>("lead_group")));
  lead_trajectory->setRobotTrajectoryMsg(lead_scene->getCurrentState(), lead_trajectory_msg);
  
  RCLCPP_INFO_STREAM(LOGGER, "Leader arm planning finishesd with " << fraction_lead * 100.0 << "% success.");
  RCLCPP_INFO_STREAM(LOGGER, "Leader arm trajectory has " << lead_trajectory->getWayPointCount() << " waypoints.");

  return fraction_lead;
}

SubTrajectoryPtr ConnectMFReverse::mergeIgnoreCollision(const std::vector<PlannerIdTrajectoryPair>& sub_trajectories,
                                  // const std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes, // unused
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
	robot_trajectory::RobotTrajectoryPtr trajectory = task_constructor::merge(subs, state, jmg, *timing); // merge and smoothing
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

bool ConnectMFReverse::splitTrajectoryWithPause(const robot_trajectory::RobotTrajectoryPtr& trajectory,
                                          const double pause_duration,
                                          const int split_index,
                                          robot_trajectory::RobotTrajectoryPtr& split_trajectory,
                                          std::vector<robot_trajectory::RobotTrajectoryPtr>& split_trajectories,
                                          bool if_return_full)
{
  auto first_part = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory->getRobotModel(), trajectory->getGroup());
  auto second_part = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory->getRobotModel(), trajectory->getGroup());

  RCLCPP_INFO_STREAM(LOGGER, "Splitting the trajectory at index: " << split_index);

  for (size_t i = 0; i < split_index; ++i) {
      double delta = (i == 0) ? 0.0 : trajectory->getWayPointDurationFromStart(i) - trajectory->getWayPointDurationFromStart(i-1);
      first_part->addSuffixWayPoint(trajectory->getWayPoint(i), delta);
  }
  std::cout << "First part has " << first_part->getWayPointCount() << " waypoints for duration: " 
    << first_part->getWayPointDurationFromStart(first_part->getWayPointCount() - 1) << std::endl;

  // Add waypoints to the second part
  for (size_t i = split_index; i < trajectory->getWayPointCount(); ++i) {
      double delta = (i == 0) ? 0.0 : trajectory->getWayPointDurationFromStart(i) - trajectory->getWayPointDurationFromStart(i-1);
      second_part->addSuffixWayPoint(trajectory->getWayPoint(i), delta);
  }
  std::cout << "Second part has " << second_part->getWayPointCount() << " waypoints for duration: " 
    << second_part->getWayPointDurationFromStart(second_part->getWayPointCount() - 1) - second_part->getWayPointDurationFromStart(0) << std::endl;

  // create a pause part
  auto pause_part = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory->getRobotModel(), trajectory->getGroup());
  moveit::core::RobotState pause_state(trajectory->getRobotModel());
  if (first_part->getWayPointCount() > 0) {
    moveit::core::RobotState& first_part_last_state = *first_part->getLastWayPointPtr();
    // Ensure zero velocity at the split points
    first_part_last_state.zeroVelocities();
    pause_state = first_part_last_state;
  } else{
    moveit::core::RobotState& second_part_first_state = *second_part->getFirstWayPointPtr();
    // Ensure zero velocity at the split points
    second_part_first_state.zeroVelocities();
    pause_state = second_part_first_state;
  }

  // Add duplicate waypoints for the pause duration
  for (double t = 0.1; t <= pause_duration; t += 0.1) {
      pause_part->addSuffixWayPoint(pause_state, 0.1);
  }
  std::cout << "Pause part has " << pause_part->getWayPointCount() << " waypoints for duration: " << pause_duration << std::endl;

  // trajectory_processing::IterativeParabolicTimeParameterization time_param;
  // // Smooth the first part
  // if (!time_param.computeTimeStamps(*first_part)) {
  //     RCLCPP_ERROR(LOGGER, "Time parameterization failed for the first part.");
  // }
  // // Smooth the pause part (no motion, so time stamps remain consistent)
  // if (!time_param.computeTimeStamps(*pause_part)) {
  //     RCLCPP_ERROR(LOGGER, "Time parameterization failed for the pause part.");
  // }
  // // Smooth the second part
  // if (!time_param.computeTimeStamps(*second_part)) {
  //     RCLCPP_ERROR(LOGGER, "Time parameterization failed for the second part.");
  // }

  // auto split_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory->getRobotModel(), trajectory->getGroup());

  // Add waypoints from the first part
  split_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory->getRobotModel(), trajectory->getGroup());
  if (first_part->getWayPointCount() > 0) {
    split_trajectory->append(*first_part, 0.0);
    split_trajectories.push_back(first_part);
  }
  split_trajectory->append(*pause_part, 0.0);
  split_trajectories.push_back(pause_part);
  if (second_part->getWayPointCount() > 0) {
    split_trajectory->append(*second_part, 0.0);
    split_trajectories.push_back(second_part);
  }

  std::cout << "Split trajectory has " << split_trajectory->getWayPointCount() << " waypoints." << std::endl;
  return true;
}

robot_trajectory::RobotTrajectory ConnectMFReverse::reinterpolateTrajectory(const robot_trajectory::RobotTrajectoryPtr& original_trajectory, 
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

void ConnectMFReverse::splitGroupFromState(const moveit::core::JointModelGroup* group,
  const moveit::core::RobotState& dual_state,
  moveit::core::RobotState& single_group_state)
{
/* For a state containing multiple groups, split only the specified group from the state */

// Set the single group state to default values
single_group_state.setToDefaultValues();

// Copy joint positions
std::vector<double> joint_positions;
dual_state.copyJointGroupPositions(group, joint_positions);
single_group_state.setJointGroupPositions(group, joint_positions);

// Copy velocities if available
if (dual_state.hasVelocities())
{
std::vector<double> joint_velocities;
dual_state.copyJointGroupVelocities(group, joint_velocities);
single_group_state.setJointGroupVelocities(group, joint_velocities);
}

// Copy accelerations if available
if (dual_state.hasAccelerations())
{
std::vector<double> joint_accelerations;
dual_state.copyJointGroupAccelerations(group, joint_accelerations);
single_group_state.setJointGroupAccelerations(group, joint_accelerations);
}

// // Copy effort if available
// if (dual_state.hasEffort())
// {
// std::vector<double> joint_effort;
// dual_state.copyJointGroupEffort(group, joint_effort);
// single_group_state.setJointGroupEffort(group, joint_effort);
// }

// Update the state to ensure consistency
single_group_state.update();
}

bool ConnectMFReverse::isTargetPoseCollidingInEEF(const planning_scene::PlanningSceneConstPtr& scene,
                                          moveit::core::RobotState& robot_state, 
                                          EigenSTL::vector_Isometry3d& poses,
                                          std::vector<const moveit::core::LinkModel*>& links,
                                          const moveit::core::JointModelGroup* jmg,
                                          collision_detection::CollisionResult* collision_result) {
if (poses.size() != links.size())
{
  RCLCPP_ERROR(LOGGER, "The number of poses does not match the number of links.");
  return false;
}

for (size_t i = 0; i < links.size(); ++i)
{
  const moveit::core::LinkModel* link = links[i];
  Eigen::Isometry3d& pose = poses[i];

  // consider all rigidly connected parent links as well
  const moveit::core::LinkModel* parent = moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(link);
  Eigen::Isometry3d transformed_pose = pose;
  if (parent != link)  
  // ensure that the collision check considers the entire rigidly connected structure of the end-effector, not just the specified link
  transformed_pose = pose * robot_state.getGlobalLinkTransform(link).inverse() * robot_state.getGlobalLinkTransform(parent);

  // place links at given pose
  robot_state.updateStateWithLinkAt(parent, transformed_pose);
}

robot_state.updateCollisionBodyTransforms();

// disable collision checking for parent links (except links fixed to root)
auto acm = scene->getAllowedCollisionMatrix();
for (size_t i = 0; i < links.size(); ++i){
  std::vector<const std::string*> pending_links;  // parent link names that might be rigidly connected to root
  const moveit::core::LinkModel* link = links[i];
  const moveit::core::LinkModel* parent = moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(link);
  while (parent) {
    pending_links.push_back(&parent->getName());
    auto link_ = parent;
    const moveit::core::JointModel* joint = link_->getParentJointModel();
    parent = joint->getParentLinkModel();

    if (joint->getType() != moveit::core::JointModel::FIXED) { //except links fixed to root
      for (const std::string* name : pending_links)
      acm.setDefaultEntry(*name, true);
      pending_links.clear();
    }
  }
}

// check collision with the world using the padded version
collision_detection::CollisionRequest req;
collision_detection::CollisionResult result;
req.contacts = (collision_result != nullptr);
if (jmg)
req.group_name = jmg->getName();
collision_detection::CollisionResult& res = collision_result ? *collision_result : result;
scene->checkCollision(req, res, robot_state, acm);
return res.collision;
}


Eigen::Quaterniond ConnectMFReverse::combineRotations(Eigen::Quaterniond grasp_orientation, 
                                                      Eigen::Quaterniond clip_orientation)
{
    /* Combine the yaw rotaton from grasping pose and other rotations from clipping pose*/

    // Extract yaw angle from grasping pose
    Eigen::Matrix3d grasp_rot = grasp_orientation.toRotationMatrix();
    double grasp_yaw = std::atan2(grasp_rot(1, 0), grasp_rot(0, 0));  // equivalent to yaw from rotation matrix
    // Build a pure yaw rotation around world Z
    Eigen::AngleAxisd yaw_rotation(grasp_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond grasp_yaw_quat(yaw_rotation);

    // Remove yaw from clipping pose by rotating back around world Z
    Eigen::Matrix3d clip_rot = clip_orientation.toRotationMatrix();
    double clip_yaw = std::atan2(clip_rot(1,0), clip_rot(0,0));
    Eigen::AngleAxisd clip_yaw_inv(-clip_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond clip_rot_without_yaw(clip_yaw_inv * clip_rot);

    // Now apply follower's yaw
    Eigen::Quaterniond combined_orientation = grasp_yaw_quat * clip_rot_without_yaw;

    return combined_orientation;
}

}  // namespace connect_master_follower
}  // namespace task_constructor
}  // namespace moveit
