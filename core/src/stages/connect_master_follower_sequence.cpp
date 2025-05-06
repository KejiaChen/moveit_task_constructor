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

#include <moveit/task_constructor/stages/connect_master_follower_sequence.h>
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ConnectMFSeq");

ConnectMFSeq::ConnectMFSeq(const std::string& name, const GroupPlannerVector& planners) 
    :Connect(name, planners)
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

    Eigen::Isometry3d default_hand_to_tcp_transform = Eigen::Isometry3d::Identity();
    default_hand_to_tcp_transform.translation().z() = 0.1034; // default z offset of the hand to TCP
}

void ConnectMFSeq::compute(const InterfaceState& from, const InterfaceState& to) {
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
    const moveit::core::RobotState& leader_final_state = leader_trajectory->getLastWayPoint();
    std::vector<double> leader_joint_positions;
    leader_final_state.copyJointGroupPositions(leader_jmg_, leader_joint_positions);
    intermediate_state.setJointGroupPositions(leader_jmg_, leader_joint_positions);
    intermediate_state.update();  // Ensure consistency

    // intermediate_scenes.push_back(intermediate_scene->diff());

    // Step 2: Compute trajectory for the second arm based on the first arm's Cartesian trajectory
    // planning_scene::PlanningScenePtr final_scene = intermediate_scene->diff();
    RCLCPP_INFO_STREAM(LOGGER, "Intermediate scene update");
    std::vector<PlannerIdTrajectoryPair> leader_trajectories;
    std::vector<PlannerIdTrajectoryPair> follower_trajectories;
    if (!computeSecondArmTrajectory(leader_trajectory, leader_trajectories ,to, follower_trajectory, follower_trajectories, intermediate_scene, intermediate_scenes)) {
        auto failed_solution = std::make_shared<SubTrajectory>();
        failed_solution->markAsFailure("Follower arm trajectory planning failed.");
        connect(from, to, failed_solution);
        return;
    }

    // intermediate_scenes.push_back(intermediate_scene->diff()); 

    // Combine trajectories into a valid dual-arm solution
    // RCLCPP_INFO_STREAM(LOGGER, "Combining leader and follower arm trajectories");
    // RCLCPP_INFO_STREAM(LOGGER, "Leader arm trajectory computed with " << leader_trajectory->getWayPointCount() << " waypoints");
    // RCLCPP_INFO_STREAM(LOGGER, "Follower arm trajectory computed with " << follower_trajectory->getWayPointCount() << " waypoints");

  // Option 2: Merge each subtrajectory sequentially
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
  
	connect(from, to, solution);
}

bool ConnectMFSeq::computeSecondArmTrajectory(robot_trajectory::RobotTrajectoryPtr& leader_trajectory,
                                          std::vector<PlannerIdTrajectoryPair>& leader_trajectories,
                                          const InterfaceState& to,
                                          robot_trajectory::RobotTrajectoryPtr& follower_trajectory,
                                          std::vector<PlannerIdTrajectoryPair>& follower_trajectories,
                                          // robot_trajectory::RobotTrajectoryPtr& dual_trajectory,
                                          planning_scene::PlanningScenePtr& lead_final_scene,
                                          std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes)
{

  const auto& props = properties();
  const moveit::core::RobotState& initial_state = lead_final_scene->getCurrentState();
  const moveit::core::RobotState& final_goal_state = to.scene()->getCurrentState();
  // const moveit::core::JointModelGroup* follow_jmg_;

  RCLCPP_INFO_STREAM(LOGGER, "Computing trajectory for the second arm");

  /* Extract the Cartesian trajectory of the first arm */ 
  std::vector<geometry_msgs::msg::Pose> leader_tip_path;
  std::vector<double> path_time_sequnce;
//   int leader_start_index = 0;

  double leader_duration_original = leader_trajectory->getDuration();
  RCLCPP_INFO_STREAM(LOGGER, "Leader arm trajectory duration: " << leader_duration_original);

  // visual_tools_.publishPath(follower_tip_path, rviz_visual_tools::YELLOW, rviz_visual_tools::MEDIUM);
  // visual_tools_.trigger();

  /********************************************************************/
  /*** Step 1: Move second arm to the first arm's starting position ***/
  /********************************************************************/
  bool success=false;
  robot_trajectory::RobotTrajectoryPtr follower_to_start_trajectory;
  std::vector<robot_trajectory::RobotTrajectoryPtr> leader_to_start_trajectories;
  std::vector<robot_trajectory::RobotTrajectoryPtr> follower_to_start_trajectories;

  // Plan joint trajectory for the follower arm
  for (const auto& pair : planner_) {
    if (pair.first == props.get<std::string>("follow_group")) {
      planning_scene::PlanningSceneConstPtr start = lead_final_scene;
      follow_jmg_ = final_goal_state.getJointModelGroup(pair.first);
      planning_scene::PlanningScenePtr end = start->diff();
      moveit::core::RobotState& goal_state = end->getCurrentStateNonConst();
      
      // Set the joint group goal
      std::vector<double> positions;
      final_goal_state.copyJointGroupPositions(follow_jmg_, positions);
      goal_state.setJointGroupPositions(follow_jmg_, positions);
      goal_state.update();

      // Plan trajectory
      auto result = pair.second->plan(start, end, follow_jmg_, props.get<double>("timeout"), follower_to_start_trajectory);
      success = bool(result);

      if (!success) {
        RCLCPP_ERROR_STREAM(LOGGER, "Follower arm planning to start failed: " << result.message);
        return false;
      }
      
      RCLCPP_INFO_STREAM(LOGGER, "Follower arm planning to start succeeded with " << follower_to_start_trajectory->getWayPointCount() << " waypoints.");
    }
  }

  // Validate and update follower arm state in intermediate_scene
  planning_scene::PlanningScenePtr intermediate_scene = lead_final_scene->diff();
  if (follower_to_start_trajectory) {
    const moveit::core::RobotState& follower_final_state = follower_to_start_trajectory->getLastWayPoint();
    std::vector<double> follower_joint_positions;
    follower_final_state.copyJointGroupPositions(follow_jmg_, follower_joint_positions);

    moveit::core::RobotState& state = intermediate_scene->getCurrentStateNonConst();
    state.setJointGroupPositions(follow_jmg_, follower_joint_positions);
    state.update();  // Ensure consistency

    RCLCPP_INFO_STREAM(LOGGER, "Follower arm state updated in intermediate_scene.");
  }

  // follower_trajectory = follower_to_start_trajectory;

  /* Time Adjustment */
  int leader_start_index = leader_trajectory->getWayPointCount() - 1;
  auto delayed_follower_to_start_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(follower_to_start_trajectory->getRobotModel(), follow_jmg_);
  // Get the time when finishing the first step
  double first_step_end_time = follower_to_start_trajectory->getWayPointDurationFromStart(follower_to_start_trajectory->getWayPointCount());
  double leader_first_step_end_time = leader_trajectory->getWayPointDurationFromStart(leader_start_index);

  double leader_second_step_start_time = leader_first_step_end_time;
  double leader_second_step_end_time = leader_duration_original;
  if (follower_to_start_trajectory){
    // Force the leader_trajectory to wait for the follower_trajectory to finish the first step
    // double pause_duration = first_step_end_time - leader_trajectory->getWayPointDurationFromStart(leader_start_index);
    double pause_duration = first_step_end_time;
    RCLCPP_INFO_STREAM(LOGGER, "Pause duration: " << pause_duration);
    auto leader_first_trajectory_with_pause = std::make_shared<robot_trajectory::RobotTrajectory>(leader_trajectory->getRobotModel(), leader_jmg_);
    if (!splitTrajectoryWithPause(leader_trajectory, pause_duration, leader_start_index, leader_first_trajectory_with_pause, leader_to_start_trajectories, false)) {
      RCLCPP_ERROR(LOGGER, "Failed to split the leader trajectory.");
      return false;
    }
    leader_trajectory = leader_first_trajectory_with_pause; // leader trajectory until dual arm tracking starts

    leader_second_step_start_time = leader_second_step_start_time + pause_duration;
    leader_second_step_end_time = leader_second_step_end_time + pause_duration;

    // Force the follower_trajectory to start after the leader_trajectory finishes the first step
    if (!splitTrajectoryWithPause(follower_to_start_trajectory, leader_first_step_end_time, 0, delayed_follower_to_start_trajectory, follower_to_start_trajectories, true)) {
      RCLCPP_ERROR(LOGGER, "Failed to delay the follower trajectory.");
      return false;
    }
    follower_trajectory = delayed_follower_to_start_trajectory;
  }

  // Phase 1: Leader moving to the first position while the follower remains still
  if (leader_to_start_trajectories[0]->getWayPointCount() != follower_to_start_trajectories[0]->getWayPointCount()) {
    RCLCPP_WARN(LOGGER, "Leader and follower trajectories have different number of waypoints for phase 1! Leader: %zu, Follower: %zu",
                 leader_to_start_trajectories[0]->getWayPointCount(), follower_to_start_trajectories[0]->getWayPointCount());
    alignTrajectoriesCount(leader_to_start_trajectories[0], follower_to_start_trajectories[0]);
  }
  leader_trajectories.push_back({"leader_arm", leader_to_start_trajectories[0]});
  follower_trajectories.push_back({"follower_arm", follower_to_start_trajectories[0]});
  
  planning_scene::PlanningScenePtr phase_1_scene;
  updateDualIntermediateState(leader_to_start_trajectories[0]->getLastWayPoint(), follower_to_start_trajectories[0]->getLastWayPoint(), lead_final_scene, phase_1_scene);
  intermediate_scenes.push_back(phase_1_scene);

  // Phase 2: Follower moving to the first position while the leader remains still
  if (leader_to_start_trajectories[1]->getWayPointCount() != follower_to_start_trajectories[1]->getWayPointCount()) {
    RCLCPP_ERROR(LOGGER, "Leader and follower trajectories have different number of waypoints for phase 2! Leader: %zu, Follower: %zu",
                 leader_to_start_trajectories[1]->getWayPointCount(), follower_to_start_trajectories[1]->getWayPointCount());
    alignTrajectoriesCount(leader_to_start_trajectories[1], follower_to_start_trajectories[1]);
  }
  leader_trajectories.push_back({"leader_arm",leader_to_start_trajectories[1]});
  follower_trajectories.push_back({"follower_arm",follower_to_start_trajectories[1]});
  
  planning_scene::PlanningScenePtr phase_2_scene;
  updateDualIntermediateState(leader_to_start_trajectories[1]->getLastWayPoint(), follower_to_start_trajectories[1]->getLastWayPoint(), phase_1_scene, phase_2_scene);
  intermediate_scenes.push_back(phase_2_scene);

  return true;
}

void ConnectMFSeq::alignTrajectoriesCount(robot_trajectory::RobotTrajectoryPtr& trajectory_1,
                                         robot_trajectory::RobotTrajectoryPtr& trajectory_2)
{
    int add_count = trajectory_2->getWayPointCount() - trajectory_1->getWayPointCount();
    if (add_count > 0) {
        for (size_t i = 0; i < add_count; ++i) {
            trajectory_1->addSuffixWayPoint(trajectory_1->getLastWayPoint(), trajectory_1->getWayPointDurationFromStart(trajectory_1->getWayPointCount()));
        }
        RCLCPP_INFO_STREAM(LOGGER, "Leader arm trajectory updated with additional pause to " << trajectory_1->getWayPointCount() << " waypoints");
    }else if (add_count < 0) {
        // use absolute value of add_count
        for (size_t i = 0; i < -add_count; ++i) {
            trajectory_2->addSuffixWayPoint(trajectory_2->getLastWayPoint(), trajectory_2->getWayPointDurationFromStart(trajectory_2->getWayPointCount()));
        }
        RCLCPP_INFO_STREAM(LOGGER, "Follower arm trajectory updated with additional pause to " << trajectory_2->getWayPointCount() << " waypoints");
    }
}

void ConnectMFSeq::updateDualIntermediateState(const moveit::core::RobotState& leader_state,
                                            const moveit::core::RobotState& follower_state,
                                            planning_scene::PlanningScenePtr& start,
                                            planning_scene::PlanningScenePtr& end) 
{
  end = start->diff();
  moveit::core::RobotState& dual_state = end->getCurrentStateNonConst();

  std::vector<double> leader_joint_positions;
  leader_state.copyJointGroupPositions(leader_jmg_, leader_joint_positions);
  dual_state.setJointGroupPositions(leader_jmg_, leader_joint_positions);

  std::vector<double> follower_joint_positions;
  follower_state.copyJointGroupPositions(follow_jmg_, follower_joint_positions);
  dual_state.setJointGroupPositions(follow_jmg_, follower_joint_positions);

  dual_state.update();  // Ensure consistency
}

bool ConnectMFSeq::computeFirstArmTrajectory(const InterfaceState& from, const InterfaceState& to,
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
      leader_jmg_ = final_goal_state.getJointModelGroup(pair.first);
      RCLCPP_INFO_STREAM(LOGGER, "leader group name: " << leader_jmg_->getName());
      intermediate_scene = start->diff();
      moveit::core::RobotState& goal_state = intermediate_scene->getCurrentStateNonConst();

      // Set the joint group goal
      std::vector<double> positions;
      final_goal_state.copyJointGroupPositions(leader_jmg_, positions);
      goal_state.setJointGroupPositions(leader_jmg_, positions);
      goal_state.update();

      // Plan trajectory
      auto result = pair.second->plan(start, intermediate_scene, leader_jmg_, props.get<double>("timeout"),
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

SubTrajectoryPtr ConnectMFSeq::mergeIgnoreCollision(const std::vector<PlannerIdTrajectoryPair>& sub_trajectories,
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

bool ConnectMFSeq::splitTrajectoryWithPause(const robot_trajectory::RobotTrajectoryPtr& trajectory,
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
  if (if_return_full) {
    split_trajectory->append(*second_part, 0.0);
    split_trajectories.push_back(second_part);
  }

  std::cout << "Split trajectory has " << split_trajectory->getWayPointCount() << " waypoints." << std::endl;
  return true;
}

void ConnectMFSeq::splitGroupFromState(const moveit::core::JointModelGroup* group,
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

}  // namespace connect_master_follower
}  // namespace task_constructor
}  // namespace moveit
