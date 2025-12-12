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
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>

#define COLOR_RED "\033[31m"
#define COLOR_RESET "\033[0m"

using namespace trajectory_processing;

namespace moveit {
namespace task_constructor {
namespace stages {

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("ConnectMFReverse");

ConnectMFReverse::ConnectMFReverse(const std::string& name, const GroupPlannerVector& planners,
                    const GroupPlannerVector& interpolation_planners,
                    const GroupCartPlannerVector& cartesian_planners,
                    const GroupPipePlannerVector& chomp_planners,
                    const GroupPlannerVector& hand_planners,
                    const moveit::planning_interface::MoveGroupInterfacePtr& move_group_lead,
                    moveit_visual_tools::MoveItVisualTools visual_tools) 
    :Connect(name, planners, hand_planners), 
     move_group_lead_(move_group_lead), 
     visual_tools_(visual_tools),
     interpolation_planner_(interpolation_planners), 
     cartesian_planner_(cartesian_planners),
     chomp_planner_(chomp_planners),
     lead_hand_to_tcp_transform_(Eigen::Isometry3d::Identity()),  // panda hand to TCP transform
     follow_hand_to_tcp_transform_(Eigen::Isometry3d::Identity()), // panda hand to TCP transform
     lead_flange_to_tcp_transform_(Eigen::Isometry3d::Identity()), // panda link8 to TCP transform
     follow_flange_to_tcp_transform_(Eigen::Isometry3d::Identity())
{
	// setTimeout(1.0);
	setCostTerm(std::make_unique<cost::PathLength>());

	auto& p = properties();
	// p.declare<MergeMode>("merge_mode", WAYPOINTS, "merge mode");
	// p.declare<double>("max_distance", 1e-2, "maximally accepted distance between end and goal sate");
	// p.declare<moveit_msgs::msg::Constraints>("path_constraints", moveit_msgs::msg::Constraints(),
	//                                          "constraints for the first arm to maintain during trajectory");
    p.declare<moveit_msgs::msg::Constraints>("lead_path_constraints", moveit_msgs::msg::Constraints(),
                                            "constraints for the leader arm to maintain during trajectory");
    p.declare<moveit_msgs::msg::Constraints>("follow_path_constraints", moveit_msgs::msg::Constraints(),
                                            "constraints for the follower arm to maintain during trajectory");
	// properties().declare<TimeParameterizationPtr>("merge_time_parameterization",
	//                                               std::make_shared<TimeOptimalTrajectoryGeneration>());

    p.declare<std::string>("lead_group", "right_panda_arm", "Group name of the leader.");
    p.declare<std::string>("lead_hand_group", "right_hand", "Group name of the leader's hand.");
    p.declare<std::string>("lead_base_link", "right_panda_link0", "Base link of the leader.");
    p.declare<std::string>("follow_group", "left_panda_arm", "Group name of the follower.");
    p.declare<std::string>("follow_hand_group", "left_hand", "Group name of the follower's hand.");
    p.declare<std::string>("follow_base_link", "left_panda_link0", "Base link of the follower.");
    p.declare<std::string>("dual_group", "dual_arm", "Group name of the dual arm.");
    p.declare<std::string>("grasp_frame", "clip5", "grasp frame name (the last fixture frame)");
    p.declare<std::string>("goal_frame", "clip6", "goal frame name");
    p.declare<GroupStringDict>("eefs", "vector of names of end-effector group");
    p.declare<double>("follow_grasp_offset", 0.05, "offset of the follower's grasping point in clip frame"); // not used
    p.declare<double>("track_offset", 0.1, "offset between leader and follower in leader's TCP frame");
    p.declare<geometry_msgs::msg::PoseStamped>("lead_grasp_pose", geometry_msgs::msg::PoseStamped(),
                                        "grasp pose of the leader arm in world frame");
    p.declare<geometry_msgs::msg::PoseStamped>("follow_grasp_pose", geometry_msgs::msg::PoseStamped(),
                                        "grasp pose of the leader arm in world frame");
    p.declare<Eigen::Quaterniond>("quat_clip2ee", Eigen::Quaterniond::Identity(), "rotation from clip frame to end-effector frame");
    p.declare<int>("clip_sign", -1, "sign of the follower's approach direction along the clip Y axis");
    p.declare<bool>("attach_pull_cable", true, "whether to attach the pulling cable, for control groups");
    p.declare<bool>("attach_transport_cable", true, "whether to attach the transport cable, for control groups");

    Eigen::Isometry3d default_hand_to_tcp_transform = Eigen::Isometry3d::Identity();
    default_hand_to_tcp_transform.translation().z() = 0.1034; // default z offset of the hand to TCP

    Eigen::Isometry3d default_flange_to_tcp_transform = default_hand_to_tcp_transform;
    Eigen::Matrix3d leader_flange_to_hand_rotation;
    leader_flange_to_hand_rotation << 0.7071, 0.7071, 0,
                                    -0.7071, 0.7071, 0,
                                    0, 0, 1;
    default_flange_to_tcp_transform.linear() = leader_flange_to_hand_rotation * default_hand_to_tcp_transform.linear();

    p.declare<Eigen::Isometry3d>("hand_to_tcp_transform", default_hand_to_tcp_transform, "transform from hand to TCP");
    p.declare<Eigen::Isometry3d>("lead_flange_to_tcp_transform", default_flange_to_tcp_transform, "transform from lead flange (panda_link8) to TCP");
    p.declare<Eigen::Isometry3d>("follow_flange_to_tcp_transform", default_flange_to_tcp_transform, "transform from follow flange (panda_link8) to TCP");

    p.declare<bool>("primary_is_leader", false, "whether the primary arm is the leader");
}

void ConnectMFReverse::init(const core::RobotModelConstPtr& robot_model) {
  Connect::init(robot_model);
  // init planners
  for (const GroupPlannerVector::value_type& pair : interpolation_planner_) {
    if (!pair.second)
      throw InitStageException(*this, "invalid planner for group: " + pair.first);
    else
      pair.second->init(robot_model);
  }
  for (const GroupCartPlannerVector::value_type& pair : cartesian_planner_) {
    if (!pair.second)
      throw InitStageException(*this, "invalid planner for group: " + pair.first);
    else
      pair.second->init(robot_model);
  }
  for (const GroupPlannerVector::value_type& pair : chomp_planner_) {
    if (!pair.second)
      throw InitStageException(*this, "invalid planner for group: " + pair.first);
    else
      pair.second->init(robot_model);
  }
}

void ConnectMFReverse::compute(const InterfaceState& from, const InterfaceState& to) {
    const auto& props = properties();
    double timeout = this->timeout();
    MergeMode mode = props.get<MergeMode>("merge_mode");
    double max_distance = props.get<double>("max_distance");
    lead_hand_to_tcp_transform_ = props.get<Eigen::Isometry3d>("lead_hand_to_tcp_transform");
    follow_hand_to_tcp_transform_ = props.get<Eigen::Isometry3d>("follow_hand_to_tcp_transform");
    lead_flange_to_tcp_transform_ = props.get<Eigen::Isometry3d>("lead_flange_to_tcp_transform");
    follow_flange_to_tcp_transform_ = props.get<Eigen::Isometry3d>("follow_flange_to_tcp_transform");

    std::vector<planning_scene::PlanningSceneConstPtr> intermediate_scenes;
    planning_scene::PlanningSceneConstPtr to_scene = to.scene();
    planning_scene::PlanningSceneConstPtr from_scene = from.scene();
    // intermediate_scenes.push_back(from_scene);

    RCLCPP_INFO_STREAM(LOGGER, "Computing dual-arm trajectory");
    auto ns = props.get<std::string>("marker_ns");

    /* Initialize ArmRoles */
    // Fill roles from properties
    leader_role.group       = props.get<std::string>("lead_group");
    leader_role.hand_group  = props.get<std::string>("lead_hand_group");
    leader_role.base_link   = props.get<std::string>("lead_base_link");
    leader_role.hand_link   = "right_panda_hand";
    leader_role.hand_to_tcp_transform = lead_hand_to_tcp_transform_;
    leader_role.flange_to_tcp_transform = lead_flange_to_tcp_transform_;
    leader_role.path_constraints = props.get<moveit_msgs::msg::Constraints>("lead_path_constraints");

    follower_role.group       = props.get<std::string>("follow_group");
    follower_role.hand_group  = props.get<std::string>("follow_hand_group");
    follower_role.base_link   = props.get<std::string>("follow_base_link");
    follower_role.hand_link   = "left_panda_hand";
    follower_role.hand_to_tcp_transform = follow_hand_to_tcp_transform_;
    follower_role.flange_to_tcp_transform = follow_flange_to_tcp_transform_;
    follower_role.path_constraints = props.get<moveit_msgs::msg::Constraints>("follow_path_constraints");

    const auto& start_state = from_scene->getCurrentState();
    leader_role.arm_jmg   = start_state.getJointModelGroup(leader_role.group);
    leader_role.hand_jmg  = start_state.getJointModelGroup(leader_role.hand_group);
    RCLCPP_INFO_STREAM(LOGGER, "Leader hand group name: " << leader_role.hand_jmg->getName());

    follower_role.arm_jmg = start_state.getJointModelGroup(follower_role.group);
    follower_role.hand_jmg= start_state.getJointModelGroup(follower_role.hand_group);
    RCLCPP_INFO_STREAM(LOGGER, "Follower hand group name: " << follower_role.hand_jmg->getName());

    follower_role.grasp_offset = props.get<double>("follow_grasp_offset");
    leader_role.grasp_offset = props.get<double>("track_offset") + follower_role.grasp_offset;

    robot_trajectory::RobotTrajectoryPtr reversed_leader_trajectory;
    robot_trajectory::RobotTrajectoryPtr reversed_follower_trajectory;
    robot_trajectory::RobotTrajectoryPtr reversed_follower_hand_trajectory;
    std::vector<PlannerIdTrajectoryPair> reversed_leader_trajectories;
    std::vector<PlannerIdTrajectoryPair> reversed_follower_trajectories;
    // moveit::core::RobotState intermediate_state = intermediate_scene->getCurrentStateNonConst();

    // relative orientation between the follower and leader's tcp at to_scene
    Eigen::Matrix3d R_follower = to_scene->getCurrentState().getGlobalLinkTransform(follower_role.hand_link).rotation().eval();
    Eigen::Matrix3d R_leader   = to_scene->getCurrentState().getGlobalLinkTransform(leader_role.hand_link).rotation().eval();
    Eigen::Quaterniond follower_target_orientation(R_follower);
    Eigen::Quaterniond leader_target_orientation(R_leader);
    transport_rotation_ = follower_target_orientation.inverse() * leader_target_orientation;
    transport_rotation_.normalize();

    /** Step 0: Sample grasping direction and orientation */
    // std::mt19937 rng(42);  // or seed from time for variability
    ClipSamplingWindow win;  // defaults: theta∈[0,120°], phi∈[0,60°]
    // Sample g in clip frame
    auto [grasping_direction_clip, quat_clip] = sample_g_in_clip(rng_, win, props.get<int>("clip_sign"));
    Eigen::Quaterniond quat_grasp_ee = (quat_clip * props.get<Eigen::Quaterniond>("quat_clip2ee")).normalized();
    // Eigen::Vector3d grasping_direction_clip = {0, 1, 0};

    // 2) Place master/follower TCPs in clip frame (keep distances fixed & collinear)
    double d_f = props.get<double>("follow_grasp_offset"); //
    double d_m = d_f + props.get<double>("track_offset"); // d_m = d_f + d_track
    Eigen::Isometry3d grasp_offset = Eigen::Isometry3d::Identity();
    grasp_offset.translation().z() = 0.03; // top of fixture base
    lead_grasp_tcp_pose_clip_   = tcp_at_clip_origin_plus(grasp_offset, grasping_direction_clip, quat_grasp_ee, d_m, props.get<std::string>("grasp_frame"));
    follow_grasp_tcp_pose_clip_ = tcp_at_clip_origin_plus(grasp_offset, grasping_direction_clip, quat_grasp_ee, d_f, props.get<std::string>("grasp_frame"));

    // 3) Transform to world for rviz visualization
    follow_grasp_tcp_pose_world_ = transformPoseWithScene(to_scene, follow_grasp_tcp_pose_clip_, "world");
    lead_grasp_tcp_pose_world_ = transformPoseWithScene(to_scene, lead_grasp_tcp_pose_clip_, "world");
    {
      const auto &oq = lead_grasp_tcp_pose_world_.pose.orientation; // geometry_msgs::msg::Quaternion
      // geometry_msgs stores x,y,z,w -> Eigen::Quaterniond(w,x,y,z)
      lead_grasp_orientation_ = Eigen::Quaterniond(oq.w, oq.x, oq.y, oq.z);
      lead_grasp_orientation_.normalize();

      // follower = leader * (follower→leader)^{-1}  (right-multiply the inverse)
      follow_grasp_orientation_ = lead_grasp_orientation_ * transport_rotation_.inverse();
      follow_grasp_orientation_.normalize();
    }
    // Clip origin in world (the goal frame origin)
    geometry_msgs::msg::PoseStamped grasp_origin_clip;
    grasp_origin_clip.header.frame_id = props.get<std::string>("grasp_frame");
    grasp_origin_clip.pose.position.z = 0.03; //clip_size[2]/2+hold_z_offset
    grasp_origin_clip.pose.orientation.w = 1.0; // identity
    grasp_origin_clip_world_ = transformPoseWithScene(to_scene, grasp_origin_clip, "world");

    leader_role.grasp_tcp_pose_clip = lead_grasp_tcp_pose_clip_;
    leader_role.grasp_tcp_pose_world = lead_grasp_tcp_pose_world_;
    leader_role.grasp_orientation = lead_grasp_orientation_;
    follower_role.grasp_tcp_pose_clip = follow_grasp_tcp_pose_clip_;
    follower_role.grasp_tcp_pose_world = follow_grasp_tcp_pose_world_;
    follower_role.grasp_orientation = follow_grasp_orientation_;

    /**********************************************Primary and Secondary ******************************************** */

    planning_scene::PlanningScenePtr primary_intermediate_scene = from_scene->diff();
    planning_scene::PlanningScenePtr primary_final_scene = from_scene->diff();
    robot_trajectory::RobotTrajectoryPtr reversed_secondary_trajectory;
    robot_trajectory::RobotTrajectoryPtr reversed_primary_trajectory;

    bool primary_is_leader = props.get<bool>("primary_is_leader");
    primary_role   = primary_is_leader ? leader_role   : follower_role;
    secondary_role = primary_is_leader ? follower_role : leader_role;

    /** Step 1: Compute trajectory for the primary arm */
    std::string primary_arm_plan_msg = "";
    if (!computePrimaryArmTrajectoryReverse(from, to, reversed_primary_trajectory, 
                                          primary_intermediate_scene, primary_final_scene,
                                          primary_arm_plan_msg)) 
    {   
        SubTrajectoryPtr failed_solution;
        if (reversed_primary_trajectory){ // failed after a partial solution
          failed_solution = std::make_shared<SubTrajectory>(reversed_primary_trajectory);  
        }else{
          failed_solution = std::make_shared<SubTrajectory>();
        }

        std::ostringstream oss;
        oss << "Grasp Direction: " << std::fixed << std::setprecision(3)
            << grasping_direction_clip.transpose()
            << "  Primary arm trajectory planning failed. " << primary_arm_plan_msg;
        failed_solution->markAsFailure(oss.str());
        connect(from, to, failed_solution);

        if (!ns.empty()) {
          visualizeGraspsForClipInputs(failed_solution->markers(), 
                                    /*success=*/false, 
                                      ns,
                                    /*leader=*/lead_grasp_tcp_pose_world_,
                                    /*follower=*/follow_grasp_tcp_pose_world_,
                                    /*origin=*/grasp_origin_clip_world_,
                                    /*d_m=*/d_m, 
                                    /*d_f=*/d_f,
                                    /*axis_len=*/0.15);
        }
        return;
    }

    // intermediate_scenes.push_back(primary_intermediate_scene);
    // intermediate_scenes.push_back(primary_final_scene);

    RCLCPP_INFO_STREAM(LOGGER, "Reversed Primary arm trajectory computed with " << reversed_primary_trajectory->getWayPointCount() << " waypoints");
    RCLCPP_INFO_STREAM(LOGGER, "Reversed Primary arm trajectory duration: " << reversed_primary_trajectory->getDuration());
    // intermediate_scenes.push_back(intermediate_scene->diff());

    /** Step 2: Compute trajectory for the secondary arm based on the primary arm's Cartesian trajectory from fixing to grasping*/
    RCLCPP_INFO_STREAM(LOGGER, "Intermediate scene update");
    std::vector<PlannerIdTrajectoryPair> reversed_secondary_trajectories;
    std::vector<PlannerIdTrajectoryPair> reversed_primary_trajectories;
    // if (!computePrimaryArmTrajectory(secondary_trajectory, secondary_trajectories ,to, primary_trajectory, primary_trajectories, leader_intermediate_scene, 
    //                               leader_final_scene, intermediate_scenes)) 
    std::string secondary_arm_plan_msg = "";
    planning_scene::PlanningScenePtr dual_grasp_scene = from_scene->diff();
    if (!computeSecondaryArmFixingToGraspReverse(reversed_primary_trajectory, reversed_primary_trajectories, 
                                            from_scene, reversed_secondary_trajectory, reversed_secondary_trajectories, 
                                            primary_intermediate_scene, primary_final_scene, intermediate_scenes, dual_grasp_scene, secondary_arm_plan_msg)) 
    {
        auto failed_solution = std::make_shared<SubTrajectory>(reversed_primary_trajectory);
        std::ostringstream oss;
        oss << "Grasp Direction: " << std::fixed << std::setprecision(3)
            << grasping_direction_clip.transpose()
            << "Secondary arm trajectory planning from fixing to grasping failed.  " << secondary_arm_plan_msg;
        failed_solution->markAsFailure(oss.str());        
        connect(from, to, failed_solution);
        if (!ns.empty()) {
          visualizeGraspsForClipInputs(failed_solution->markers(), 
                                    /*success=*/false, 
                                      ns,
                                    /*leader=*/lead_grasp_tcp_pose_world_,
                                    /*follower=*/follow_grasp_tcp_pose_world_,
                                    /*origin=*/grasp_origin_clip_world_,
                                    /*d_m=*/d_m, 
                                    /*d_f=*/d_f,
                                    /*axis_len=*/0.15);
        }
        return;
    }

    /* Assign primary and secondary trajectories back to leader and follower trajectory*/
    if (primary_is_leader) {
        reversed_leader_trajectory = reversed_primary_trajectory;
        reversed_follower_trajectory = reversed_secondary_trajectory;
        reversed_leader_trajectories = reversed_primary_trajectories;
        reversed_follower_trajectories = reversed_secondary_trajectories;
    } else {
        reversed_leader_trajectory = reversed_secondary_trajectory;
        reversed_follower_trajectory = reversed_primary_trajectory;
        reversed_leader_trajectories = reversed_secondary_trajectories;
        reversed_follower_trajectories = reversed_primary_trajectories;
    }


    /**********************************************Primary and Secondary ******************************************** */

    /*Step 3 Leader and Follower plan from grasp to current (from_scene) */
    std::string dual_arm_plan_msg = "";
    if (!computeDualGraspToCurrentTrajectoryReverse(reversed_follower_trajectory, reversed_follower_hand_trajectory, reversed_follower_trajectories, from_scene, reversed_leader_trajectory, reversed_leader_trajectories, 
                                                    intermediate_scenes, dual_grasp_scene, dual_arm_plan_msg)) 
    {
        auto failed_solution = std::make_shared<SubTrajectory>(reversed_primary_trajectory);
        std::ostringstream oss;
        oss << "Grasp Direction: " << std::fixed << std::setprecision(3)
            << grasping_direction_clip.transpose()
            << "Dual arm trajectory planning from grasping to current failed." << dual_arm_plan_msg;
        failed_solution->markAsFailure(oss.str());        
        connect(from, to, failed_solution);
        if (!ns.empty()) {
          visualizeGraspsForClipInputs(failed_solution->markers(), 
                                    /*success=*/false, 
                                      ns,
                                    /*leader=*/lead_grasp_tcp_pose_world_,
                                    /*follower=*/follow_grasp_tcp_pose_world_,
                                    /*origin=*/grasp_origin_clip_world_,
                                    /*d_m=*/d_m, 
                                    /*d_f=*/d_f,
                                    /*axis_len=*/0.15);
        }
        return;
    }

    // Step 3: Reverse the trajectories and connect them sequentially
    std::vector<PlannerIdTrajectoryPair> leader_trajectories;
    std::vector<PlannerIdTrajectoryPair> follower_trajectories;
    // Reverse the trajectory vectors
    for (int i = static_cast<int>(reversed_leader_trajectories.size()) - 1; i >= 0; --i) {
    // for (int i = 0; i >= 0; --i) {
        RCLCPP_INFO_STREAM(LOGGER, "Reversed subtrajectory " << i);
        auto reversed_leader_trajectory_pair = reversed_leader_trajectories[i];
        auto& reversed_leader_trajectory = reversed_leader_trajectory_pair.trajectory;
        auto leader_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(*reversed_leader_trajectory);
        leader_trajectory->reverse();
        leader_trajectories.push_back({reversed_leader_trajectory_pair.planner_id, leader_trajectory});

        auto reversed_follower_trajectory_pair = reversed_follower_trajectories[i];
        auto& reversed_follower_trajectory = reversed_follower_trajectory_pair.trajectory;
        auto follower_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(*reversed_follower_trajectory);
        follower_trajectory->reverse();
        follower_trajectories.push_back({reversed_follower_trajectory_pair.planner_id, follower_trajectory});
    }

    // Reverse the trajectory
    auto leader_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(*reversed_leader_trajectory);
    leader_trajectory->reverse();
    auto follower_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(*reversed_follower_trajectory);
    follower_trajectory->reverse();
    auto follower_hand_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(follower_trajectory->getRobotModel(), follower_role.hand_jmg);
    if (follower_grasp_index_ >= 0) {
      follower_hand_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(*reversed_follower_hand_trajectory);
      follower_hand_trajectory->reverse();

      follower_grasp_index_ = intermediate_scenes.size() - follower_grasp_index_ - 1; // Adjust index to match the reversed order
    } else {
      RCLCPP_WARN_STREAM(LOGGER, "Follower hand trajectory not set, skipping empty trajectory");
    }
    
    // Reverse the order of scenes in intermediate_scenes
    std::reverse(intermediate_scenes.begin(), intermediate_scenes.end());

    // Combine trajectories into a valid dual-arm solution
    RCLCPP_INFO_STREAM(LOGGER, "Combining leader and follower arm trajectories");
    RCLCPP_INFO_STREAM(LOGGER, "Leader arm trajectory computed with " << leader_trajectory->getWayPointCount() << " waypoints");
    RCLCPP_INFO_STREAM(LOGGER, "Follower arm trajectory computed with " << follower_trajectory->getWayPointCount() << " waypoints");

    // Merge each subtrajectory sequentially
    std::vector<PlannerIdTrajectoryPair> dual_sub_trajectories;
    std::string dual_merge_msg = "";
    for (int i = 0; i < leader_trajectories.size(); ++i) {
      if (i==follower_grasp_index_) {
        // Add follower hand trajectory at the grasp index
        dual_sub_trajectories.push_back({"follower_hand", follower_hand_trajectory});
        RCLCPP_INFO_STREAM(LOGGER, "Follower hand trajectory added at index " << i);
      }

        std::vector<PlannerIdTrajectoryPair> sub_trajectories;
        sub_trajectories.push_back({"leader_arm", leader_trajectories[i].trajectory});
        sub_trajectories.push_back({"follower_arm", follower_trajectories[i].trajectory});

        SubTrajectoryPtr dual_sub_trajectory = mergeIgnoreCollision(sub_trajectories, intermediate_scenes[i], from.scene()->getCurrentState());
        if (!dual_sub_trajectory) {
          dual_merge_msg = "Failed to merge sub trajectories of phase " + std::to_string(i);
          RCLCPP_ERROR_STREAM(LOGGER, "Failed to merge sub trajectories of phase " << i);
          continue;
        }
        if (i == leader_trajectories.size() - 1 && !ns.empty()) {
          // put “solution summary” markers on the last leaf
          visualizeGraspsForClipInputs(dual_sub_trajectory->markers(),
                                      /*success=*/true, ns,
                                      lead_grasp_tcp_pose_world_,
                                      follow_grasp_tcp_pose_world_,
                                      grasp_origin_clip_world_,
                                      d_m, d_f, 0.15);
        }

        dual_sub_trajectories.push_back({"dual_arm", dual_sub_trajectory->trajectory()});
        RCLCPP_INFO_STREAM(LOGGER, "Merge trajectory of phase " << i << " with " << dual_sub_trajectory->trajectory()->getWayPointCount() << " waypoints");
    }

    SolutionBasePtr solution;
    solution = makeSequential(dual_sub_trajectories, intermediate_scenes, from, to);
    if (dual_merge_msg != "") {
        std::ostringstream oss;
        oss << "Grasp Direction: " << std::fixed << std::setprecision(3)
            << grasping_direction_clip.transpose()
            << dual_merge_msg;
      solution->markAsFailure(oss.str());
      RCLCPP_ERROR_STREAM(LOGGER, dual_merge_msg);
    }

    // set msgs from both arms as comment
    std::ostringstream oss;
    solution->setComment(primary_arm_plan_msg + " | " + secondary_arm_plan_msg);

    // SolutionBasePtr solution;
    // // solution = std::make_shared<SubTrajectory>(reversed_primary_trajectory, 0.0, "connect_master_follower");
    // solution = makeSequential(reversed_primary_trajectories, second_arm_intermediate_scenes, to, from);
    
    // if (!ns.empty()) {
    //   visualizeGraspsForClipInputs(solution->markers(), 
    //                             /*success=*/true, 
    //                               ns,
    //                             /*leader=*/lead_grasp_tcp_pose_world_,
    //                             /*follower=*/follow_grasp_tcp_pose_world_,
    //                             /*origin=*/grasp_origin_clip_world_,
    //                             /*d_m=*/d_m, 
    //                             /*d_f=*/d_f,
    //                             /*axis_len=*/0.15);
    // }
    connect(from, to, solution);
    return;
}

bool ConnectMFReverse::ExtractPrimaryArmCartesianTrajectory(const robot_trajectory::RobotTrajectoryPtr& primary_trajectory,
                                                   const moveit::core::RobotState& final_goal_state,
                                                   std::vector<geometry_msgs::msg::Pose>& primary_tip_path,
                                                   std::vector<double>& path_time,
                                                  //  int& start_index,
                                                   double start_offset,
                                                //    std::vector<PlannerIdTrajectoryPair>& primary_trajectories,
                                                //    robot_trajectory::RobotTrajectoryPtr& follower_grasp_trajectory,
                                                   robot_trajectory::RobotTrajectoryPtr& primary_track_trajectory,
                                                   bool reverse)
{  
  // std::vector<geometry_msgs::msg::Pose> primary_tip_path;
  geometry_msgs::msg::Pose primary_start_hand_pose_msg;
  Eigen::Isometry3d primary_start_tip_pose;
 
  int start_index = primary_start_index_;

  bool track_start = false;

  // bool start = false;
  for (size_t i = 0; i < primary_trajectory->getWayPointCount(); ++i) {
    const auto& point = primary_trajectory->getWayPoint(i);
    double delta_time = (i == 0) ? 0.0 : primary_trajectory->getWayPointDurationFromStart(i) - primary_trajectory->getWayPointDurationFromStart(i - 1);    

    if (!final_goal_state.knowsFrameTransform(primary_role.hand_link)) {
        RCLCPP_ERROR(LOGGER, "Link '%s' not found in model '%s'", primary_role.hand_link.c_str(), final_goal_state.getRobotModel()->getName().c_str());
        return false;
    }
    Eigen::Isometry3d ee_link_pose = point.getGlobalLinkTransform(primary_role.hand_link); // hand pose
    // Apply the transform to get the pose of the actual end-effector
    Eigen::Isometry3d tip_pose = ee_link_pose * primary_role.hand_to_tcp_transform;
    if (start_index== -1){
        // find follower start index based on the distance
        // follower start pose
        if (i == 0) {
            tf2::convert(ee_link_pose, primary_start_hand_pose_msg);
            primary_start_tip_pose = tip_pose;
            continue;
        }
        
        if (i > 0){
            // check distance to the primary start point to decide the leader start point
            double distance_from_start  = (tip_pose.translation() - primary_start_tip_pose.translation()).norm();
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
                //                     primary_trajectory->getWayPointDurationFromStart(i), primary_trajectory->getWayPointDurationFromStart(i-1), delta_time);
        
                primary_track_trajectory->addSuffixWayPoint(point, delta_time);
            
                // Convert to geometry_msgs::Pose
                geometry_msgs::msg::Pose pose;
                tf2::convert(tip_pose, pose);
                primary_tip_path.push_back(pose);
        
                // log the time for each point
                path_time.push_back(primary_trajectory->getWayPointDurationFromStart(i));
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
                    RCLCPP_INFO_STREAM(LOGGER, "leader starts tracking from waypoint index: " << start_index << "until " << primary_trajectory->getWayPointCount());
                }
                primary_track_trajectory->addSuffixWayPoint(point, delta_time);
                // Convert to geometry_msgs::Pose
                geometry_msgs::msg::Pose pose;
                tf2::convert(tip_pose, pose);
                primary_tip_path.push_back(pose);
        
                // log the time for each point
                path_time.push_back(primary_trajectory->getWayPointDurationFromStart(i));
            }
        }
    } 
    
  }

  if (primary_tip_path.empty()) {
    RCLCPP_ERROR(LOGGER, "primary arm trajectory is empty.");
    return false;
  }

  return true;
}

bool ConnectMFReverse::computeSecondaryArmFixingToGraspReverse(robot_trajectory::RobotTrajectoryPtr& primary_trajectory,
                                                  std::vector<PlannerIdTrajectoryPair>& primary_trajectories,
                                                  planning_scene::PlanningSceneConstPtr& goal_scene,
                                                  robot_trajectory::RobotTrajectoryPtr& secondary_trajectory,
                                                  std::vector<PlannerIdTrajectoryPair>& secondary_trajectories,
                                                  planning_scene::PlanningScenePtr& primary_intermediate_scene,
                                                  planning_scene::PlanningScenePtr& primary_final_scene,
                                                  std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes,
                                                  planning_scene::PlanningScenePtr& phase_1_scene,     // <-- NEW: output grasp scene
                                                  std::string& return_message)
{

  const auto& props = properties();
  const moveit::core::RobotState& initial_state = primary_final_scene->getCurrentState();
  Eigen::Isometry3d secondary_initial_pose_tcp = initial_state.getGlobalLinkTransform(secondary_role.hand_link) * secondary_role.hand_to_tcp_transform;
  Eigen::Quaterniond secondary_initial_orientation(initial_state.getGlobalLinkTransform(secondary_role.hand_link).rotation());
  // secondary_role.arm_jmg = initial_state.getJointModelGroup(props.get<std::string>("lead_group"));
  return_message = "";

  const moveit::core::RobotState& final_goal_state = goal_scene->getCurrentState();
  intermediate_scenes.push_back(goal_scene);

  // double start_offset = 0.15;
  double track_offset = props.get<double>("track_offset");
  // double grasp_follower_offset = props.get<double>("follow_grasp_offset");
  // double grasp_leader_offset = track_offset + grasp_follower_offset; // not used if leader_start_index_ is already set

  RCLCPP_INFO_STREAM(LOGGER, "Computing trajectory for the fisrt arm");

  /* Extract the Cartesian trajectory of the first arm */ 
  std::vector<geometry_msgs::msg::Pose> primary_tip_path;
  std::vector<double> path_time_sequnce;
  // int follower_start_index = 0;

  // reverse the follower trajectory
//   auto reversed_primary_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(*primary_trajectory);
//   reversed_primary_trajectory->reverse();
  // std::cout << "Reversed follower trajectory:" << std::endl;
  // reversed_primary_trajectory->print(std::cout);
  // primary_trajectory = reversed_primary_trajectory;

//   double follower_duration_original = reversed_primary_trajectory->getDuration();
//   RCLCPP_INFO_STREAM(LOGGER, "follower arm trajectory duration: " << follower_duration_original);

//   auto reversed_primary_track_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(reversed_primary_trajectory->getRobotModel(), primary_role.arm_jmg);
  auto primary_track_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(primary_trajectory->getRobotModel(), primary_role.arm_jmg);
//   auto primary_to_goal_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(primary_trajectory->getRobotModel(), primary_role.arm_jmg);
  if (!ExtractPrimaryArmCartesianTrajectory(primary_trajectory, final_goal_state, primary_tip_path, path_time_sequnce, 
                                          primary_role.grasp_offset, primary_track_trajectory, true)) {
    RCLCPP_INFO_STREAM(LOGGER, "Failed to extract primary arm Cartesian trajectory.");
    return false;
  }

  RCLCPP_INFO_STREAM(LOGGER, "Primary arm trajectory with " << primary_trajectory->getWayPointCount() << " waypoints, " 
                              << primary_track_trajectory->getWayPointCount() << " for tracking");
                            //   << primary_to_goal_trajectory->getWayPointCount() << " for goal");
//   for (size_t i = 0; i < primary_tip_path.size(); ++i) {
//     const auto& pose_msg = primary_tip_path[i];
//     Eigen::Isometry3d original_pose;
//     tf2::fromMsg(pose_msg, original_pose);
//     std::cout << "follower tip position: " << original_pose.translation().transpose() << std::endl;
//     // std::cout << "leader tip orientation: " << Eigen::Quaterniond(original_pose.rotation()).coeffs().transpose() << std::endl;
//   }


  /* Obtain the Cartesian path for the leader from the follower's Cartesian path */
  std::vector<geometry_msgs::msg::Pose> secondary_tip_path;
  //   std::vector<geometry_msgs::msg::Pose> leader_hand_path;

  Eigen::Isometry3d secondary_hand_grasp_pose;
  Eigen::Isometry3d secondary_tip_grasp_pose;
  Eigen::Isometry3d secondary_start_pose_tcp = secondary_initial_pose_tcp;
  Eigen::Quaterniond secondary_start_orientation(secondary_initial_orientation);
  secondary_start_orientation.normalize();
  RCLCPP_INFO_STREAM(LOGGER, "Secondary arm start orientation: " << secondary_start_orientation.coeffs().transpose());

  // set secondary final orientation
  // geometry_msgs::msg::PoseStamped secondary_grasp_pose_msg = props.get<geometry_msgs::msg::PoseStamped>("lead_grasp_pose");
  geometry_msgs::msg::PoseStamped secondary_grasp_pose_msg = secondary_role.grasp_tcp_pose_world;
  Eigen::Isometry3d secondary_grasp_pose_tcp;
  tf2::fromMsg(secondary_grasp_pose_msg.pose, secondary_grasp_pose_tcp);
  Eigen::Quaterniond grasp_orientation(secondary_grasp_pose_tcp.rotation());
  RCLCPP_INFO_STREAM(LOGGER, "Secondary arm grasp orientation: " << grasp_orientation.coeffs().transpose());
  Eigen::Vector3d secondary_grasp_target_position = secondary_grasp_pose_tcp.translation();
  // // grasp's yaw and start's pitch and roll
  // Eigen::Quaterniond combined_orientation = combineRotations(grasp_orientation, secondary_start_orientation);
  // Eigen::Quaterniond leader_grasp_orientation = combined_orientation;

//   Eigen::Quaterniond leader_grasp_orientation(secondary_start_orientation);
  RCLCPP_INFO_STREAM(LOGGER, "Secondary arm final orientation: " << secondary_role.grasp_orientation.coeffs().transpose());

  Eigen::Isometry3d primary_start_pose;
  tf2::fromMsg(primary_tip_path.front(), primary_start_pose);
  Eigen::Isometry3d relative_start_transform = primary_start_pose.inverse() * secondary_start_pose_tcp;

  Eigen::Isometry3d primary_grasp_pose;
  tf2::fromMsg(primary_tip_path.back(), primary_grasp_pose);
  Eigen::Isometry3d relative_grasp_transform = primary_grasp_pose.inverse() * secondary_grasp_pose_tcp;

  // check if the relative transform is constant between start and grasp
  Eigen::Isometry3d relative_transform_change = primary_start_pose.inverse() * primary_grasp_pose;
  double position_diff = (relative_transform_change.translation() - relative_start_transform.translation()).norm();
  Eigen::Quaterniond relative_start_rotation(Eigen::Quaterniond(relative_start_transform.rotation()));
  Eigen::Quaterniond relative_grasp_rotation(Eigen::Quaterniond(relative_grasp_transform.rotation()));
  relative_start_rotation.normalize();
  relative_grasp_rotation.normalize();
  Eigen::Quaterniond relative_rotation_diff = relative_start_rotation.conjugate() * relative_grasp_rotation;
  double angle_diff = std::acos(relative_rotation_diff.w()) * 2.0 * 180.0 / M_PI; // in degrees 
  RCLCPP_WARN_STREAM(LOGGER, "Difference of relative transform from start to grasp: position " << position_diff << " m, rotation " << angle_diff << " degrees");

  int length = primary_tip_path.size();
  for (size_t i = 0; i < length; ++i) {
    double percentage = (double)i / (double)length;

    const auto& pose_msg = primary_tip_path[i];
    Eigen::Isometry3d primary_tip_pose;
    tf2::fromMsg(pose_msg, primary_tip_pose);

    // // preserve only yaw rotation (around Z axis) of the follower
    // Eigen::Matrix3d original_rot = primary_tip_pose.rotation();
    // double follower_yaw = std::atan2(original_rot(1, 0), original_rot(0, 0));  // equivalent to yaw from rotation matrix
    // Eigen::AngleAxisd yaw_rotation(follower_yaw, Eigen::Vector3d::UnitZ());
    // Eigen::Quaterniond yaw_quat(yaw_rotation);
    // Eigen::Matrix3d yaw_rot_matrix = yaw_quat.toRotationMatrix();

    // // Define the offset in the ee frame
    // Eigen::Vector3d offset_ee(track_offset, 0.0, 0.0);  // offset along the X-axis of the follower's EEF frame
    // // Transform the offset to the world frame
    // Eigen::Vector3d offset_in_world = yaw_rot_matrix * offset_ee;
    // Eigen::Isometry3d secondary_tip_pose;
    // secondary_tip_pose.translation() = primary_tip_pose.translation() + offset_in_world;

    // secondary_start_orientation.normalize();
    // leader_grasp_orientation.normalize();

    // // compute the leader orientation based on the relative rotation at the starting point
    // Eigen::Quaterniond follower_orientation = Eigen::Quaterniond(primary_tip_pose.rotation());
    // Eigen::Quaterniond leader_orientation = relative_start_rotation * follower_orientation;
    // // leader_orientation.normalize();
    // secondary_tip_pose.linear() = leader_orientation.toRotationMatrix();

    // // interpolate the orientation
    // // Eigen::Quaterniond interpolated_orientation = secondary_start_orientation.slerp(percentage, leader_grasp_orientation);
    // // // interpolated_orientation.normalize();
    // // secondary_tip_pose.linear() = interpolated_orientation.toRotationMatrix();

    // // pose at hand
    // // Eigen::Isometry3d lead_hand_pose = secondary_tip_pose*lead_hand_frame_transform;

    Eigen::Isometry3d secondary_tip_pose = primary_tip_pose * relative_start_transform;                        // full rigid composition
    // Eigen::Isometry3d secondary_tip_pose = transport_transform_ * primary_tip_pose;  // using the constant relative transform between leader and follower TCPs

    geometry_msgs::msg::Pose secondary_tip_pose_msg;
    tf2::convert(secondary_tip_pose, secondary_tip_pose_msg);
    secondary_tip_path.push_back(secondary_tip_pose_msg);

    // geometry_msgs::msg::Pose lead_hand_pose_msg;
    // tf2::convert(lead_hand_pose, lead_hand_pose_msg);
    // leader_hand_path.push_back(lead_hand_pose_msg);
  }

  // check the distance between the last point and the target grasp point
  const auto &p = secondary_tip_path.back().position; // geometry_msgs::msg::Point
  Eigen::Vector3d final_secondary_tip_position(p.x, p.y, p.z);
  Eigen::Vector3d path_position_difference = final_secondary_tip_position - secondary_grasp_target_position;
  // double orientation_difference = leader_tip_orientation.angularDistance(target_orientation);
  RCLCPP_INFO_STREAM(LOGGER, "Position difference between the target pose and the pose on path at grasping for the secondary arm: " << path_position_difference.norm());
  return_message += "secondary path grasp offset: " + std::to_string(path_position_difference(0)) + ", " + std::to_string(path_position_difference(1)) + ", " + std::to_string(path_position_difference(2));

//   Test the last orientation of cartesian wapyoints
//   geometry_msgs::msg::Pose last_pose_hand = leader_hand_path.back();
//   tf2::fromMsg(last_pose_hand, secondary_hand_grasp_pose);
  geometry_msgs::msg::Pose last_pose_tcp = secondary_tip_path.back();
  tf2::fromMsg(last_pose_tcp, secondary_tip_grasp_pose);

  Eigen::Vector3d grasp_hand_position = secondary_hand_grasp_pose.translation();
  Eigen::Quaterniond grasp_hand_orientation(secondary_hand_grasp_pose.rotation());
  grasp_hand_orientation.normalize();
  RCLCPP_INFO_STREAM(LOGGER, "secondary arm first step should end at hand position: " << grasp_hand_position.transpose());
  RCLCPP_INFO_STREAM(LOGGER, "secondary arm first step should end at hand orientation: " << grasp_hand_orientation.coeffs().transpose());

  // Test if position secondary_hand_grasp_pose == leader_hand_path[0]
  Eigen::Vector3d grasp_tip_position = secondary_tip_grasp_pose.translation();
  Eigen::Quaterniond grasp_tip_orientation(secondary_tip_grasp_pose.rotation());
  RCLCPP_INFO_STREAM(LOGGER, "secondary arm first step should end at tcp position: " << grasp_tip_position.transpose());
  RCLCPP_INFO_STREAM(LOGGER, "secondary arm first step should end at tcp orientation: " << grasp_tip_orientation.coeffs().transpose());

//   visual_tools_.publishPath(secondary_tip_path, rviz_visual_tools::YELLOW, rviz_visual_tools::MEDIUM);
//   visual_tools_.trigger();

//   check lead_tip_path
  // for (size_t i = 0; i < secondary_tip_path.size(); ++i) {
  //   const auto& pose_msg = secondary_tip_path[i];
  //   Eigen::Isometry3d original_pose;
  //   tf2::fromMsg(pose_msg, original_pose);
  //   Eigen::Quaterniond original_orientation(original_pose.rotation());
  //   std::cout << "leader tip position: " << original_pose.translation().transpose() << " orientation: " << original_orientation.coeffs().transpose() << std::endl;
  // }

  /************************************************************************************************************/
  /*** Step 1: BACKWARD Planning: move secondary arm starting from "to" to track primary until grasping point ***/
  /***********************************************************************************************************/
  robot_trajectory::RobotTrajectoryPtr secondary_track_trajectory;

  planning_scene::PlanningSceneConstPtr start = primary_intermediate_scene;
  planning_scene::PlanningScenePtr intermediate_scene = start->diff();
//   planning_scene::PlanningScenePtr intermediate_scene = primary_intermediate_scene->diff();
// //   update the scene with the follower arm's grasp state
//   const moveit::core::RobotState& follower_grasp_state = primary_track_trajectory->getLastWayPoint();
// //   const moveit::core::RobotState& follower_grasp_state = primary_intermediate_scene->getCurrentState();
//   std::vector<double> follower_joint_positions;
//   follower_grasp_state.copyJointGroupPositions(primary_role.arm_jmg, follower_joint_positions);

//   moveit::core::RobotState& state = intermediate_scene->getCurrentStateNonConst();
//   state.setJointGroupPositions(primary_role.arm_jmg, follower_joint_positions);
//   state.update();  //

  double fraction_secondary = SecondaryArmFollow(intermediate_scene, secondary_tip_path, secondary_track_trajectory);

  if (fraction_secondary < 1.0) {
    return_message = "secondary arm failed to track the primary's trajectory. Fraction: " + std::to_string(fraction_secondary);
    RCLCPP_ERROR(LOGGER, "%s", return_message.c_str());
    return false;
  }else{
    RCLCPP_INFO(LOGGER, "secondary arm successfully tracked the primary's cartesian path with %zu waypoints", secondary_track_trajectory->getWayPointCount());

    std::cout << "secondary track trajectory:" << std::endl;
    secondary_track_trajectory->print(std::cout);

    // // // Update intermediate scene
    // const moveit::core::RobotState& leader_final_state = secondary_track_trajectory->getLastWayPoint();
    // std::vector<double> leader_joint_positions;
    // leader_final_state.copyJointGroupPositions(secondary_role.arm_jmg, leader_joint_positions);

    // moveit::core::RobotState& state = intermediate_scene->getCurrentStateNonConst();
    // state.setJointGroupPositions(secondary_role.arm_jmg, leader_joint_positions);
    // state.update(true);  // Ensure consistency

    // RCLCPP_INFO_STREAM(LOGGER, "leader arm state updated in intermediate_scene.");
  }

  // set tip_link to get cumulative arc length
  if (!secondary_track_trajectory->setTipLink(secondary_role.base_link, secondary_role.flange_to_tcp_transform, 0.00)) {
    return_message = "Failed to set tip link for secondary trajectory.";
    RCLCPP_ERROR(LOGGER, "%s", return_message.c_str());
    return false;
  }

  if (!primary_track_trajectory->setTipLink(primary_role.base_link, primary_role.flange_to_tcp_transform, 0.00)) {
    return_message = "Failed to set tip link for primary trajectory.";
    RCLCPP_ERROR(LOGGER, "%s", return_message.c_str());
    return false;
  }

  double primary_total_distance = primary_track_trajectory->getWayPointArcDistanceFromStart(primary_track_trajectory->getWayPointCount());
  std::cout << "primary track trajectory total distance: " << primary_total_distance << std::endl;

  double secondary_total_distance = secondary_track_trajectory->getWayPointArcDistanceFromStart(secondary_track_trajectory->getWayPointCount());
  std::cout << "secondary track trajectory total distance: " << secondary_total_distance << std::endl;

  /*******************************************resample the secondary trajectory to match the secondary_tip_path*****************************************/
  robot_trajectory::RobotTrajectoryPtr secondary_track_resample_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(secondary_track_trajectory->getRobotModel(), secondary_role.arm_jmg);
  Eigen::Isometry3d robot_base_pose = secondary_track_trajectory->getWayPoint(0).getGlobalLinkTransform(secondary_role.base_link);

  size_t j_start = 0;
  double last_progress = 0.0;
  for (size_t i = 0; i < secondary_tip_path.size(); ++i) {
    const auto& pose_msg_in_world = secondary_tip_path[i];
    Eigen::Isometry3d original_pose;
    tf2::fromMsg(pose_msg_in_world, original_pose);
    Eigen::Isometry3d desired_secondary_tip_pose_in_base = robot_base_pose.inverse() * original_pose;

    // Eigen::Isometry3d desired_secondary_tip_pose_in_base = original_pose * Eigen::Isometry3d::Identity(); // desired is the original pose
    RCLCPP_INFO_STREAM(LOGGER, "Desired secondary tip pose at index " << i << ": " << desired_secondary_tip_pose_in_base.translation().transpose());

    double position_error_tolerance = 0.001; // 1cm tolerance
    std::vector<std::pair<size_t, Eigen::Isometry3d>> best_position_candidates;
    best_position_candidates.clear();
    double best_position_error = std::numeric_limits<double>::max();
    size_t best_j = j_start;

    for (size_t j = j_start; j < secondary_track_trajectory->getWayPointCount(); ++j) {
      Eigen::Isometry3d secondary_tip_pose_in_base = secondary_track_trajectory->getWayPointPose(j);
      Eigen::Vector3d secondary_tip_position_in_base = secondary_track_trajectory->getWayPointPosition(j);

      // check position alignment
      double position_err = (secondary_tip_position_in_base - desired_secondary_tip_pose_in_base.translation()).norm();
      RCLCPP_INFO_STREAM(LOGGER, "Position at index " << j << " is " << secondary_tip_position_in_base.transpose() << " with position error: " << position_err);
      if (position_err < best_position_error) {
        best_position_error = position_err;
      }
      if (position_err < position_error_tolerance) {
        best_position_candidates.push_back({j, secondary_tip_pose_in_base});
      }
    }

    if (best_position_candidates.size() == 0) {
      RCLCPP_WARN_STREAM(LOGGER, "No position candidates found for secondary tip at index " << i << ". Start interpolation from index " << j_start);
      // interpolation in Cartesian space
      auto secondary_interpolated_state = std::make_shared<moveit::core::RobotState>(secondary_track_trajectory->getRobotModel());
      int before = 0, after = 0;
      double blend = 1.0;
      bool interpolated = secondary_track_trajectory->getStateAtPosition(desired_secondary_tip_pose_in_base.translation(), secondary_interpolated_state, j_start, before, after, blend);

      if (!interpolated) {
        return_message = "Failed to interpolate secondary state at index " + std::to_string(i);
        RCLCPP_ERROR(LOGGER, "%s", return_message.c_str());
        // save trajectories locally whether match is found or not
        dumpTrajectoryTXTIndexed(*primary_track_trajectory,  "primary_no_matching",  primary_role.group, "MTC_connect_visualization", primary_role.flange_to_tcp_transform, primary_role.base_link); // saves ./failed_left_001.txt, etc.
        dumpTrajectoryTXTIndexed(*secondary_track_trajectory, "secondary_no_matching", secondary_role.group, "MTC_connect_visualization", secondary_role.flange_to_tcp_transform, secondary_role.base_link); // saves ./failed_right_001.txt, etc.
        dumpPathTxTIndexed(secondary_tip_path, "secondary_tcp_path_no_matching");
        return false;
      }
      if (blend < 1){
        j_start = before;
      }else{
        j_start = after;
      }
      secondary_track_resample_trajectory->addSuffixWayPoint(secondary_interpolated_state, 0.1);
    }else{
      // check orientation alignment
      double orientation_error_tolerance = 1e-4;
      std::vector<std::pair<size_t, Eigen::Isometry3d>> best_orientation_candidates;
      best_orientation_candidates.clear();
      double best_orientation_error = std::numeric_limits<double>::max();
      size_t best_orientation_j = j_start;
      for (auto & candidate : best_position_candidates) {
        Eigen::Isometry3d candidate_transform = candidate.second;
        const Eigen::Matrix3d original_rotation = original_pose.linear();              // or original_pose.rotation()
        const Eigen::Matrix3d candidate_rotation = candidate_transform.linear();        // or candidate_transform.rotation()

        // Ignore spin about A's local y-axis (matches your Python ignore_axis='y')
        double orientation_err = computeRotationDistanceIgnoreAxisChar(original_rotation, candidate_rotation, 'None');
        
        if (orientation_err < best_orientation_error) {
          best_orientation_error = orientation_err;
          best_orientation_candidates.clear(); // reset the best candidates
          best_orientation_candidates.push_back(candidate);
          best_orientation_j = candidate.first;
        }
        if (orientation_err - best_orientation_error < orientation_error_tolerance) {
          best_orientation_candidates.push_back(candidate);
        }
      }

      if (best_orientation_candidates.size() <= 1) {
        best_j = best_orientation_j; // use the best j from the orientation candidates
        RCLCPP_INFO_STREAM(LOGGER, "No orientation alignment candidates found. Using best j " << best_j
                                              <<" for waypoint i" << i
                                              << " with position error: " << best_position_error
                                              << " and orientation error: " << best_orientation_error);
      }else{
        size_t minimum_j = best_orientation_candidates[0].first;
        for (auto & candidate : best_orientation_candidates) {
          size_t j_candidate = candidate.first;
          if (j_candidate > j_start && j_candidate < minimum_j) {
            minimum_j = j_candidate;
          }
        }
        best_j = minimum_j; // use the minimum j from the best candidates
        RCLCPP_INFO_STREAM(LOGGER, "Best alignment for waypoint " << i
                                                << " found at index " << best_j
                                              << " with position error: " << best_position_error
                                              << " and orientation error: " << best_orientation_error);
      }

      secondary_track_resample_trajectory->addSuffixWayPoint(secondary_track_trajectory->getWayPoint(best_j), 0.1);
      j_start = best_j; // update j_start to the best j found
    }

  }

  RCLCPP_INFO_STREAM(LOGGER, "Resampled secondary trajectory has " << secondary_track_resample_trajectory->getWayPointCount() << " waypoints.");

  // add the last point to ensure reaching the goal
  // if (secondary_track_resample_trajectory->getWayPointCount() > 1){
  //   const moveit::core::RobotState& last_point = secondary_track_trajectory->getLastWayPoint();
  //   const moveit::core::RobotState& second_last_point = secondary_track_trajectory->getWayPoint(secondary_track_trajectory->getWayPointCount()-2);
  //   secondary_track_resample_trajectory->addSuffixWayPoint(last_point, 0.1);
  // }

  // phase_1_scene: scene after tracking
  // planning_scene::PlanningScenePtr phase_1_scene; // TODO@KejiaChen: is phase_3_scene the same as intermediate_scene?
  planning_scene::PlanningScenePtr temp_start = start->diff();
  updateDualIntermediateState(secondary_track_resample_trajectory->getLastWayPoint(), secondary_role,
                              primary_track_trajectory->getLastWayPoint(), primary_role,
                              temp_start, phase_1_scene);

  secondary_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(*secondary_track_resample_trajectory);
  primary_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(*primary_track_trajectory);
  // Add them to the vector
  secondary_trajectories.push_back({"secondary_arm", secondary_track_resample_trajectory});
  primary_trajectories.push_back({"primary_arm", primary_track_trajectory});

  // intermediate scene is added in a reverse order
  intermediate_scenes.push_back(phase_1_scene);

  // check reached leader grasp position 
  const moveit::core::RobotState& intermediate_state = secondary_track_resample_trajectory->getLastWayPoint();
  Eigen::Isometry3d secondary_tip_transform = intermediate_state.getGlobalLinkTransform(secondary_role.hand_link) * secondary_role.hand_to_tcp_transform;
  Eigen::Vector3d secondary_tip_position = secondary_tip_transform.translation();
  // Eigen::Quaterniond leader_tip_orientation(secondary_tip_transform.rotation());
  // Eigen::Quaterniond target_orientation(target_pose.rotation());
  Eigen::Vector3d position_difference = secondary_tip_position - secondary_grasp_target_position;
  // double orientation_difference = leader_tip_orientation.angularDistance(target_orientation);
  RCLCPP_INFO_STREAM(LOGGER, "After resample, position difference between the target pose and the planned pose at grasping for the secondary arm: " << position_difference.norm());
  return_message += "secondary reached grasp offset: " + std::to_string(position_difference(0)) + ", " + std::to_string(position_difference(1)) + ", " + std::to_string(position_difference(2));

  // store successful trajectories
  dumpTrajectoryTXTIndexed(*primary_track_trajectory,  "primary_success",  primary_role.group, "MTC_connect_visualization", primary_role.flange_to_tcp_transform, primary_role.base_link); 
  // dumpTrajectoryTXTIndexed(*follower_track_resample_trajectory,  "follower_success_resample",  "left_panda_arm", "MTC_connect_visualization", follow_flange_to_tcp_transform_, props.get<std::string>("follow_base_link"));
  dumpTrajectoryTXTIndexed(*secondary_track_trajectory, "secondary_success", secondary_role.group, "MTC_connect_visualization", secondary_role.flange_to_tcp_transform, secondary_role.base_link);
  dumpTrajectoryTXTIndexed(*secondary_track_resample_trajectory, "secondary_success_resample", secondary_role.group, "MTC_connect_visualization",  secondary_role.flange_to_tcp_transform, secondary_role.base_link);

  // save secondary tip path
  dumpPathTxTIndexed(secondary_tip_path, "secondary_tcp_path_success"); // saves ./failed_right_path_001.txt, etc.

  return true;
}


bool ConnectMFReverse::computeDualGraspToCurrentTrajectoryReverse(robot_trajectory::RobotTrajectoryPtr& follower_trajectory,
                                                  robot_trajectory::RobotTrajectoryPtr& follower_hand_trajectory,
                                                  std::vector<PlannerIdTrajectoryPair>& follower_trajectories,
                                                  planning_scene::PlanningSceneConstPtr& goal_scene,
                                                  robot_trajectory::RobotTrajectoryPtr& leader_trajectory,
                                                  std::vector<PlannerIdTrajectoryPair>& leader_trajectories,
                                                  // robot_trajectory::RobotTrajectoryPtr& dual_trajectory,
                                                  std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes,
                                                  planning_scene::PlanningScenePtr& phase_1_scene,
                                                  std::string& return_message) 
{ 
  const auto& props = properties();
  const moveit::core::RobotState& final_goal_state = goal_scene->getCurrentState();

  /*******************************************/
  /*** Step 1.5: Grasping of follower arm ***/
  /******************************************/
  RCLCPP_INFO_STREAM(LOGGER, "Follower arm grasping");
  // Set the hand joint grasping target
  planning_scene::PlanningScenePtr pregrasp_scene = phase_1_scene->diff();
  moveit::core::RobotState& pregrasp_state = pregrasp_scene->getCurrentStateNonConst();
  std::vector<double>intermediate_hand_positions;
  final_goal_state.copyJointGroupPositions(follower_role.hand_jmg, intermediate_hand_positions);
  RCLCPP_INFO_STREAM(LOGGER, "Follower hand joint positions: " << intermediate_hand_positions[0]);
  pregrasp_state.setJointGroupPositions(follower_role.hand_jmg, intermediate_hand_positions);
  pregrasp_state.update();
  
  /*Planning for hand*/
  for (const GroupPlannerVector::value_type& pair : hand_planner_) {
    if (pair.first == follower_role.hand_group) {
      planning_scene::PlanningSceneConstPtr start = phase_1_scene;
      planning_scene::PlanningSceneConstPtr end = pregrasp_scene;

      // Plan trajectory for the hand
      auto result_hand = pair.second->plan(start, end, follower_role.hand_jmg, 
                                          props.get<double>("timeout"), follower_hand_trajectory);
      if (!result_hand) {
        return_message = "Follower hand planning to grasp pose failed.";
        RCLCPP_ERROR(LOGGER, "%s", return_message.c_str());
        return false;
      }
      RCLCPP_INFO_STREAM(LOGGER, "Follower hand trajectory planning success ");
    }
  }

  if (!follower_hand_trajectory->empty()) {
    planning_scene::PlanningScenePtr phase_1_grasp_scene;
    updateDualIntermediateState(leader_trajectory->getLastWayPoint(), leader_role,
                                follower_hand_trajectory->getLastWayPoint(), follower_role,
                                phase_1_scene, phase_1_grasp_scene);
    
    intermediate_scenes.push_back(phase_1_grasp_scene);

    follower_grasp_index_ = intermediate_scenes.size() - 1;
  } else {
    RCLCPP_WARN(LOGGER, "Hand trajectory is empty.");
  }


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
  planning_scene::PlanningScenePtr tension_scene = pregrasp_scene->diff();
  // update the scene with the follower arm's goal state
  std::vector<double> follower_joint_positions;
  final_goal_state.copyJointGroupPositions(follower_role.arm_jmg, follower_joint_positions);
  moveit::core::RobotState& state = tension_scene->getCurrentStateNonConst();
  state.setJointGroupPositions(follower_role.arm_jmg, follower_joint_positions);  
  state.update(true);

  // Build a temporary planning scene for follower planning
  planning_scene::PlanningScenePtr temp_scene = tension_scene->diff();

  // Names
  const std::string leader_hand = "right_panda_hand"; // or props.get<std::string>("lead_hand_link")
  const std::string tmp_object_id = "pull_cable_leader_fixture";

  if (props.get<bool>("attach_pull_cable")) {
    // Compute endpoints (in world)
    Eigen::Isometry3d T_world_fixture = temp_scene->getFrameTransform(props.get<std::string>("grasp_frame"));
    Eigen::Isometry3d T_world_leader_tcp = temp_scene->getFrameTransform(leader_hand) * leader_role.hand_to_tcp_transform;

    Eigen::Vector3d A = T_world_leader_tcp.translation();
    Eigen::Vector3d B = T_world_fixture.translation() + Eigen::Vector3d(0, 0, 0.03); // slightly above the grasp frame
    Eigen::Vector3d dir_world = (B - A);
    double full_len = dir_world.norm();

    // Optional margin so the cylinder doesn't poke into geometry
    const double radius = 0.015;         // 4 mm cable
    const double end_margin = 2.0 * radius; // trim at both ends
    double use_len = std::max(1e-3, full_len);
    Eigen::Vector3d dir_world_unit;
    if (full_len > 1e-9)
      dir_world_unit = dir_world / full_len;
    else
      dir_world_unit = Eigen::Vector3d::UnitX();

    // Attach: direction is from leader TCP toward fixture
    std::vector<std::string> leader_touch = {
      "right_panda_hand", "right_panda_leftfinger", "right_panda_rightfinger"
      // add more leader links if you want to be extra-safe, but touch_links is usually hand+fingers
    };
    
    attachCollisionCableGeneric(temp_scene,
                                tmp_object_id,
                                use_len,
                                radius,
                                dir_world_unit,                 // only direction matters inside helper
                                leader_hand,
                                leader_role.hand_to_tcp_transform,    // leader hand→TCP
                                leader_touch,
                                LOGGER);

    // Also allow collisions with the grasp fixture object (so cable doesn't collide with it)
    {
      auto& acm = temp_scene->getAllowedCollisionMatrixNonConst();
      acm.setEntry(tmp_object_id, props.get<std::string>("grasp_frame"), true);
    }
  }

  // Now plan follower with the temporary attached cable in the scene
  for (const auto& pair : planner_) {
    if (pair.first == follower_role.group) {
      // Start scene: same environment as temp_scene, but follower at pregrasp pose
      planning_scene::PlanningScenePtr start_scene = temp_scene->diff();
      std::vector<double> follower_start_positions;
      {
        // get follower start joints from pregrasp_scene (or current robot state)
        const auto& pregrasp_state = pregrasp_scene->getCurrentState();
        pregrasp_state.copyJointGroupPositions(follower_role.arm_jmg, follower_start_positions);
      }
      moveit::core::RobotState& start_state = start_scene->getCurrentStateNonConst();
      start_state.setJointGroupPositions(follower_role.arm_jmg, follower_start_positions);
      start_state.update(true);

      planning_scene::PlanningSceneConstPtr start = start_scene;
      planning_scene::PlanningSceneConstPtr end   = temp_scene;

      auto result = pair.second->plan(start, end, follower_role.arm_jmg,
                                      props.get<double>("timeout"),
                                      follower_to_goal_trajectory);
      
      if (props.get<bool>("attach_pull_cable")){
        // Immediately detach & remove the temporary cable (keeps scene clean even if you reuse temp_scene)
        detachCollisionCableWorldAndRobot(temp_scene, tmp_object_id);
      }

      if (!result) {
        return_message = "Follower arm planning to goal (with temp cable attached to leader) failed: " + result.message;
        RCLCPP_ERROR(LOGGER, "%s", return_message.c_str());
        return false;
      }

      RCLCPP_INFO_STREAM(LOGGER, "Follower plan succeeded with " << follower_to_goal_trajectory->getWayPointCount() << " waypoints (temp cable attached).");
    }
  }

  // Plan joint trajectory for the leader arm
  for (const auto& pair : planner_) {
    if (pair.first == leader_role.group) {
      planning_scene::PlanningSceneConstPtr end = goal_scene;
      planning_scene::PlanningSceneConstPtr start = tension_scene;

      // add line constraint
      std::string constraint_link = leader_role.hand_link;
    //   auto line_constraint = setLineConstraint(start, end, constraint_link);
      auto constraint = setBoxConstraint(start, end, constraint_link);
      
      // Plan trajectory
      auto result = pair.second->plan(start, end, leader_role.arm_jmg, props.get<double>("timeout"), leader_to_goal_trajectory, constraint);
      success = bool(result);

      if (!success) {
      return_message = "Leader arm planning to goal failed: " + result.message;
      RCLCPP_ERROR(LOGGER, return_message.c_str());
      return false;
      }

      RCLCPP_INFO_STREAM(LOGGER, "leader arm planning to goal succeeded with " << leader_to_goal_trajectory->getWayPointCount() << " waypoints.");
      //   return true;
    }
  }

  /* Time Adjustment */ 
  RCLCPP_INFO_STREAM(LOGGER, "Adding pause for the leader and follower arm trajectories");
  auto delayed_leader_to_goal_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(leader_to_goal_trajectory->getRobotModel(), leader_role.arm_jmg);
  // Get the time when finishing the first step
  double leader_to_goal_duration = leader_to_goal_trajectory->getWayPointDurationFromStart(leader_to_goal_trajectory->getWayPointCount());
  double follower_to_goal_duration = follower_to_goal_trajectory->getWayPointDurationFromStart(follower_to_goal_trajectory->getWayPointCount());

//   double follower_second_step_start_time = follower_to_goal_duration;
//   double follower_second_step_end_time = follower_duration_original;
  if (leader_to_goal_trajectory){
    // Force the leader_trajectory to start after the follower_trajectory finishes the first step
    if (!splitTrajectoryWithPause(leader_to_goal_trajectory, follower_to_goal_duration, 0, delayed_leader_to_goal_trajectory, leader_to_goal_trajectories, true)) {
        return_message = "Failed to delay the leader trajectory.";
        RCLCPP_ERROR(LOGGER, "%s", return_message.c_str());
        return false;
    }
    // leader_trajectory = delayed_leader_to_goal_trajectory;

    // Force the follower_trajectory to wait for the leader_trajectory to finish the first step
    // double pause_duration = leader_to_goal_duration - follower_trajectory->getWayPointDurationFromStart(follower_start_index);
    double pause_duration = leader_to_goal_duration;
    RCLCPP_INFO_STREAM(LOGGER, "Follower pause duration: " << pause_duration);
    auto follower_to_goal_trajectory_with_pause = std::make_shared<robot_trajectory::RobotTrajectory>(follower_trajectory->getRobotModel(), follower_role.arm_jmg);
    if (!splitTrajectoryWithPause(follower_to_goal_trajectory, leader_to_goal_duration, follower_to_goal_trajectory->getWayPointCount(), follower_to_goal_trajectory_with_pause, follower_to_goal_trajectories, false)) {
        return_message = "Failed to extend the follower trajectory.";
        RCLCPP_ERROR(LOGGER, return_message.c_str());
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
  updateDualIntermediateState(leader_to_goal_trajectories[0]->getLastWayPoint(), leader_role,
                               follower_to_goal_trajectories[0]->getLastWayPoint(), follower_role,
                               phase_1_scene, phase_2_scene);
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

void ConnectMFReverse::updateDualIntermediateState(const moveit::core::RobotState& first_state,
                                            ArmRole& first_role,
                                            const moveit::core::RobotState& second_state,
                                            ArmRole& second_role,
                                            planning_scene::PlanningScenePtr& start,
                                            planning_scene::PlanningScenePtr& end) 
{
  end = start->diff();
  moveit::core::RobotState& dual_state = end->getCurrentStateNonConst();

  // Update the arm state
  std::vector<double> first_joint_positions;
  first_state.copyJointGroupPositions(first_role.arm_jmg, first_joint_positions);
  dual_state.setJointGroupPositions(first_role.arm_jmg, first_joint_positions);
  dual_state.update();  // Ensure consistency
  std::vector<double> second_joint_positions;
  second_state.copyJointGroupPositions(second_role.arm_jmg, second_joint_positions);
  dual_state.setJointGroupPositions(second_role.arm_jmg, second_joint_positions);

  // Update the hand state
  std::vector<double> first_hand_positions;
  first_state.copyJointGroupPositions(first_role.hand_jmg, first_hand_positions);
  dual_state.setJointGroupPositions(first_role.hand_jmg, first_hand_positions);
  std::vector<double> second_hand_positions;
  second_state.copyJointGroupPositions(second_role.hand_jmg, second_hand_positions);
  dual_state.setJointGroupPositions(second_role.hand_jmg, second_hand_positions);

  dual_state.update();  // Ensure consistency
}

bool ConnectMFReverse::computePrimaryArmTrajectoryReverse(const InterfaceState& from, const InterfaceState& to,
                                                        robot_trajectory::RobotTrajectoryPtr& primary_trajectory,
                                                        planning_scene::PlanningScenePtr& intermediate_scene,
                                                        planning_scene::PlanningScenePtr& final_scene,
                                                        std::string& return_message) 
{
  const auto& props = properties();
  const moveit::core::RobotState& start_state = to.scene()->getCurrentState();
  const moveit::core::RobotState& final_state = from.scene()->getCurrentState();

  RCLCPP_INFO(LOGGER, "Computing trajectory for the follower arm.");

  double track_offset = props.get<double>("track_offset");
  // double grasp_follower_offset = props.get<double>("follow_grasp_offset");
//   double grasp_leader_offset = track_offset + grasp_follower_offset;

  /* Option 2: Passing intermediate waypoint*/
  planning_scene::PlanningScenePtr start = to.scene()->diff();

  Eigen::Isometry3d secondary_hand_transform = start_state.getGlobalLinkTransform(secondary_role.hand_link) * secondary_role.hand_to_tcp_transform;
  Eigen::Isometry3d primary_hand_transform = start_state.getGlobalLinkTransform(primary_role.hand_link) * primary_role.hand_to_tcp_transform;
  Eigen::Vector3d cable_vector_in_world = (secondary_hand_transform.translation() - primary_hand_transform.translation()).normalized();  

  // ---- Step 1: Plan from goal to grasping ----
  const moveit::core::LinkModel* eef_link = start_state.getLinkModel(primary_role.hand_link);
  // Offset from current link frame to ik_frame (used by solver)
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity(); // offset is identity

  robot_trajectory::RobotTrajectoryPtr traj_1;
  robot_trajectory::RobotTrajectoryPtr traj_cartesian;
  Eigen::Isometry3d target_pose;
  Eigen::Isometry3d primary_grasp_pose_tcp;
  for (const auto& pair: cartesian_planner_) {
  // for (const auto& pair: planner_) {
    if (pair.first == primary_role.group) {

      // follow_jmg_ = final_state.getJointModelGroup(pair.first);

      const moveit::core::RobotState& start_state = start->getCurrentState();

      // Get current pose of EEF in world frame
      Eigen::Isometry3d current_pose = start_state.getGlobalLinkTransform(eef_link);
      Eigen::Quaterniond clip_orientation(current_pose.rotation());
      RCLCPP_INFO_STREAM(LOGGER, "Primary arm current position: " << current_pose.translation().transpose());
      RCLCPP_INFO_STREAM(LOGGER, "Primary arm current orientation: " << clip_orientation.coeffs().transpose());

      // Option 1: Get the primary_grasp_pose in world frame as target
      // geometry_msgs::msg::PoseStamped follower_grasp_pos_msg = props.get<geometry_msgs::msg::PoseStamped>("follow_grasp_pose");
      geometry_msgs::msg::PoseStamped primary_grasp_pos_msg =  primary_role.grasp_tcp_pose_world;
      tf2::fromMsg(primary_grasp_pos_msg.pose, primary_grasp_pose_tcp);
      Eigen::Quaterniond grasp_orientation(primary_grasp_pose_tcp.rotation());
      
      // Eigen::Quaterniond combined_orientation = combineRotations(grasp_orientation, clip_orientation);
      // primary_grasp_pose_tcp.linear() = combined_orientation.toRotationMatrix();
      // follower should have the same rotation to leader at grasping and at at the start
      primary_grasp_pose_tcp.linear() = primary_role.grasp_orientation.toRotationMatrix();
      Eigen::Isometry3d primary_grasp_pose_eef = primary_grasp_pose_tcp * (primary_role.hand_to_tcp_transform.inverse());

      target_pose = primary_grasp_pose_eef;
      Eigen::Quaterniond target_orientation(target_pose.rotation());
      
      // // Option 2: Get the primary_grasp_pose in EEF frame as target
      // // Apply local +X offset
      // Eigen::Isometry3d target_pose = current_pose * Eigen::Translation3d(grasp_follower_offset, 0.0, 0.0);

      // TODO@KejiaChen: check if the target pose is valid
      
      RCLCPP_INFO_STREAM(LOGGER, "Primary arm grasp position: " << target_pose.translation().transpose());
      RCLCPP_INFO_STREAM(LOGGER, "Primary arm grasp orientation: " << target_orientation.coeffs().transpose());

      // auto result_cartesian = pair.second->plan(start, *eef_link, offset, target_pose, primary_role.arm_jmg,
      //                                 props.get<double>("timeout"), traj_cartesian, path_constraints);

      // if (!result_cartesian) {
      //   return_message = "Follower arm planning to grasp pose with cartesian planner failed.";
      //   RCLCPP_ERROR(LOGGER, "%s", return_message.c_str());
      //   return false;
      // }
      
    }
  }

  primary_trajectory.reset(new robot_trajectory::RobotTrajectory(start->getRobotModel(), primary_role.arm_jmg));

  // // initialize follow_hand_jmg
  // for (const GroupPlannerVector::value_type& pair : hand_planner_) {
  //   if (pair.first == primary_role.hand_group) {
  //     follow_hand_jmg_ = final_state.getJointModelGroup(pair.first);
  //     RCLCPP_INFO_STREAM(LOGGER, "Follower hand group name: " << follow_hand_jmg_->getName());
  //   }
  //   if (pair.first == secondary_role.hand_group) {
  //     leader_hand_jmg_ = final_state.getJointModelGroup(pair.first);
  //     RCLCPP_INFO_STREAM(LOGGER, "Leader hand group name: " << leader_hand_jmg_->getName());
  //   }
  // }
  // get finger joint positions of start and final state
  std::vector<double> start_finger_positions;
  start_state.copyJointGroupPositions(primary_role.hand_jmg, start_finger_positions);
  RCLCPP_INFO_STREAM(LOGGER, "Primary hand start joint positions: " << start_finger_positions[0]);
  std::vector<double> final_finger_positions;
  final_state.copyJointGroupPositions(primary_role.hand_jmg, final_finger_positions);
  RCLCPP_INFO_STREAM(LOGGER, "Primary hand final joint positions: " << final_finger_positions[0]);

  // if (!props.get<bool>("attach_transport_cable")) {
  //   // cartesian planner
  //   // auto result_cartesian = pair.second->plan(start, *eef_link, offset, target_pose, primary_role.arm_jmg,
  //   //                                      props.get<double>("timeout"), traj_cartesian, path_constraints);

  //   // if (!result_cartesian) {
  //   //   return_message = "Follower arm planning to grasp pose with cartesian planner failed.";
  //   //   RCLCPP_ERROR(LOGGER, "%s", return_message.c_str());
  //   //   return false;
  //   // }

  //   // traj_1 = traj_cartesian;
  //   RCLCPP_INFO_STREAM(LOGGER, "Cable collision not considered.");
  // }else
  {
    /*Planning target for arm*/
    planning_scene::PlanningScenePtr start_with_cable = start->diff();
    std::string object_id = "grasped_cable";
    // cable should be initially aligned with the vector pointing from the leader hand to the follower hand
    attachCollisionCable(start_with_cable, object_id,  track_offset, 0.01,  cable_vector_in_world, primary_role.hand_link, 
                        // allow collision links
                        {primary_role.hand_link, "left_panda_leftfinger", "left_panda_rightfinger", secondary_role.hand_link, "right_panda_leftfinger", "right_panda_rightfinger"},
                        props.get<bool>("attach_transport_cable"));
    // intermediate_scenes.push_back(start_with_cable);
    
    // cartesian planner is only to obtain grasp_scene
    planning_scene::PlanningScenePtr grasp_with_cable = start_with_cable->diff();
    moveit::core::RobotState& grasp_state = grasp_with_cable->getCurrentStateNonConst();

    // Option 1: Set the arm joint target from the cartesian trajectory
    // const moveit::core::RobotState& traj1_final_state = traj_cartesian->getLastWayPoint();
    // std::vector<double> intermediate_arm_positions;
    // traj1_final_state.copyJointGroupPositions(primary_role.arm_jmg, intermediate_arm_positions);
    // grasp_state.setJointGroupPositions(primary_role.arm_jmg, intermediate_arm_positions);
    // grasp_state.update();

    // Option 2: Set the arm joint target from IK
    // Define your seed (bias) joint configuration
    // std::vector<double> seed = {0.0, -0.8, 0.0, -2.2, 0.0, 1.5, 0.5};
    // state.setJointGroupPositions(primary_role.arm_jmg, seed);
    bool success = grasp_state.setFromIK(primary_role.arm_jmg, target_pose, primary_role.hand_link, props.get<double>("timeout"));
    if (!success) {
      return_message = "Follower arm grasp pose set from IK failed.";
      RCLCPP_ERROR(LOGGER, "%s", return_message.c_str());
      return false;
    }
    std::vector<double> intermediate_arm_positions;
    grasp_state.copyJointGroupPositions(primary_role.arm_jmg, intermediate_arm_positions);
    RCLCPP_INFO_STREAM(LOGGER, "Follower arm grasp joint position: " << intermediate_arm_positions[0] << " " << intermediate_arm_positions[1] << " " << intermediate_arm_positions[2] << " "
                                            << intermediate_arm_positions[3] << " " << intermediate_arm_positions[4] << " " << intermediate_arm_positions[5] << " "
                                            << intermediate_arm_positions[6]);
    grasp_state.update();
    
    /*Planning for arm*/
    // ompl planner plans to the grasp scene
    robot_trajectory::RobotTrajectoryPtr traj_ompl_1;
    for (const auto& pair: planner_) {
      if (pair.first == primary_role.group) {
        // Plan trajectory

        // add orientation constraint
        // std::string constraint_link = "left_panda_hand";
        auto constraint = setOrientationConstraint(start_state, primary_role.hand_link);

        auto result_1 = pair.second->plan(start_with_cable, grasp_with_cable, primary_role.arm_jmg, 
                                        props.get<double>("timeout"), traj_ompl_1); // constraint); // follow_path_constraints
        // auto result_1 = pair.second->plan(start_with_cable, *eef_link, offset, target_pose, primary_role.arm_jmg,
        //                                 props.get<double>("timeout"), traj_ompl_1);
        if (!result_1) {
          return_message = "Follower arm planning to grasp pose with ompl failed.";
          RCLCPP_ERROR(LOGGER, "%s", return_message.c_str());
          return false;
        }
      }
    }

    // chomp planner plans to the grasp scene
    for (const auto& pair: chomp_planner_) {
      if (pair.first == primary_role.group) {
        // initialize the trajectory
        
        moveit_msgs::msg::RobotTrajectory robot_ref_traj_msg;
        traj_ompl_1->getRobotTrajectoryMsg(robot_ref_traj_msg);
        moveit_msgs::msg::GenericTrajectory generic_ref_traj_msg;
        generic_ref_traj_msg.joint_trajectory.resize(1);
        generic_ref_traj_msg.joint_trajectory[0] = robot_ref_traj_msg.joint_trajectory;
        RCLCPP_INFO_STREAM(LOGGER, "Set cartesian trajectory as initial trajectory.");
        // Plan trajectory
        auto result_1 = pair.second->plan(start_with_cable, grasp_with_cable, primary_role.arm_jmg, props.get<double>("timeout"), traj_1, generic_ref_traj_msg, primary_role.path_constraints);
        if (!result_1) {
          return_message = "Follower arm planning to grasp pose with chomp failed.";
          RCLCPP_ERROR(LOGGER, "%s", return_message.c_str());
          return false;
        }
      }
    }

    if (!traj_1->empty()) {
      intermediate_scene = start->diff();
      moveit::core::RobotState& intermediate_state = intermediate_scene->getCurrentStateNonConst();
      // update arm state
      const moveit::core::RobotState& traj1_final_state = traj_1->getLastWayPoint();
      std::vector<double> intermediate_arm_positions;
      traj1_final_state.copyJointGroupPositions(primary_role.arm_jmg, intermediate_arm_positions);
      intermediate_state.setJointGroupPositions(primary_role.arm_jmg, intermediate_arm_positions);
      intermediate_state.update();

      primary_trajectory->append(*traj_1, 0.1);
      primary_start_index_ = primary_trajectory->getWayPointCount();
      RCLCPP_INFO_STREAM(LOGGER, "primary arm trajectory start index: " << primary_start_index_);

      // Compare the reached primary pose with the target pose
      Eigen::Isometry3d primary_tip_transform = intermediate_state.getGlobalLinkTransform(primary_role.hand_link) * primary_role.hand_to_tcp_transform;
      Eigen::Vector3d primary_tip_position = primary_tip_transform.translation();
      // Eigen::Quaterniond follower_tip_orientation(primary_tip_transform.rotation());
      Eigen::Vector3d goal_position = primary_grasp_pose_tcp.translation();
      // Eigen::Quaterniond target_orientation(target_pose.rotation());
      Eigen::Vector3d position_difference = primary_tip_position - goal_position;
      // double orientation_difference = follower_tip_orientation.angularDistance(target_orientation);
      RCLCPP_INFO_STREAM(LOGGER, "Position difference between the target pose and the planned pose at grasping for the primary arm: " << position_difference.norm());
      return_message = "primary grasp position offset: " + std::to_string(position_difference(0)) + ", " + std::to_string(position_difference(1)) + ", " + std::to_string(position_difference(2));

      // compare rotation and show difference in angle
      Eigen::Matrix3d rotation_difference = primary_tip_transform.rotation().transpose() * primary_grasp_pose_tcp.rotation();
      Eigen::AngleAxisd rotation_difference_aa(rotation_difference);
      double rotation_difference_angle = rotation_difference_aa.angle();
      RCLCPP_INFO_STREAM(LOGGER, "Rotation difference (angle) between the target pose and the planned pose at grasping for the primary arm: " << rotation_difference_angle);
      return_message = return_message + ", rotation difference (angle): " + std::to_string(rotation_difference_angle);
    } else {
      RCLCPP_WARN(LOGGER, "Arm trajectory is empty.");
    }

    // Detach the cable from the primary arm
    detachCollisionCable(start_with_cable, object_id);

  }
  
  // ---- Step 2: Plan from grasping to start ----
  robot_trajectory::RobotTrajectoryPtr traj_2;
  robot_trajectory::RobotTrajectoryPtr traj_ompl_2(new robot_trajectory::RobotTrajectory(start->getRobotModel(), primary_role.arm_jmg));
  // update the intermediate scene with the primary hand's goal state
  planning_scene::PlanningScenePtr pregrasp_scene = intermediate_scene->diff();
  moveit::core::RobotState& pregrasp_state = pregrasp_scene->getCurrentStateNonConst();
  std::vector<double>intermediate_hand_positions;
  final_state.copyJointGroupPositions(primary_role.hand_jmg, intermediate_hand_positions);
  pregrasp_state.setJointGroupPositions(primary_role.hand_jmg, intermediate_hand_positions);
  pregrasp_state.update();

  for (const auto& pair : planner_) {
    if (pair.first == primary_role.group) {
      final_scene = start->diff();
      moveit::core::RobotState& goal_state = final_scene->getCurrentStateNonConst();
      std::vector<double> goal_arm_positions;
      final_state.copyJointGroupPositions(primary_role.arm_jmg, goal_arm_positions);
      goal_state.setJointGroupPositions(primary_role.arm_jmg, goal_arm_positions);
      goal_state.update();

      // Plan trajectory
      auto result_2 = pair.second->plan(pregrasp_scene, final_scene, primary_role.arm_jmg, props.get<double>("timeout"),
                                      traj_ompl_2);
      if (!result_2) {
        return_message = "Primary arm planning from grasp pose to final with ompl failed.";
        RCLCPP_ERROR(LOGGER, "%s", return_message.c_str());
        return false;
      }
    }
  }

  for (const auto& pair: chomp_planner_) {
    if (pair.first == primary_role.group) {
      // initialize the trajectory
      moveit_msgs::msg::RobotTrajectory robot_ref_traj_msg;
      traj_ompl_2->getRobotTrajectoryMsg(robot_ref_traj_msg);
      moveit_msgs::msg::GenericTrajectory generic_ref_traj_msg;
      generic_ref_traj_msg.joint_trajectory.resize(1);
      generic_ref_traj_msg.joint_trajectory[0] = robot_ref_traj_msg.joint_trajectory;
      RCLCPP_INFO_STREAM(LOGGER, "Set cartesian trajectory as initial trajectory.");
      // Plan trajectory
      auto result_2 = pair.second->plan(intermediate_scene, final_scene, primary_role.arm_jmg, props.get<double>("timeout"), traj_2, generic_ref_traj_msg);
      if (!result_2) {
        return_message = "Primary arm planning from grasp pose to final with chomp failed.";
        RCLCPP_ERROR(LOGGER, "%s", return_message.c_str());
        return false;
      }
    }
  }

  // ---- Step 4: Concatenate and assign ----
  primary_trajectory->append(*traj_2, 0.0);

  return true;

  // return false;
}

double ConnectMFReverse::SecondaryArmFollow(planning_scene::PlanningScenePtr& intermediate_scene,
                                std::vector<geometry_msgs::msg::Pose> secondary_tip_path,
                                robot_trajectory::RobotTrajectoryPtr& secondary_trajectory) {

  const auto& props = properties();
  move_group_lead_->setPoseReferenceFrame("world");
  move_group_lead_->setStartState(intermediate_scene->getCurrentState());

  auto secondary_scene = intermediate_scene->diff();

  // Define the transform from the "leader_ee_link" to the actual end-effector frame
//   Eigen::Matrix3d leader_flange_to_ee_rotation;
//   leader_flange_to_ee_rotation << 0.7071, 0.7071, 0,
//                             -0.7071, 0.7071, 0,
//                             0, 0, 1;
//   Eigen::Isometry3d lead_flange_to_tcp_transform = hand_to_tcp_transform_;
//   // TODO@KejiaChen: AUTO CORRECT
//   lead_flange_to_tcp_transform.translation().z() = lead_flange_to_tcp_transform.translation().z() + 0.036;
//   lead_flange_to_tcp_transform.linear() = leader_flange_to_ee_rotation*hand_to_tcp_transform_.linear();
  Eigen::Isometry3d secondary_flange_to_tcp_transform = secondary_role.flange_to_tcp_transform;
  RCLCPP_INFO_STREAM(LOGGER, "Secondary arm flange to tcp translation: " << secondary_flange_to_tcp_transform.translation().transpose() << 
                             " orientation: " << Eigen::Quaterniond(secondary_flange_to_tcp_transform.rotation()).coeffs().transpose());

  // compute joint trajectory from the cartesian path
  moveit_msgs::msg::RobotTrajectory secondary_trajectory_msg;

  std::cout<<"secondary arm tip path has " << secondary_tip_path.size() << " waypoints." << std::endl;

  // TODO@KejiaChen: set time_parameterization to false will lead to problems
  double fraction_secondary = move_group_lead_->computeCartesianPath(secondary_tip_path, 0.001, 0.0, secondary_trajectory_msg, true,
                                                                    nullptr, secondary_flange_to_tcp_transform, true);
  // leader_cartesian_planner_.plan(secondary_scene, secondary_role.arm_jmg->getLinkModel(primary_role.hand_link),
  //                                lead_grasp_frame_transform, 
  // leader_hand_path, 0.01, 0.0, secondary_trajectory_msg);

  secondary_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(
      secondary_scene->getRobotModel(), secondary_scene->getRobotModel()->getJointModelGroup(secondary_role.group));
  secondary_trajectory->setRobotTrajectoryMsg(secondary_scene->getCurrentState(), secondary_trajectory_msg);
  
  RCLCPP_INFO_STREAM(LOGGER, "Secondary arm planning finishesd with " << fraction_secondary * 100.0 << "% success.");
  RCLCPP_INFO_STREAM(LOGGER, "Secondary arm trajectory has " << secondary_trajectory->getWayPointCount() << " waypoints.");

  return fraction_secondary;
}

SubTrajectoryPtr ConnectMFReverse::mergeIgnoreCollision(const std::vector<PlannerIdTrajectoryPair>& sub_trajectories,
                                  const planning_scene::PlanningSceneConstPtr& intermediate_scene,
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

	// check merged trajectory for collisions
	if (!intermediate_scene->isPathValid(*trajectory, properties().get<moveit_msgs::msg::Constraints>("path_constraints"))){
		RCLCPP_INFO(LOGGER, "Collision detected in merged trajectory");
		return SubTrajectoryPtr();
	}

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
  
  // TODO@KejiaChen: check if the orientation is aligned

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

/*utils functions*/
Eigen::Quaterniond ConnectMFReverse::combineRotations(Eigen::Quaterniond grasp_orientation, 
                                                      Eigen::Quaterniond fixing_orientation)
{
    /* Combine the yaw rotaton from grasping pose and other rotations from fixing pose*/

    // Extract yaw angle from grasping pose
    Eigen::Matrix3d grasp_rot = grasp_orientation.toRotationMatrix();
    double grasp_yaw = std::atan2(grasp_rot(1, 0), grasp_rot(0, 0));  // equivalent to yaw from rotation matrix
    // Build a pure yaw rotation around world Z
    Eigen::AngleAxisd yaw_rotation(grasp_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond grasp_yaw_quat(yaw_rotation);

    // Remove yaw from fixing pose by rotating back around world Z
    Eigen::Matrix3d fixing_rot = fixing_orientation.toRotationMatrix();
    double fixing_yaw = std::atan2(fixing_rot(1,0), fixing_rot(0,0));
    Eigen::AngleAxisd fixing_yaw_inv(-fixing_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond fixing_rot_without_yaw(fixing_yaw_inv * fixing_rot);

    // Now apply follower's yaw
    Eigen::Quaterniond combined_orientation = grasp_yaw_quat * fixing_rot_without_yaw;

    return combined_orientation;
}

moveit_msgs::msg::Constraints ConnectMFReverse::setLineConstraint(planning_scene::PlanningSceneConstPtr start,
                                                                planning_scene::PlanningSceneConstPtr end,
                                                                std::string constraint_link_name)
{
  // Define a straight-line path constraint
  moveit_msgs::msg::Constraints path_constraints;
  moveit_msgs::msg::PositionConstraint position_constraint;

  position_constraint.header.frame_id = "world";  // Reference frame for the constraint
  position_constraint.link_name = constraint_link_name;  // End-effector link

  // Define the constraint region as a cylinder along the straight line
  shape_msgs::msg::SolidPrimitive constraint_region;
  constraint_region.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  constraint_region.dimensions = {0.1, 0.02};  // Height (length of the line) and radius

  // Compute the midpoint of the straight line
  Eigen::Vector3d start_position = start->getCurrentState().getGlobalLinkTransform(constraint_link_name).translation();
  Eigen::Vector3d goal_position = end->getCurrentState().getGlobalLinkTransform(constraint_link_name).translation();
  Eigen::Vector3d midpoint = (start_position + goal_position) / 2.0;

  // Set the pose of the cylinder
  geometry_msgs::msg::Pose constraint_pose;
  constraint_pose.position.x = midpoint.x();
  constraint_pose.position.y = midpoint.y();
  constraint_pose.position.z = midpoint.z();

  // Compute the orientation of the cylinder to align with the straight line
  Eigen::Vector3d direction = (goal_position - start_position).normalized();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), direction);
  constraint_pose.orientation.x = orientation.x();
  constraint_pose.orientation.y = orientation.y();
  constraint_pose.orientation.z = orientation.z();
  constraint_pose.orientation.w = orientation.w();

  position_constraint.constraint_region.primitives.push_back(constraint_region);
  position_constraint.constraint_region.primitive_poses.push_back(constraint_pose);
  position_constraint.weight = 1.0;  // Full weight for this constraint

  path_constraints.position_constraints.push_back(position_constraint);

  visual_tools_.publishCylinder(start_position, goal_position, rviz_visual_tools::TRANSLUCENT_DARK, rviz_visual_tools::SMALL, "line_constraint");
  visual_tools_.trigger();

  return path_constraints;
}


moveit_msgs::msg::Constraints ConnectMFReverse::setBoxConstraint(planning_scene::PlanningSceneConstPtr start,
                                                                    planning_scene::PlanningSceneConstPtr end,
                                                                    std::string constraint_link_name)
{
  Eigen::Isometry3d current_pose = start->getCurrentState().getGlobalLinkTransform(constraint_link_name);
  Eigen::Isometry3d goal_pose = end->getCurrentState().getGlobalLinkTransform(constraint_link_name);

  // Compute the box center and dimensions
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = (current_pose.translation().x() + goal_pose.translation().x()) / 2.0;
  box_pose.position.y = (current_pose.translation().y() + goal_pose.translation().y()) / 2.0;
  box_pose.position.z = (current_pose.translation().z() + goal_pose.translation().z()) / 2.0;
  box_pose.orientation.w = 1.0; // Identity quaternion for box orientation

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {
      fabs(goal_pose.translation().x() - current_pose.translation().x())+0.05,  // Length (x)
      fabs(goal_pose.translation().y() - current_pose.translation().y())+0.05,  // Width (y)
      fabs(goal_pose.translation().z() - current_pose.translation().z())+0.05   // Height (z)
    };

  // Create position constraint
  moveit_msgs::msg::PositionConstraint box_constraint;
  box_constraint.header.frame_id = "world"; // Replace with the appropriate reference frame
  box_constraint.link_name = constraint_link_name; // Replace with the relevant link name
  box_constraint.constraint_region.primitives.emplace_back(box);
  box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
  box_constraint.weight = 1.0;

  // Visualize the box constraint
  Eigen::Vector3d box_point_1(
      box_pose.position.x - box.dimensions[0] / 2.0,
      box_pose.position.y - box.dimensions[1] / 2.0,
      box_pose.position.z - box.dimensions[2] / 2.0
  );
  Eigen::Vector3d box_point_2(
      box_pose.position.x + box.dimensions[0] / 2.0,
      box_pose.position.y + box.dimensions[1] / 2.0,
      box_pose.position.z + box.dimensions[2] / 2.0
  );
//   visual_tools_.publishCuboid(box_point_1, box_point_2, rviz_visual_tools::TRANSLUCENT_DARK);
//   visual_tools_.trigger();

  // Wrap in a generic Constraints message
  moveit_msgs::msg::Constraints box_constraints;
  box_constraints.position_constraints.emplace_back(box_constraint);

  return box_constraints;
}

// moveit_msgs::msg::TrajectoryConstraints ConnectMFReverse::createTrajectoryConstraintsFromTrajectory(const moveit_msgs::msg::RobotTrajectory& robot_traj_msg)
// {
//   moveit_msgs::msg::TrajectoryConstraints trajectory_constraints;
//   trajectory_constraints.constraints.reserve(robot_traj_msg.joint_trajectory.points.size());

//   for (const auto& point : robot_traj_msg.joint_trajectory.points) {
//     moveit_msgs::msg::Constraints waypoint_constraints;

//     for (size_t i = 0; i < robot_traj_msg.joint_trajectory.joint_names.size(); ++i) {
//       moveit_msgs::msg::JointConstraint jc;
//       jc.joint_name = robot_traj_msg.joint_trajectory.joint_names[i];
//       jc.position = point.positions[i];
//       jc.tolerance_above = 1e-5;
//       jc.tolerance_below = 1e-5;
//       jc.weight = 1.0;
//       waypoint_constraints.joint_constraints.push_back(jc);
//     }

//     trajectory_constraints.constraints.push_back(waypoint_constraints);
//   }

//   return trajectory_constraints;
// }

void ConnectMFReverse::attachCollisionCable(planning_scene::PlanningScenePtr scene,
                                             const std::string& id, 
                                             double length,
                                             double radius,
                                             Eigen::Vector3d vec_in_world,
                                             const std::string& attach_link, 
                                             std::vector<std::string> touch_links,
                                             bool enable_cable_collision)
{
    moveit_msgs::msg::AttachedCollisionObject attach_msg;
    attach_msg.link_name = attach_link;
    attach_msg.object.header.frame_id = attach_link;
    attach_msg.object.id = id;

    // Add geometry of cable
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.CYLINDER;
    prim.dimensions = {length, radius}; // height (along local Z), radius

    // Step 1: Create pose in TCP frame (cylinder lying along +X, end at origin)
    Eigen::Isometry3d cylinder_pose_tcp = Eigen::Isometry3d::Identity();
    // Convert direction from world frame into attach_link tcp frame
    Eigen::Isometry3d world_to_hand = scene->getFrameTransform(attach_link).inverse();
    Eigen::Vector3d vec_in_hand = world_to_hand.linear() * vec_in_world.normalized();
    // Calculate the rotation of cylinder Z-axis to align with the direction
    Eigen::Quaterniond align_quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), vec_in_hand);
    cylinder_pose_tcp.linear() = align_quat.toRotationMatrix();
    // Rotate cylinder Z-axis → X-axis using +90° about Y
    // cylinder_pose_tcp.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    // Translate it so one end sits at TCP
    cylinder_pose_tcp.translation() =  vec_in_hand.normalized() * (0.5*length);
    // Step 2: Transform to `left_panda_hand` frame
    Eigen::Isometry3d cylinder_pose_in_hand = primary_role.hand_to_tcp_transform * cylinder_pose_tcp;
    // Step 3: Convert to geometry_msgs::Pose
    geometry_msgs::msg::Pose pose_msg = tf2::toMsg(cylinder_pose_in_hand);

    RCLCPP_INFO_STREAM(LOGGER, "Attach collision object: " << id 
                               << " position: " << pose_msg.position.x << ", " << pose_msg.position.y << ", " << pose_msg.position.z 
                               << " orientation: " << pose_msg.orientation.x << ", " << pose_msg.orientation.y << ", " << pose_msg.orientation.z << ", " << pose_msg.orientation.w);

    attach_msg.object.primitives.push_back(prim);
    attach_msg.object.primitive_poses.push_back(pose_msg);
    attach_msg.object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Ignore collision with both grippers
    attach_msg.touch_links = touch_links;

    scene->processAttachedCollisionObjectMsg(attach_msg);

    // add the object but disable cable collision if enable_cable_collision is false
    if (!enable_cable_collision) {
      collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();

      bool allow = true;  // always allowed to collide

      // Let dlo_obj collide with everything by default
      acm.setDefaultEntry(id, allow);
      acm.setEntry(id, allow);
    }

    // visualization
    Eigen::Isometry3d pose_in_world = scene->getFrameTransform(attach_link) * cylinder_pose_in_hand;
    // Convert the pose to a geometry_msgs::Pose for visualization
    geometry_msgs::msg::Pose pose_msg_world = tf2::toMsg(pose_in_world);
    visual_tools_.publishCylinder(pose_msg_world, rviz_visual_tools::ORANGE, length, radius);
    visual_tools_.trigger();

}

void ConnectMFReverse::detachCollisionCable(planning_scene::PlanningScenePtr scene,
                                            const std::string& id)
{
    // Step 1: Detach from robot
    moveit_msgs::msg::AttachedCollisionObject detach_attached;
    detach_attached.object.id = id;
    detach_attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene->processAttachedCollisionObjectMsg(detach_attached);

    // Step 2: Remove from world (scene)
    moveit_msgs::msg::CollisionObject remove_from_world;
    remove_from_world.id = id;
    remove_from_world.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene->processCollisionObjectMsg(remove_from_world);
}

// Same logic as your attachCollisionCable(), but takes hand_to_tcp as a parameter
void ConnectMFReverse::attachCollisionCableGeneric(planning_scene::PlanningScenePtr scene,
                                 const std::string& id,
                                 double length,
                                 double radius,
                                 const Eigen::Vector3d& vec_in_world,
                                 const std::string& attach_link,
                                 const Eigen::Isometry3d& hand_to_tcp_transform,
                                 const std::vector<std::string>& touch_links,
                                 const rclcpp::Logger& LOGGER)
{
  moveit_msgs::msg::AttachedCollisionObject attach_msg;
  attach_msg.link_name = attach_link;
  attach_msg.object.header.frame_id = attach_link;
  attach_msg.object.id = id;

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = prim.CYLINDER;
  prim.dimensions = {length, radius}; // height (along local +Z), radius

  // 1) Pose in TCP frame: +Z aligned to vec_in_world (expressed in attach_link)
  Eigen::Isometry3d cylinder_pose_tcp = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d world_to_hand = scene->getFrameTransform(attach_link).inverse();
  Eigen::Vector3d vec_in_hand = world_to_hand.linear() * vec_in_world.normalized();
  Eigen::Quaterniond align_quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), vec_in_hand);
  cylinder_pose_tcp.linear() = align_quat.toRotationMatrix();
  cylinder_pose_tcp.translation() = vec_in_hand.normalized() * (0.5 * length);

  // 2) Pose in hand frame (hand→TCP offset)
  Eigen::Isometry3d cylinder_pose_in_hand = hand_to_tcp_transform * cylinder_pose_tcp;

  // 3) To msg
  geometry_msgs::msg::Pose pose_msg = tf2::toMsg(cylinder_pose_in_hand);

  attach_msg.object.primitives.push_back(prim);
  attach_msg.object.primitive_poses.push_back(pose_msg);
  attach_msg.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Leader touch links = no collision with leader
  attach_msg.touch_links = touch_links;

  scene->processAttachedCollisionObjectMsg(attach_msg);

  // viz (world pose)
  Eigen::Isometry3d pose_in_world = scene->getFrameTransform(attach_link) * cylinder_pose_in_hand;
  geometry_msgs::msg::Pose pose_msg_world = tf2::toMsg(pose_in_world);
  visual_tools_.publishCylinder(pose_msg_world, rviz_visual_tools::BLUE, length, radius);
  visual_tools_.trigger();

  RCLCPP_INFO_STREAM(LOGGER, "Attached temp cable '" << id << "' to " << attach_link
                        << " len=" << length << " r=" << radius);
}

void ConnectMFReverse::detachCollisionCableWorldAndRobot(planning_scene::PlanningScenePtr scene,
                                       const std::string& id)
{
  moveit_msgs::msg::AttachedCollisionObject detach_attached;
  detach_attached.object.id = id;
  detach_attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  scene->processAttachedCollisionObjectMsg(detach_attached);

  moveit_msgs::msg::CollisionObject remove_from_world;
  remove_from_world.id = id;
  remove_from_world.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  scene->processCollisionObjectMsg(remove_from_world);
}


}  // namespace connect_master_follower
}  // namespace task_constructor
}  // namespace moveit
