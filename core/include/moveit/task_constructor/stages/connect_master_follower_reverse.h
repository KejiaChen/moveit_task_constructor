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
   Desc:    Connect arbitrary states by motion planning
*/

#pragma once
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cctype>
#include <tf2_eigen/tf2_eigen.hpp>

namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotState);
}
}  // namespace moveit

namespace moveit {
namespace task_constructor {

namespace stages {

/** Connect arbitrary InterfaceStates by motion planning
 *
 * The states may differ in various planning groups.
 * To connect both states, the planners provided for individual sub groups are applied in the
 * specified order. Each planner only plan for joints within the corresponding planning group.
 * Finally, an attempt is made to merge the sub trajectories of individual planning results.
 * If this fails, the sequential planning result is returned.
 */
using GroupPoseDict = std::map<std::string, geometry_msgs::msg::PoseStamped>;
using GroupStringDict = std::map<std::string, std::string>;

// ---- 1) Dump one trajectory to a TXT file: time, q..., dq... ----
inline bool dumpTrajectoryTXT(const robot_trajectory::RobotTrajectory& traj,
                              const std::string& joint_filename,
                              const std::string& tcp_filename,
                              const std::string& group_name = "",
                              const Eigen::Isometry3d &offset = Eigen::Isometry3d::Identity(),
                              std::string base_link_name = "base_link",
                              char delim = ' ',          // use ' ' for space-separated
                              int precision = 6,
                              double fallback_dt = -1.0) // e.g., 0.05 if times are all zero
{
  const std::string group = group_name.empty() ? traj.getGroupName() : group_name;
  const auto* jmg = traj.getRobotModel()->getJointModelGroup(group);
  if (!jmg) return false;

  std::vector<const moveit::core::LinkModel*> tips;
  if (!jmg->getEndEffectorTips(tips) || tips.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("RobotTrajectory"), "Unable to get end effector tips from jmg");
    return false;
  }

  const auto& names = jmg->getVariableNames();
  const size_t N = names.size();
  const size_t M = traj.getWayPointCount();
  if (M == 0 || N == 0) return false;

  std::filesystem::create_directories(std::filesystem::path(joint_filename).parent_path());
  std::ofstream joint_out(joint_filename);
  if (!joint_out) return false;
  joint_out.setf(std::ios::fixed, std::ios::floatfield);
  joint_out << std::setprecision(precision);

  std::filesystem::create_directories(std::filesystem::path(tcp_filename).parent_path());
  std::ofstream tcp_out(tcp_filename);
  if (!tcp_out) return false;
  tcp_out.setf(std::ios::fixed, std::ios::floatfield);
  tcp_out << std::setprecision(precision);

  // // Header
  // out << "# time";
  // for (auto& n : names) out << delim << "q/" << n;
  // for (auto& n : names) out << delim << "dq/" << n;
  // out << "\n";

  // Gather data
  std::vector<double> T(M, 0.0);
  std::vector<std::vector<double>> Q(M, std::vector<double>(N, 0.0));
  std::vector<std::vector<double>> dQ(M, std::vector<double>(N, 0.0));
  std::vector<Eigen::Isometry3d> TCP_pose_in_world(M, Eigen::Isometry3d::Identity());
  std::vector<Eigen::Isometry3d> TCP_pose_in_base(M, Eigen::Isometry3d::Identity());

  bool any_velocity = false;
  Eigen::Isometry3d robot_base_pose = traj.getWayPoint(0).getGlobalLinkTransform(base_link_name);
  for (size_t i = 0; i < M; ++i) {
    const auto& s = traj.getWayPoint(i);
    T[i] = traj.getWayPointDurationFromStart(i);
    s.copyJointGroupPositions(jmg, Q[i]);
    s.copyJointGroupVelocities(jmg, dQ[i]); // zeros if not set

    for (const moveit::core::LinkModel* ee_parent_link : tips){
      // pose in world frame for publishing
      Eigen::Isometry3d ee_pose_in_world= s.getGlobalLinkTransform(ee_parent_link);
      // Apply the translation in the z-axis
      Eigen::Isometry3d tcp_pose_in_world = ee_pose_in_world*offset;
      TCP_pose_in_world[i] = tcp_pose_in_world;

      // ee_pose in robot base frame for storage 
      Eigen::Isometry3d ee_pose_in_base = robot_base_pose.inverse()*ee_pose_in_world;
      // Apply the translation in the z-axiss
      Eigen::Isometry3d tcp_pose_in_base = ee_pose_in_base*offset;
      TCP_pose_in_base[i] = tcp_pose_in_base;
    }

    for (double v : dQ[i]) if (std::abs(v) > 1e-12) { any_velocity = true; break; }
  }

  // Dump rows
  for (size_t i = 0; i < M; ++i) {
    joint_out << T[i];
    for (size_t j = 0; j < N; ++j) joint_out << delim << Q[i][j];
    for (size_t j = 0; j < N; ++j) joint_out << delim << dQ[i][j];
    joint_out << "\n";
  }

  // Dump TCP poses
  for (size_t i = 0; i < M; ++i) {
    tcp_out << T[i];
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 4; ++row) {
        tcp_out << delim << TCP_pose_in_base[i](row, col);
      }
    }
    tcp_out << "\n";  // Newline for the next matrix
  }

  return true;
}

// ---- 2) Auto-incremented filename in a directory (current folder by default) ----
inline std::filesystem::path nextIndexedFile(const std::filesystem::path& dir,
                                              const std::string& prefix,
                                              const std::string& ext = ".txt",
                                              int width = 3,
                                              int start_index = 1)
{
  std::filesystem::create_directories(dir);
  int max_idx = start_index - 1;

  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    if (!entry.is_regular_file()) continue;
    const auto name = entry.path().filename().string();

    // check prefix_
    if (name.size() < prefix.size() + 1 + ext.size()) continue;
    if (name.compare(0, prefix.size(), prefix) != 0) continue;
    if (name[prefix.size()] != '_') continue;

    // check suffix .ext
    if (name.compare(name.size() - ext.size(), ext.size(), ext) != 0) continue;

    // digits in the middle
    const auto digits = name.substr(prefix.size() + 1,
                                    name.size() - prefix.size() - 1 - ext.size());
    if (digits.empty() || !std::all_of(digits.begin(), digits.end(),
                                       [](unsigned char c){ return std::isdigit(c); }))
      continue;

    int idx = std::stoi(digits);
    if (idx > max_idx) max_idx = idx;
  }

  const int next = std::max(start_index, max_idx + 1);
  std::ostringstream oss;
  oss << prefix << "_" << std::setw(width) << std::setfill('0') << next << ext;
  return dir / oss.str();
}

// ---- 3) Convenience: dump with auto-index into current folder ----
inline bool dumpTrajectoryTXTIndexed(const robot_trajectory::RobotTrajectory& traj,
                                     const std::string& prefix,
                                     const std::string& group_name = "",
                                     const std::filesystem::path& dir = std::filesystem::current_path(),
                                     const Eigen::Isometry3d &offset = Eigen::Isometry3d::Identity(),
                                      std::string base_link_name = "base_link",
                                     char delim = ' ', int precision = 6,
                                     double fallback_dt = 0.05, // pick something reasonable
                                     int width = 3, int start_index = 1)
{
  const auto joint_path = nextIndexedFile(dir, prefix + "_joint", ".txt", width, start_index);
  const auto tcp_path = nextIndexedFile(dir, prefix + "_tcp", ".txt", width, start_index);
  RCLCPP_INFO(rclcpp::get_logger("RobotTrajectory"),
            "Dumping trajectory to %s", joint_path.string().c_str());
  return dumpTrajectoryTXT(traj, joint_path.string(), tcp_path.string(), group_name, offset, base_link_name, delim, precision, fallback_dt);
}

inline bool dumpPathTxTIndexed(const std::vector<geometry_msgs::msg::Pose>& path,
                              const std::string& prefix,
                              char delim = ' ', 
                              int precision = 6)
{
  std::string filename = nextIndexedFile("MTC_connect_visualization", prefix, ".txt", 3, 1);
  std::ofstream out(filename);
  if (!out) return false;
  out.setf(std::ios::fixed, std::ios::floatfield);
  out << std::setprecision(precision);

  for (const auto& pose : path)
  {
    out << pose.position.x << delim
        << pose.position.y << delim
        << pose.position.z << delim
        << pose.orientation.x << delim
        << pose.orientation.y << delim
        << pose.orientation.z << delim
        << pose.orientation.w << "\n";
  }
  return true;
}

inline double computeRotationDistance(
    const Eigen::Matrix3d& Ra,
    const Eigen::Matrix3d& Rb)
{
  const Eigen::Matrix3d R_rel = Ra.transpose() * Rb;
  const Eigen::AngleAxisd aa(R_rel);
  // angle in [0, pi], axis expressed in Ra frame
  return aa.angle();
}

inline double computeRotationDistanceIgnoreAxisChar(
    const Eigen::Matrix3d& Ra,
    const Eigen::Matrix3d& Rb,
    char ignore_axis) // 'x'|'y'|'z' (case-insensitive)
{
  const Eigen::Matrix3d R_rel = Ra.transpose() * Rb;
  const Eigen::AngleAxisd aa(R_rel);
  Eigen::Vector3d w = aa.angle() * aa.axis();  // rotation vector in Ra frame

  char ax = static_cast<char>(std::tolower(ignore_axis));
  Eigen::Vector3d v(0,0,0);
  if (ax == 'x') v = Eigen::Vector3d::UnitX();
  else if (ax == 'y') v = Eigen::Vector3d::UnitY();
  else if (ax == 'z') v = Eigen::Vector3d::UnitZ();
  else return w.norm(); // unknown char → no ignore

  const double n = v.norm();
  if (n < 1e-12) return w.norm();
  v /= n;

  const Eigen::Vector3d w_perp = w - v * (v.dot(w)); // remove spin about v
  return w_perp.norm();
}

inline double computeRotationDistanceIgnoreAxisVec(
    const Eigen::Matrix3d& Ra,
    const Eigen::Matrix3d& Rb,
    const Eigen::Vector3d& ignore_axis_in_Ra) // already in Ra frame
{
  const Eigen::Matrix3d R_rel = Ra.transpose() * Rb;
  const Eigen::AngleAxisd aa(R_rel);
  Eigen::Vector3d w = aa.angle() * aa.axis();  // rotation vector in Ra frame

  Eigen::Vector3d v = ignore_axis_in_Ra;
  const double n = v.norm();
  if (n < 1e-12) return w.norm();
  v /= n;

  const Eigen::Vector3d w_perp = w - v * (v.dot(w));
  return w_perp.norm();
}


class ConnectMFReverse : public Connect
{

public:
  using GroupCartPlannerVector = std::vector<std::pair<std::string, solvers::CartesianPathPtr>>;
  using GroupPipePlannerVector = std::vector<std::pair<std::string, solvers::PipelinePlannerPtr>>;
protected:
  GroupPlannerVector interpolation_planner_;
  GroupCartPlannerVector cartesian_planner_;
  GroupPipePlannerVector chomp_planner_;
  // GroupPlannerVector hand_planner_;

public:
  ConnectMFReverse(const std::string& name, const GroupPlannerVector& planners, 
            const GroupPlannerVector& interpolation_planners,
            const GroupCartPlannerVector& cartesian_planners,
            const GroupPipePlannerVector& chomp_planners,
            const GroupPlannerVector& hand_planners,
            const moveit::planning_interface::MoveGroupInterfacePtr& move_group_follow,
            moveit_visual_tools::MoveItVisualTools visual_tools);

  void setEndEffector(const GroupStringDict& eefs) {setProperty("eefs", eefs); }

  void init(const moveit::core::RobotModelConstPtr& robot_model) override;

  /* Either Leader or Follower's Path Constraints can be set. Never set both.*/
  // void setLeaderPathConstraints(moveit_msgs::msg::Constraints path_constraints) {
	// 	setProperty("lead_path_constraints", std::move(path_constraints));
	// }

  void setFollowerPathConstraints(moveit_msgs::msg::Constraints path_constraints) {
    setProperty("follow_path_constraints", std::move(path_constraints));
  }

protected:
  void compute(const InterfaceState& from, const InterfaceState& to) override;

private:
  // struct MatchQuality {
  //   double arc_length;
  //   std::shared_ptr<moveit::core::RobotState> state;
  //   double magnitude_diff;
  //   double alignment;
  //   double score;
  // };

  bool computeSecondArmTrajectoryReverse(const InterfaceState& from, const InterfaceState& to,
                                        robot_trajectory::RobotTrajectoryPtr& follower_trajectory,
                                        planning_scene::PlanningScenePtr& intermediate_scene,
                                        planning_scene::PlanningScenePtr& final_scene,
                                        std::string& return_message,
                                        bool attach_object=false); 

  bool computeFirstArmTrajectoryReverse(robot_trajectory::RobotTrajectoryPtr& follower_trajectory,
                                        robot_trajectory::RobotTrajectoryPtr& follower_hand_trajectory,
                                        std::vector<PlannerIdTrajectoryPair>& follower_trajectories,
                                        planning_scene::PlanningSceneConstPtr& to_scene,
                                        robot_trajectory::RobotTrajectoryPtr& leader_trajectory,
                                        std::vector<PlannerIdTrajectoryPair>& leader_trajectories,
                                        // robot_trajectory::RobotTrajectoryPtr& dual_trajectory,
                                        planning_scene::PlanningScenePtr& follow_intermediate_scene,
                                        planning_scene::PlanningScenePtr& follow_final_scene,
                                        std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes,
                                        std::string& return_message);
 
  bool ExtractSecondArmCartesianTrajectory(const robot_trajectory::RobotTrajectoryPtr& follower_trajectory,
                                            const moveit::core::RobotState& final_goal_state,
                                            std::vector<geometry_msgs::msg::Pose>& follower_tip_path,
                                            std::vector<double>& path_time,
                                            //  int& start_index,
                                            double start_offset,
                                            // robot_trajectory::RobotTrajectoryPtr& follower_grasp_trajectory,
                                            robot_trajectory::RobotTrajectoryPtr& follower_track_trajectory,
                                            bool reverse=false);
  
  double FirstArmFollow(planning_scene::PlanningScenePtr& intermediate_scene,
                        std::vector<geometry_msgs::msg::Pose> leader_tip_path,
                        robot_trajectory::RobotTrajectoryPtr& lead_trajectory);

  SubTrajectoryPtr mergeIgnoreCollision(const std::vector<PlannerIdTrajectoryPair>& sub_trajectories,
                                  const planning_scene::PlanningSceneConstPtr& intermediate_scene,
                                  const moveit::core::RobotState& state);
  
  bool splitTrajectoryWithPause(const robot_trajectory::RobotTrajectoryPtr& trajectory,
                                const double pause_duration,
                                const int split_index,
                                robot_trajectory::RobotTrajectoryPtr& split_trajectory,
                                std::vector<robot_trajectory::RobotTrajectoryPtr>& split_trajectories,
                                bool if_return_full=true);
                              
  robot_trajectory::RobotTrajectory reinterpolateTrajectory(const robot_trajectory::RobotTrajectoryPtr& original_trajectory, 
                                                           double total_time, double waypoint_interval);
  
  void splitGroupFromState(const moveit::core::JointModelGroup* group,
                            const moveit::core::RobotState& dual_state,
                            moveit::core::RobotState& single_group_state);

  void updateDualIntermediateState(const moveit::core::RobotState& leader_state,
                                    const moveit::core::RobotState& follower_state,
                                    planning_scene::PlanningScenePtr& start,
                                    planning_scene::PlanningScenePtr& end);
  
  bool isTargetPoseCollidingInEEF(const planning_scene::PlanningSceneConstPtr& scene,
                                  moveit::core::RobotState& robot_state, 
                                  EigenSTL::vector_Isometry3d& poses,
                                  std::vector<const moveit::core::LinkModel*>& links,
                                  const moveit::core::JointModelGroup* jmg = nullptr,
                                  collision_detection::CollisionResult* collision_result = nullptr);

  Eigen::Quaterniond combineRotations(Eigen::Quaterniond grasp_orientation, 
                                      Eigen::Quaterniond clip_orientation);

  moveit_msgs::msg::Constraints setLineConstraint(planning_scene::PlanningSceneConstPtr start,
                                                planning_scene::PlanningSceneConstPtr end,
                                                std::string constraint_link_name);

  moveit_msgs::msg::Constraints setBoxConstraint(planning_scene::PlanningSceneConstPtr start,
                                                    planning_scene::PlanningSceneConstPtr end,
                                                    std::string constraint_link_name);

  moveit_msgs::msg::Constraints setOrientationConstraint(const moveit::core::RobotState& current_state, 
                                                        const std::string& constraint_link_name)
  { 
    const auto& current_pose = current_state.getGlobalLinkTransform(constraint_link_name);
    Eigen::Quaterniond current_orientation(current_pose.rotation());

    // Create orientation constraint
    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    orientation_constraint.header.frame_id = "world";
    orientation_constraint.link_name = constraint_link_name;
    orientation_constraint.orientation = tf2::toMsg(current_orientation);
    orientation_constraint.parameterization = moveit_msgs::msg::OrientationConstraint::XYZ_EULER_ANGLES;  
    orientation_constraint.absolute_x_axis_tolerance = M_PI;     // free
    orientation_constraint.absolute_y_axis_tolerance = M_PI;     // LOCK twist about local Y axis
    orientation_constraint.absolute_z_axis_tolerance = M_PI;     // free
    orientation_constraint.weight = 1.0;
    
    // Wrap in a generic Constraints message
    moveit_msgs::msg::Constraints orientation_constraints;
    orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);

    return orientation_constraints;
  }
  
  // moveit_msgs::msg::Constraints createTrajectoryConstraintsFromTrajectory(const moveit_msgs::msg::RobotTrajectory& robot_traj_msg);

  void attachCollisionCable(planning_scene::PlanningScenePtr scene,
                            const std::string& id, 
                            double length,
                            double radius,
                            Eigen::Vector3d vec_in_world,
                            const std::string& attach_link, 
                            std::vector<std::string> touch_links);

  void detachCollisionCable(planning_scene::PlanningScenePtr scene,
                             const std::string& id);
  
  const moveit::core::JointModelGroup* follow_jmg_;
  const moveit::core::JointModelGroup* follow_hand_jmg_;
  const moveit::core::JointModelGroup* leader_jmg_;
  const moveit::core::JointModelGroup* leader_hand_jmg_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_lead_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;

  Eigen::Isometry3d hand_to_tcp_transform_;
  Eigen::Isometry3d lead_flange_to_tcp_transform_;
  Eigen::Isometry3d follow_flange_to_tcp_transform_;

  int follower_start_index_ = -1;
  int reversed_follower_start_index_ = -1;
  int follower_grasp_index_ = -1;
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
