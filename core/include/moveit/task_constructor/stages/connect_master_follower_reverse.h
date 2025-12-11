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
#include <visualization_msgs/msg/marker_array.hpp>
#include <rviz_marker_tools/marker_creation.h>
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <random>

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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ConnectMFReverse");

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

  std::mt19937 rng_{ std::random_device{}() };
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
                                        std::string& return_message); 

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
                            std::vector<std::string> touch_links,
                            bool enable_cable_collision);

  void detachCollisionCable(planning_scene::PlanningScenePtr scene,
                             const std::string& id);

  void attachCollisionCableGeneric(planning_scene::PlanningScenePtr scene,
                                 const std::string& id,
                                 double length,
                                 double radius,
                                 const Eigen::Vector3d& vec_in_world,
                                 const std::string& attach_link,
                                 const Eigen::Isometry3d& hand_to_tcp_transform,
                                 const std::vector<std::string>& touch_links,
                                 const rclcpp::Logger& LOGGER);

  void detachCollisionCableWorldAndRobot(planning_scene::PlanningScenePtr scene,
                                       const std::string& id); 

  // --- Helper: resolve "frame_name" into a world transform without TF ---
  // Works for: robot/link frames, planning frame, and *world objects by id*.
  inline bool resolveFrameInSceneTFfree(const planning_scene::PlanningSceneConstPtr& scene,
                                        const std::string& frame_name,
                                        Eigen::Isometry3d& T_world_frame_out)
  {
    // Planning frame shortcut
    if (frame_name == scene->getPlanningFrame()) {
      T_world_frame_out.setIdentity();
      return true;
    }

    // Known to PlanningScene's internal transform graph?
    if (scene->knowsFrameTransform(frame_name)) {
      T_world_frame_out = scene->getFrameTransform(frame_name);
      return true;
    }

    // Try as a world object id
    auto obj = scene->getWorld()->getObject(frame_name);
    if (obj) {
      // Prefer the object's pose in world, if available (MoveIt stores pose_ for the object)
      // NOTE: pose_ is public in World::Object in current MoveIt; if not, fall back to first shape pose.
  #if defined(HAVE_WORLD_OBJECT_POSE_) || 1
      // Most MoveIt versions expose obj->pose_
      T_world_frame_out = obj->pose_;
      return true;
  #else
      // Fallback: use first shape pose if present (already in world)
      if (!obj->shape_poses_.empty()) {
        T_world_frame_out = obj->shape_poses_.front();
        return true;
      }
  #endif
    }

    return false; // unknown to scene
  }

  // --- Transform pose_in from its header frame -> target_frame, TF-free via PlanningScene/World ---
  inline geometry_msgs::msg::PoseStamped transformPoseWithScene(
      const planning_scene::PlanningSceneConstPtr& scene,
      const geometry_msgs::msg::PoseStamped& pose_in,
      const std::string& target_frame)
  {
    geometry_msgs::msg::PoseStamped out = pose_in;

    // Early exit if frames already match
    if (pose_in.header.frame_id == target_frame)
      return out;

    // Resolve source frame in world
    Eigen::Isometry3d T_world_src;
    if (!resolveFrameInSceneTFfree(scene, pose_in.header.frame_id, T_world_src)) {
      RCLCPP_ERROR(LOGGER, "Cannot resolve source frame '%s' in PlanningScene/World",
                  pose_in.header.frame_id.c_str());
      return out; // unchanged
    }

    // Resolve target frame in world
    Eigen::Isometry3d T_world_tgt;
    if (!resolveFrameInSceneTFfree(scene, target_frame, T_world_tgt)) {
      RCLCPP_ERROR(LOGGER, "Cannot resolve target frame '%s' in PlanningScene/World",
                  target_frame.c_str());
      return out; // unchanged
    }

    // Pose in source frame -> Eigen
    Eigen::Isometry3d T_src_pose = Eigen::Isometry3d::Identity();
    tf2::fromMsg(pose_in.pose, T_src_pose);

    // target <- pose  = (target <- world) * (world <- source) * (source <- pose)
    const Eigen::Isometry3d T_tgt_pose = T_world_tgt.inverse() * T_world_src * T_src_pose;

    out.header.frame_id = target_frame;
    out.pose = tf2::toMsg(T_tgt_pose);
    return out;
  }


  /* Sample Grasping Position and Orientation*/
  struct ClipSamplingWindow {
    double theta_min = 0;        // 30°
    double theta_max = M_PI / 2.0;  // 150°
    double phi_min   = 0.0;
    double phi_max   = 0.0;//M_PI / 12.0;        // 15°
  };

  // Return unit vector g in the CLIP frame (goal frame)
  inline std::pair<Eigen::Vector3d, Eigen::Quaterniond> sample_g_in_clip(std::mt19937& rng,
                                                                        ClipSamplingWindow win,
                                                                        int clip_sign) {
    std::uniform_real_distribution<double> U(0.0, 1.0);

    // 1) Sample phi uniformly in [phi_min, phi_max]
    double phi = win.phi_min + (win.phi_max - win.phi_min) * U(rng);
    double cp   = std::cos(phi);
    double sp   = std::sin(phi);

    // 2) Area-uniform theta: sample cos(theta) uniformly on [cos(theta_max), cos(theta_min)]
    // const double cmin = std::cos(win.theta_max);   // lower cos = more tilt
    // const double cmax = std::cos(win.theta_min);   // upper cos
    // const double c    = cmin + (cmax - cmin) * U(rng);   // cos(theta)
    // const double s    = std::sqrt(std::max(0.0, 1.0 - c * c));
    double theta = win.theta_min + (win.theta_max - win.theta_min) * U(rng);
    double c     = std::cos(theta);
    double s     = std::sin(theta);

    // u=[1,0,0], v=[0,1,0], w=[0,0,1] in the clip frame
    // g = c*u + s*(cp*v + sp*w)
    //   = [ c,  s*cp,  s*sp ]
    Eigen::Vector3d g(c, s * cp, s * sp);
    // align with clip frame, rotate 180° about z (w): (x,y,z) -> (-x,-y,z)
    if (clip_sign < 0) {
      g.x() = -g.x();
      g.y() = -g.y();
    }else {
      g.x() = -s * cp;
      g.y() = c;
    }
    // g.x() = clip_sign * g.x();
    // g.y() = clip_sign * g.y();
    // g.z() unchanged

    // construct quaternion with theta and phi
    if (clip_sign < 0) {
      theta += M_PI/2; // flip about z
    }else{
      theta = theta;
    }
    Eigen::AngleAxisd Rz_theta(theta, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Rx_phi  (phi,   Eigen::Vector3d::UnitX());
    Eigen::Quaterniond quat_clip = Eigen::Quaterniond(Rx_phi) * Eigen::Quaterniond(Rz_theta); // Rx(phi)*Rz(theta)
    if (clip_sign < 0) {
      quat_clip = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())) * quat_clip;  // Rz(pi) *
    }

    return {g.normalized(), quat_clip.normalized()};  // already unit, normalization is a safety net
  }

  // // Optional: convenience that returns a Vector3Stamped in the clip frame
  // inline geometry_msgs::msg::Vector3Stamped sample_g_msg(std::mt19937& rng,
  //                                                       const std::string& clip_frame,
  //                                                       ClipSamplingWindow win) {
  //   // Eigen::Vector3d g;
  //   // Eigen::Quaterniond quat_clip;
  //   auto [g, quat_clip] = sample_g_in_clip(rng, win, );
  //   geometry_msgs::msg::Vector3Stamped out;
  //   out.header.frame_id = clip_frame;
  //   out.vector.x = g.x();
  //   out.vector.y = g.y();
  //   out.vector.z = g.z();
  //   return out;
  // }

  inline Eigen::Isometry3d getLinkPoseInClipFrame(
      const planning_scene::PlanningSceneConstPtr& scene,
      const std::string& link_name,
      const std::string& clip_frame)
  {
    // world <- link
    const moveit::core::RobotState& rs = scene->getCurrentState();
    const Eigen::Isometry3d T_world_link = rs.getGlobalLinkTransform(link_name);

    // world <- clip
    if (!scene->knowsFrameTransform(clip_frame)) {
      throw std::runtime_error("PlanningScene does not know clip frame: " + clip_frame);
    }
    const Eigen::Isometry3d T_world_clip = scene->getFrameTransform(clip_frame);

    // clip <- link
    return T_world_clip.inverse() * T_world_link;
  }

  // --- 2) Pick the default orientation (clip frame) closest to current ---
  inline Eigen::Quaterniond selectDefaultOrientationInClip(
      const Eigen::Quaterniond& q_current_clip,
      const Eigen::Quaterniond& q1_clip,
      const Eigen::Quaterniond& q2_clip,
      bool select_orientation /*if false, always q1*/)
  {
    if (!select_orientation) return q1_clip;
    const double a1 = q_current_clip.angularDistance(q1_clip);
    const double a2 = q_current_clip.angularDistance(q2_clip);
    return (a1 <= a2) ? q1_clip : q2_clip;
  }

  // --- 3) Build TCP pose at grasp_center + d*g with selected default orientation (clip frame) ---
  inline geometry_msgs::msg::PoseStamped tcp_at_clip_with_default_orientation(
      const planning_scene::PlanningSceneConstPtr& scene,
      const std::string& ee_link_name,           // e.g., "right_panda_hand"
      const Eigen::Isometry3d& grasp_center_clip,// origin/center in clip frame
      const Eigen::Vector3d& g_clip,             // unit grasp direction (clip frame)
      double d,                                   // distance along g
      const std::string& clip_frame,
      bool select_orientation,                    // mimic your flag
      // your two default quaternions in CLIP frame:
      const Eigen::Quaterniond& q_default_1_clip,
      const Eigen::Quaterniond& q_default_2_clip)
  {
    // current EE orientation in CLIP frame
    const Eigen::Isometry3d T_clip_link = getLinkPoseInClipFrame(scene, ee_link_name, clip_frame);
    const Eigen::Quaterniond q_curr_clip(T_clip_link.rotation());

    // choose default orientation
    const Eigen::Quaterniond q_goal_clip =
        selectDefaultOrientationInClip(q_curr_clip, q_default_1_clip, q_default_2_clip, select_orientation);

    // position = grasp_center + d*g
    const Eigen::Vector3d p_clip = grasp_center_clip.translation() + d * g_clip;

    // pack PoseStamped in CLIP frame
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = clip_frame;
    p.pose.position = tf2::toMsg(p_clip);
    p.pose.orientation = tf2::toMsg(q_goal_clip.normalized());
    return p;
  }

  // Example: compute TCP positions in clip frame, then (optionally) make PoseStamped
  inline geometry_msgs::msg::PoseStamped tcp_at_clip_origin_plus(
                                                                const Eigen::Isometry3d & grasp_center,
                                                                const Eigen::Vector3d& g,
                                                                const Eigen::Quaterniond& q,
                                                                double d,
                                                                const std::string& clip_frame) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = clip_frame;
    p.pose.position.x = grasp_center.translation().x() + d * g.x();
    p.pose.position.y = grasp_center.translation().y() + d * g.y();
    p.pose.position.z = grasp_center.translation().z() + d * g.z();
    // Orientation can be set later (e.g., align hand axis with ±g). Identity here:
    Eigen::Quaterniond nq = q.normalized();
    p.pose.orientation.x = nq.x();
    p.pose.orientation.y = nq.y();
    p.pose.orientation.z = nq.z();
    p.pose.orientation.w = nq.w();
    return p;
  }

  // Build yaw quaternion (about world Z) from a world-frame direction vector
  inline Eigen::Quaterniond yawFromDirectionWorldZ(const Eigen::Vector3d& dir_world) {
    // Project onto XY, fallback to +X if degenerate
    Eigen::Vector2d p(dir_world.x(), dir_world.y());
    if (p.norm() < 1e-9) return Eigen::Quaterniond::Identity();
    double yaw = std::atan2(p.y(), p.x());
     return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  }

  // Combine: yaw from sampled direction, pitch/roll from fixing
  inline Eigen::Quaterniond combineYawFromDir_keepFixingPitchRoll(
      const Eigen::Vector3d& grasp_dir_world,
      const Eigen::Quaterniond& fixing_orientation)
  {
    // 1) yaw from grasp direction
    Eigen::Quaterniond grasp_yaw_q = yawFromDirectionWorldZ(grasp_dir_world);

    // 2) remove yaw from fixing
    Eigen::Matrix3d R_fix = fixing_orientation.toRotationMatrix();
    double fix_yaw = std::atan2(R_fix(1,0), R_fix(0,0));
    Eigen::AngleAxisd yaw_inv(-fix_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond fix_wo_yaw(yaw_inv * R_fix);

    // 3) apply grasp yaw
    return grasp_yaw_q * fix_wo_yaw;
  }

  static void makeAxisArrows(visualization_msgs::msg::Marker& m_template,
                            const Eigen::Isometry3d& T_world,
                            double axis_len,
                            std::deque<visualization_msgs::msg::Marker>& out)
  {
    const Eigen::Vector3d p0 = T_world.translation();
    const Eigen::Quaterniond q(T_world.rotation());

    auto make_arrow = [&](const Eigen::Vector3d& axis_dir,
                          const std_msgs::msg::ColorRGBA& color,
                          int id){
      visualization_msgs::msg::Marker m = m_template;
      m.id = id;

      const Eigen::Vector3d p1 = p0 + q * axis_dir.normalized() * axis_len;
      rviz_marker_tools::makeArrow(m, p0, p1, 0.08 * axis_len, 0.024 * axis_len); // shaft len, head len
      m.scale.x = 0.012;   // shaft diameter
      m.scale.y = 0.024;   // head diameter
      m.scale.z = 0.0;     // unused by makeArrow path
      m.color = color;
      out.push_back(std::move(m));
    };

    std_msgs::msg::ColorRGBA red, green, blue;
    red.r=1.0; red.a=1.0; green.g=1.0; green.a=1.0; blue.b=1.0; blue.a=1.0;

    make_arrow(Eigen::Vector3d::UnitX(), red,   /*id*/0); // X
    make_arrow(Eigen::Vector3d::UnitY(), green, /*id*/1); // Y
    make_arrow(Eigen::Vector3d::UnitZ(), blue,  /*id*/2); // Z
  }

  // ---------- adapted visualizer for your inputs ----------
  static void visualizeGraspsForClipInputs(std::deque<visualization_msgs::msg::Marker>& markers,
                                          bool success,
                                          const std::string& ns,
                                          const geometry_msgs::msg::PoseStamped& leader_tcp_world,
                                          const geometry_msgs::msg::PoseStamped& follower_tcp_world,
                                          const geometry_msgs::msg::PoseStamped& clip_origin_world,
                                          double d_m, 
                                          double d_f,
                                          double axis_len = 0.15)
  {
    // Frame id: use leader’s header (you’re passing "world")
    const std::string frame_id = leader_tcp_world.header.frame_id;

    // Convert poses to Eigen
    Eigen::Isometry3d T_leader = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_follower = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_origin = Eigen::Isometry3d::Identity();
    tf2::fromMsg(leader_tcp_world.pose,   T_leader);
    tf2::fromMsg(follower_tcp_world.pose, T_follower);
    tf2::fromMsg(clip_origin_world.pose,  T_origin);

    const Eigen::Vector3d p_lead    = T_leader.translation();
    const Eigen::Vector3d p_follow  = T_follower.translation();
    const Eigen::Vector3d p_origin  = T_origin.translation();

    // Derive g_world from the geometry (leader is at p_origin + d_m * g)
    Eigen::Vector3d g_world = p_lead - p_origin;
    double g_n = g_world.norm();
    if (g_n > 1e-9) g_world /= g_n;      // normalize
    else            g_world = Eigen::Vector3d::UnitX(); // fallback

    visualization_msgs::msg::Marker m;
    m.ns = ns;
    m.header.frame_id = frame_id;
    m.action = visualization_msgs::msg::Marker::ADD;

    // 1) Axis frames at each TCP
    makeAxisArrows(m, T_leader,   axis_len, markers);
    m.id += 3; // avoid id clashes between frames
    makeAxisArrows(m, T_follower, axis_len, markers);
    m.id += 3;

    // 2) Grasp lines from clip origin to each TCP (green if success, red otherwise)
    std_msgs::msg::ColorRGBA ok, bad;
    ok.g=1.0; ok.a=1.0; bad.r=1.0; bad.a=1.0;

    auto line_to_tcp = [&](const Eigen::Vector3d& p_tcp, int id, const std_msgs::msg::ColorRGBA& color){
      visualization_msgs::msg::Marker l = m;
      l.id = id;
      l.type = visualization_msgs::msg::Marker::ARROW;
      rviz_marker_tools::makeArrow(l, p_origin, p_tcp, 0.06 * axis_len, 0.018 * axis_len);
      l.scale.x = 0.008;  // shaft dia
      l.scale.y = 0.018;  // head dia
      l.color = color;
      markers.push_back(std::move(l));
    };

    const auto& C = success ? ok : bad;
    line_to_tcp(p_lead,   m.id++, C);
    line_to_tcp(p_follow, m.id++, C);

    // 3) Intended grasp direction at the clip origin
    {
      visualization_msgs::msg::Marker g = m;
      g.id = m.id++;
      g.type = visualization_msgs::msg::Marker::ARROW;
      rviz_marker_tools::makeArrow(g, p_origin, p_origin + g_world * axis_len, 0.06 * axis_len, 0.018 * axis_len);
      rviz_marker_tools::setColor(g.color, rviz_marker_tools::CYAN);
      markers.push_back(std::move(g));
    }

    // 4) Text with d_m / d_f
    {
      visualization_msgs::msg::Marker t = m;
      t.id = m.id++;
      t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      Eigen::Vector3d text_pos = p_origin + 0.5 * axis_len * Eigen::Vector3d::UnitZ();
      t.pose.position = tf2::toMsg(text_pos);
      t.scale.z = 0.06; // text height
      std::ostringstream oss;
      oss << "d_m=" << std::fixed << std::setprecision(3) << d_m << "  d_f=" << std::fixed << std::setprecision(3) << d_f;
      t.text = oss.str();
      rviz_marker_tools::setColor(t.color, rviz_marker_tools::WHITE);
      markers.push_back(std::move(t));
    }
  }
  
  const moveit::core::JointModelGroup* follow_jmg_;
  const moveit::core::JointModelGroup* follow_hand_jmg_;
  const moveit::core::JointModelGroup* leader_jmg_;
  const moveit::core::JointModelGroup* leader_hand_jmg_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_lead_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;

  Eigen::Isometry3d lead_hand_to_tcp_transform_;
  Eigen::Isometry3d follow_hand_to_tcp_transform_;
  Eigen::Isometry3d lead_flange_to_tcp_transform_;
  Eigen::Isometry3d follow_flange_to_tcp_transform_;
  
  Eigen::Quaterniond lead_grasp_orientation_;
  Eigen::Quaterniond follow_grasp_orientation_;
  Eigen::Quaterniond transport_rotation_; // should be a constant during transport
  Eigen::Isometry3d transport_transform_; // should be a constant during transport

  geometry_msgs::msg::PoseStamped lead_grasp_tcp_pose_clip_;
  geometry_msgs::msg::PoseStamped follow_grasp_tcp_pose_clip_;
  geometry_msgs::msg::PoseStamped lead_grasp_tcp_pose_world_;
  geometry_msgs::msg::PoseStamped follow_grasp_tcp_pose_world_;

  geometry_msgs::msg::PoseStamped lead_reached_grasp_tcp_pose_world_;
  geometry_msgs::msg::PoseStamped follow_reached_grasp_tcp_pose_world_;

  int follower_start_index_ = -1;
  int reversed_follower_start_index_ = -1;
  int follower_grasp_index_ = -1;
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
