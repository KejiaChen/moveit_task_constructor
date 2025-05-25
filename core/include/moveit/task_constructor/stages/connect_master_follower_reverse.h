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

protected:
  void compute(const InterfaceState& from, const InterfaceState& to) override;

private:
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
                                        std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes);
 
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
                                  // const std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes,
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
