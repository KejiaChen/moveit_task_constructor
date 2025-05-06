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
#include <moveit/task_constructor/solvers/cartesian_path.h>

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

class ConnectMFSeq : public Connect
{

public:
  using GroupCartPlannerVector = std::vector<std::pair<std::string, solvers::CartesianPathPtr>>;

protected:
  GroupPlannerVector interpolation_planner_;
  GroupCartPlannerVector cartesian_planner_;

public:
  ConnectMFSeq(const std::string& name, const GroupPlannerVector& planners);

protected:
  void compute(const InterfaceState& from, const InterfaceState& to) override;

private:
  bool computeFirstArmTrajectory(const InterfaceState& from, const InterfaceState& to,
                                  robot_trajectory::RobotTrajectoryPtr& first_arm_trajectory,
                                  planning_scene::PlanningScenePtr& intermediate_scene);

  bool computeSecondArmTrajectory(robot_trajectory::RobotTrajectoryPtr& leader_trajectory,
                                  std::vector<PlannerIdTrajectoryPair>& leader_trajectories,
                                  const InterfaceState& to,
                                  robot_trajectory::RobotTrajectoryPtr& follower_trajectory,
                                  std::vector<PlannerIdTrajectoryPair>& follower_trajectories,
                                  planning_scene::PlanningScenePtr& lead_final_scene,
                                  std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes);

  SubTrajectoryPtr mergeIgnoreCollision(const std::vector<PlannerIdTrajectoryPair>& sub_trajectories,
                                  // const std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes,
                                  const moveit::core::RobotState& state);
  
  bool splitTrajectoryWithPause(const robot_trajectory::RobotTrajectoryPtr& trajectory,
                                const double pause_duration,
                                const int split_index,
                                robot_trajectory::RobotTrajectoryPtr& split_trajectory,
                                std::vector<robot_trajectory::RobotTrajectoryPtr>& split_trajectories,
                                bool if_return_full=true);
                              
  void splitGroupFromState(const moveit::core::JointModelGroup* group,
                            const moveit::core::RobotState& dual_state,
                            moveit::core::RobotState& single_group_state);

  void alignTrajectoriesCount(robot_trajectory::RobotTrajectoryPtr& trajectory_1,
                              robot_trajectory::RobotTrajectoryPtr& trajectory_2);

  void updateDualIntermediateState(const moveit::core::RobotState& leader_state,
                                    const moveit::core::RobotState& follower_state,
                                    planning_scene::PlanningScenePtr& start,
                                    planning_scene::PlanningScenePtr& end);

  const moveit::core::JointModelGroup* follow_jmg_;
  const moveit::core::JointModelGroup* leader_jmg_;
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
