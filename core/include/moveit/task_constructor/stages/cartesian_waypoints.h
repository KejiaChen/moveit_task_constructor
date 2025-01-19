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
   Desc:    Move to joint-state or Cartesian goal pose
*/

#pragma once

#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/solvers/planner_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace moveit {
namespace task_constructor {
namespace stages {

class CartesianWaypointStage : public PropagatingEitherWay
{
public:
  CartesianWaypointStage(const std::string& name, const solvers::PlannerInterfacePtr& planner, const moveit::planning_interface::MoveGroupInterfacePtr& move_group);

  void init(const moveit::core::RobotModelConstPtr& robot_model) override;

  void setWaypoints(const std::vector<geometry_msgs::msg::Pose>& waypoints);
  void setGroup(const std::string& group);
  void setIKFrame(const geometry_msgs::msg::PoseStamped& ik_frame);
  void setIKFrame(const Eigen::Isometry3d& pose, const std::string& link);
  void setPathConstraints(const moveit_msgs::msg::Constraints& constraints);

protected:
  bool compute(const InterfaceState& state, planning_scene::PlanningScenePtr& scene, SubTrajectory& solution,
               Interface::Direction dir) override;

private:
  solvers::PlannerInterfacePtr planner_;
  std::vector<geometry_msgs::msg::Pose> waypoints_;
  std::string group_name_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  geometry_msgs::msg::PoseStamped ik_frame_;
  moveit_msgs::msg::Constraints path_constraints_;
};

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
