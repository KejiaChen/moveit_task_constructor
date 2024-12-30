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
#include <moveit_msgs/msg/constraints.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

namespace moveit {
namespace core {
class RobotState;
}
}  // namespace moveit
namespace moveit {
namespace task_constructor {
namespace stages {

/** Perform a Cartesian motion relative to some link */
using GroupPoseDict = std::map<std::string, geometry_msgs::msg::PoseStamped>;
using GroupStringDict = std::map<std::string, std::string>;
class MoveRelativeMultiple : public PropagatingEitherWay
{
public:
	struct PlannerIdTrajectoryPair
	{
		std::string planner_id;
		robot_trajectory::RobotTrajectoryConstPtr trajectory;
	};

    using GroupPlannerVector = std::vector<std::pair<std::string, solvers::PlannerInterfacePtr>>;
	MoveRelativeMultiple(const std::string& name = "move relative",
	             const GroupPlannerVector& planners = {});

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

	void setGroup(const std::vector<std::string>& groups) { setProperty("groups", groups); }
	/// setters for IK frame
	void setIKFrame(GroupPoseDict& poses) { setProperty("ik_frames", poses); }
	void setIKFrame(std::map<std::string, Eigen::Isometry3d>& poses, GroupStringDict& links);
	template <typename T>
	void setIKFrame(std::map<std::string,T>& p, GroupStringDict& links) {
		std::map<std::string, Eigen::Isometry3d> poses;
		poses = p;
		setIKFrame(poses, links);
	}
	// void setIKFrame(const std::string& link) { setIKFrame(Eigen::Isometry3d::Identity(), link); }

	/// set minimum / maximum distance to move
	void setMinDistance(double distance) { setProperty("min_distance", distance); }
	void setMaxDistance(double distance) { setProperty("max_distance", distance); }
	void setMinMaxDistance(double min_distance, double max_distance) {
		setProperty("min_distance", min_distance);
		setProperty("max_distance", max_distance);
	}

	void setPathConstraints(moveit_msgs::msg::Constraints path_constraints) {
		setProperty("path_constraints", std::move(path_constraints));
	}

	/// perform twist motion on specified link
	void setDirection(const geometry_msgs::msg::TwistStamped& twist) { setProperty("direction", twist); }
	/// translate link along given direction
	void setDirection(const geometry_msgs::msg::Vector3Stamped& direction) { setProperty("direction", direction); }
	/// move specified joint variables by given amount
	void setDirection(const std::map<std::string, double>& joint_deltas) { setProperty("direction", joint_deltas); }

protected:
	// return false if trajectory shouldn't be stored
	bool compute(const InterfaceState& state, planning_scene::PlanningScenePtr& scene, SubTrajectory& trajectory,
	             Interface::Direction dir) override;
	
	SubTrajectoryPtr merge(const std::vector<PlannerIdTrajectoryPair>& sub_trajectories,
                           const planning_scene::PlanningSceneConstPtr& current_scene,
                           const moveit::core::RobotState& initial_state);

protected:
	GroupPlannerVector planner_;
	moveit::core::JointModelGroupPtr merged_jmg_;
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
