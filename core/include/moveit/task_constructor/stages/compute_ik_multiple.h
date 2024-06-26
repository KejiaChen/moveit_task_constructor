/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  Copyright (c) 2017, Hamburg University
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
/* Authors: Robert Haschke, Michael Goerner */

#pragma once

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/cost_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>

namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotState);
MOVEIT_CLASS_FORWARD(JointModelGroup);
}  // namespace core
}  // namespace moveit

namespace moveit {
namespace task_constructor {
namespace stages {

/** Wrapper for any pose generator stage to compute IK poses for a Cartesian pose.
 *
 * The wrapper reads a target_pose from the interface state of solutions provided
 * by the wrapped stage. This Cartesian pose (PoseStamped msg) is used as a goal
 * pose for inverse kinematics.
 *
 * Usually, the end effector's parent link or the group's tip link is used as
 * the IK frame, which should be moved to the goal frame. However, any other
 * IK frame can be defined (which is linked to the tip of the group).
 *
 * Properties of the internally received InterfaceState can be forwarded to the
 * newly generated, externally exposed InterfaceState.
 */

using GroupPoseDict = std::map<std::string, geometry_msgs::PoseStamped>;
using GroupStringDict = std::map<std::string, std::string>;

class ComputeIKMultiple : public WrapperBase
{
public:
	ComputeIKMultiple(const std::string& name = "IK", Stage::pointer&& child = Stage::pointer(), const std::vector<std::string>& group_names = {"panda_1","panda_2"}, const std::string& whole_body_group = "dual_arm");

    // using GroupPlannerVector = std::vector<std::pair<std::string, solvers::PlannerInterfacePtr> >;
    
    // using GroupEEVector = std::vector<std::pair<std::string, std::string> >;
    // using GroupDefaultPoseVector = std::vector<std::pair<std::string, std::string> >;
	// using GroupPoseDict = std::map<std::string, geometry_msgs::PoseStamped>;
	// using GroupStringDict = std::map<std::string, std::string>;

	void reset() override;
	void init(const core::RobotModelConstPtr& robot_model) override;
	void onNewSolution(const SolutionBase& s) override;

	bool canCompute() const override;

	void compute() override;

	void setEndEffector(const GroupStringDict& eefs) {setProperty("eefs", eefs); }
	void setSubGroups(const std::vector<std::string>& groups) { setProperty("groups", groups); }
	void setGroup(const std::string& group) { setProperty("group", group); }

	/// setters for IK frame
	void setIKFrame(GroupPoseDict& poses) { setProperty("ik_frames", poses); }
	void setIKFrame(std::map<std::string, Eigen::Isometry3d>& poses, GroupStringDict& links);
	template <typename T>
	void setIKFrame(std::map<std::string,T>& p, GroupStringDict& links) {
		std::map<std::string, Eigen::Isometry3d> poses;
		poses = p;
		setIKFrame(poses, links);
	}
	// void setIKFrame(const GroupStringDict& link) { 
	// 	std::map<std::string, Eigen::Isometry3d> poses;

	// 	// Iterate over each element in the link map
	// 	for (const auto& pair : link) {
	// 		// Set the value in the pose map to Eigen::Isometry3d::Identity()
	// 		poses[pair.first] = Eigen::Isometry3d::Identity();
	// 	}
	// 	setIKFrame(Eigen::Isometry3d::Identity(), link); }

	/** setters for target pose property
	 *
	 * This property should almost always be set in the InterfaceState sent by the child.
	 * If possible, avoid setting it manually.
	 */
	void setTargetPose(GroupPoseDict& poses) { setProperty("target_poses", poses); }
	void setTargetPose(std::map<std::string, Eigen::Isometry3d>& poses, GroupStringDict& frames);
	template <typename T>
	void setTargetPose(std::map<std::string,T>& p, GroupStringDict& frames) {
		std::map<std::string, Eigen::Isometry3d> poses;
		poses = p;
		setTargetPose(poses, frames);
	}

	void setMaxIKSolutions(uint32_t n) { setProperty("max_ik_solutions", n); }
	void setIgnoreCollisions(bool flag) { setProperty("ignore_collisions", flag); }
	void setMinSolutionDistance(double distance) { setProperty("min_solution_distance", distance); }

protected:
	ordered<const SolutionBase*> upstream_solutions_;
    std::vector<std::string> group_names_;
	std::string whole_body_group_;
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
