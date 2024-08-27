/*********************************************************************
 * Software License Agreement (BSD License)
 *
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

/* Authors: Michael Goerner
   Desc:    Generator Stage for simple grasp poses
*/

#pragma once

#include <moveit/task_constructor/stages/generate_pose.h>

namespace moveit {
namespace task_constructor {
namespace stages {

using GroupPoseDict = std::map<std::string, geometry_msgs::PoseStamped>;
using GroupStringDict = std::map<std::string, std::string>;
using GroupVectorDict = std::map<std::string, std::vector<double>>;

class GenerateGraspPoseDual : public GeneratePose
{
public:
	GenerateGraspPoseDual(const std::string& name = "generate grasp pose for follower", const std::vector<std::string>& group_names = {"panda_1","panda_2"});

	void init(const core::RobotModelConstPtr& robot_model) override;
	void compute() override;

    void get_exploration_axis(Eigen::Vector3d& rotation_axis);

	void setEndEffector(const GroupStringDict& eefs) {setProperty("eefs", eefs); }
	void setObject(const GroupStringDict& objects) { setProperty("objects", objects); }
	void setAngleDelta(double delta) { setProperty("angle_delta", delta); }
	void setTargetDelta(const GroupVectorDict& target_deltas) { setProperty("target_deltas", target_deltas); }
	void setTargetOrient(const GroupVectorDict& target_orients) { setProperty("target_orients", target_orients); }

	void setPreGraspPose(const std::map<std::string, std::string>& pregrasps) { setProperty("pregrasps", pregrasps); }
	void setPreGraspPose(const std::map<std::string,moveit_msgs::RobotState>& pregrasps) { setProperty("pregrasps", pregrasps); }
	void setGraspPose(const std::string& grasp) { setProperty("grasp", grasp); }
	void setGraspPose(const moveit_msgs::RobotState& grasp) { setProperty("grasp", grasp); }

protected:
	void onNewSolution(const SolutionBase& s) override;
	std::vector<std::string> group_names_;
	std::string whole_body_group_;
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
