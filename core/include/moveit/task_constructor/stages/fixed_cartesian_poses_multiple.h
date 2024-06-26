/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Bielefeld University
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

/* Authors: Robert Haschke
   Desc:    Spawn fixed Cartesian poses
*/

#pragma once

#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/cost_queue.h>
#include <geometry_msgs/PoseStamped.h>

namespace moveit {
namespace task_constructor {
namespace stages {

class FixedCartesianPosesMultiple : public MonitoringGenerator
{
public:
	FixedCartesianPosesMultiple(const std::string& name = "generate poses");

	void reset() override;
	bool canCompute() const override;
	void compute() override;

	void addPosePair(const std::map<std::string, geometry_msgs::PoseStamped>& pose_pair);

protected:
	void onNewSolution(const SolutionBase& s) override;
	ordered<const SolutionBase*> upstream_solutions_;
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
