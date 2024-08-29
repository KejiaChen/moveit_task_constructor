/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld + Hamburg University
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

#include <moveit/task_constructor/stages/generate_grasp_pose_dual.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace moveit {
namespace task_constructor {
namespace stages {

GenerateGraspPoseDual::GenerateGraspPoseDual(const std::string& name, const std::vector<std::string>& group_names) : GeneratePose(name), group_names_(std::move(group_names)){
	auto& p = properties();
	p.declare<GroupStringDict>("eefs", "vector of names of end-effector group");
	p.declare<GroupStringDict>("objects");
    p.declare<GroupVectorDict>("target_deltas", "relative position of target pose in object frame"); //{1, 0, 0},
	p.declare<GroupVectorDict>("target_orients", "relative orientation of target pose in object frame"); //{1, 0, 0},
	p.declare<double>("angle_delta", 0.1, "angular steps (rad)");
    p.declare<std::string>("explr_axis", "x", "axis around which rotation is performed to spawn various grasps");

	p.declare<boost::any>("pregrasps", "pregrasp EE posture for each group");
	p.declare<boost::any>("grasp", "grasp EE posture");
	p.declare<std::string>("generate_group"); // generate pose only for selected group.
	p.declare<std::string>("planning_frame"); // planning frame, by default the clip frame
}

static void applyPreGrasp(moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
                          const Property& diff_property, std::string group_name) {
	try {
		// try named joint pose
		const std::map<std::string, std::string>& string_map = boost::any_cast<const std::map<std::string, std::string>&>(diff_property.value());
		const std::string& diff_state_name{ boost::any_cast<std::string>(string_map.at(group_name)) };
		if (!state.setToDefaultValues(jmg, diff_state_name)) {
			throw moveit::Exception{ "unknown state '" + diff_state_name + "'" };
		}
		return;
	} catch (const boost::bad_any_cast&) {
	}

	try {
		// try RobotState
		const std::map<std::string, moveit_msgs::RobotState>& msg_map = boost::any_cast<const std::map<std::string, moveit_msgs::RobotState>&>(diff_property.value());
		const moveit_msgs::RobotState& robot_state_msg = boost::any_cast<moveit_msgs::RobotState>(msg_map.at(group_name));
		if (!robot_state_msg.is_diff)
			throw moveit::Exception{ "RobotState message must be a diff" };
		const auto& accepted = jmg->getJointModelNames();
		for (const auto& joint_name_list :
		     { robot_state_msg.joint_state.name, robot_state_msg.multi_dof_joint_state.joint_names })
			for (const auto& name : joint_name_list)
				if (std::find(accepted.cbegin(), accepted.cend(), name) == accepted.cend())
					throw moveit::Exception("joint '" + name + "' is not part of group '" + jmg->getName() + "'");
		robotStateMsgToRobotState(robot_state_msg, state);
		return;
	} catch (const boost::bad_any_cast&) {
	}

	throw moveit::Exception{ "no named pose or RobotState message" };
}

bool validateEEF(const PropertyMap& props, const moveit::core::RobotModelConstPtr& robot_model,
                 const moveit::core::JointModelGroup*& jmg, std::string* msg, std::string group_name) {
	try {
		const GroupStringDict& eefs = props.get<GroupStringDict>("eefs");
		const std::string& eef = eefs.at(group_name);
			if (!robot_model->hasEndEffector(eef)) {
				if (msg)
					*msg = "Unknown end effector: " + eef;
				return false;
			} else
				jmg = robot_model->getEndEffector(eef);
	} catch (const Property::undefined&) {
	}
	return true;
}

bool validaEEFpose(const PropertyMap& props, const moveit::core::RobotModelConstPtr& robot_model,
                  const moveit::core::JointModelGroup*& jmg, std::string* msg, std::string group_name) {
	moveit::core::RobotState test_state{robot_model};
	try {
		applyPreGrasp(test_state, jmg, props.property("pregrasps"), group_name);
	} catch (const moveit::Exception& e) {
		if (msg)
				*msg = "invalid pregrasp for group: " + group_name;
			return false;
	}
	return true;
}

void GenerateGraspPoseDual::init(const core::RobotModelConstPtr& robot_model) {
	InitStageException errors;
	try {
		GeneratePose::init(robot_model);
	} catch (InitStageException& e) {
		errors.append(e);
	}

	const auto& props = properties();

	// check angle_delta
	if (props.get<double>("angle_delta") == 0.)
		errors.push_back(*this, "angle_delta must be non-zero");

	// check availability of objects
	props.get<GroupStringDict>("objects");

	const moveit::core::JointModelGroup* eef_jmg = nullptr;
	const moveit::core::JointModelGroup* jmg = nullptr;
	std::string msg;

	// Validate one robot group at a time
	for (auto& group_name : group_names_) {
		// check availability of eefs
		if (!validateEEF(props, robot_model, eef_jmg, &msg, group_name))
			errors.push_back(*this, msg);
		if (!validaEEFpose(props, robot_model, eef_jmg, &msg, group_name))
			errors.push_back(*this, msg);
	}

	if (errors)
		throw errors;
}

void GenerateGraspPoseDual::onNewSolution(const SolutionBase& s) {
	planning_scene::PlanningSceneConstPtr scene = s.end()->scene();

	const auto& props = properties();
	for (auto& object : props.get<GroupStringDict>("objects")) {
		if (!scene->knowsFrameTransform(object.second)) {
			const std::string msg = "object '" + object.second + "' not in scene";
			spawn(InterfaceState{ scene }, SubTrajectory::failure(msg));
			return;
		}
	}

	upstream_solutions_.push(&s);
}

void GenerateGraspPoseDual::compute() {
	if (upstream_solutions_.empty())
		return;
	planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

	const auto& props = properties();
	const GroupStringDict& eefs = props.get<GroupStringDict>("eefs");
	const GroupVectorDict& target_deltas = props.get<GroupVectorDict>("target_deltas");
	const GroupVectorDict& target_orients = props.get<GroupVectorDict>("target_orients");

	moveit::core::RobotState& robot_state = scene->getCurrentStateNonConst();
	double current_angle = 0.0;
	std::string generate_group = props.get<std::string>("generate_group");

	for (auto& group_name : group_names_) {
		// set end effector pose
		const std::string& eef = eefs.at(group_name);
		const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getEndEffector(eef);

		// moveit::core::RobotState& robot_state = scene->getCurrentStateNonConst();
		try {
			applyPreGrasp(robot_state, jmg, props.property("pregrasps"), group_name);
		} catch (const moveit::Exception& e) {
			spawn(InterfaceState{ scene }, SubTrajectory::failure(std::string{ "invalid pregrasp: " } + e.what()));
			return;
		}
	}
	
	// iterate over one robot group
	// TODO@KejiaChen: iterate for multiple robot groups

	while (current_angle < 2. * M_PI && current_angle > -2. * M_PI) {
		std::map<std::string, geometry_msgs::PoseStamped> target_poses; //TODO@KejiaChen: how to get keys?
		for (auto& group_name : group_names_) {
			// set end effector pose
			const std::string& eef = eefs.at(group_name);
			const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getEndEffector(eef);

			// moveit::core::RobotState& robot_state = scene->getCurrentStateNonConst();
			// try {
			// 	applyPreGrasp(robot_state, jmg, props.property("pregrasps"));
			// } catch (const moveit::Exception& e) {
			// 	spawn(InterfaceState{ scene }, SubTrajectory::failure(std::string{ "invalid pregrasp: " } + e.what()));
			// 	return;
			// }

			// set target pose
			geometry_msgs::PoseStamped raw_pose_msg;
			// By default, target pose is set to the origin of object frame, and the position is trasnlated by a delta
			raw_pose_msg.header.frame_id = props.get<GroupStringDict>("objects").at(group_name);

			// default goal pose
			raw_pose_msg.pose.position.x = target_deltas.at(group_name)[0];
			raw_pose_msg.pose.position.y = target_deltas.at(group_name)[1];
			raw_pose_msg.pose.position.z = target_deltas.at(group_name)[2];
			raw_pose_msg.pose.orientation.x = target_orients.at(group_name)[0]; // 0.0;
			raw_pose_msg.pose.orientation.y = target_orients.at(group_name)[1]; // 0.9999997;
			raw_pose_msg.pose.orientation.z = target_orients.at(group_name)[2]; // 0.0;
			raw_pose_msg.pose.orientation.w = target_orients.at(group_name)[3]; // 0.0007963;

			Eigen::Isometry3d raw_pose;
			tf2::fromMsg(raw_pose_msg.pose, raw_pose);
			Eigen::Vector3d raw_position = raw_pose.translation();
			ROS_WARN_STREAM("Raw position: " << raw_position.transpose());
			Eigen::Matrix3d raw_orientation = raw_pose.rotation();
			Eigen::Quaterniond raw_orientation_quaternion(raw_orientation);
			ROS_WARN_STREAM("Raw orientation: " 
							<< "x: " << raw_orientation_quaternion.x() 
							<< ", y: " << raw_orientation_quaternion.y() 
							<< ", z: " << raw_orientation_quaternion.z() 
							<< ", w: " << raw_orientation_quaternion.w());

			geometry_msgs::PoseStamped transformed_pose_msg;
			// transform goal pose into planning frame
			if (raw_pose_msg.header.frame_id != props.get<std::string>("planning_frame")) {
				const Eigen::Isometry3d& planning_frame = scene->getFrameTransform(props.get<std::string>("planning_frame"));  // valid isometry by contract
				// ROS_WARN_STREAM("planning frame: " << planning_frame.matrix());
				const Eigen::Isometry3d& target_frame = scene->getFrameTransform(raw_pose_msg.header.frame_id);    // valid isometry by contract
				// ROS_WARN_STREAM("target frame: " << target_frame.matrix());
				Eigen::Isometry3d transform = planning_frame.inverse()*target_frame;
				// ROS_WARN_STREAM("transform: " << transform.matrix());
				Eigen::Vector3d transformed_position = transform*raw_position;
				transformed_pose_msg.pose.position = tf2::toMsg(transformed_position);
				Eigen::Matrix3d transformed_orientation = transform.linear()*raw_orientation;
				Eigen::Quaterniond transformed_orientation_quaternion(transformed_orientation);
				transformed_pose_msg.pose.orientation = tf2::toMsg(transformed_orientation_quaternion);
				transformed_pose_msg.header.frame_id = props.get<std::string>("planning_frame");
				// ROS_WARN_STREAM("transformed pose: " << transformed_position.transpose());
			} else {
				transformed_pose_msg = raw_pose_msg;
			}

			geometry_msgs::PoseStamped target_pose_msg;
			target_pose_msg.header.frame_id = props.get<std::string>("planning_frame");
			if (group_name == generate_group){
				// iterate over genrate group
				Eigen::Vector3d rotation_axis;
				get_exploration_axis(rotation_axis);
				Eigen::Isometry3d default_target_pose(Eigen::Translation3d(transformed_pose_msg.pose.position.x, 
																	   transformed_pose_msg.pose.position.y, 
																	   transformed_pose_msg.pose.position.z)*
												  Eigen::Quaterniond(transformed_pose_msg.pose.orientation.w, 
																	transformed_pose_msg.pose.orientation.x, 
																	transformed_pose_msg.pose.orientation.y, 
																	transformed_pose_msg.pose.orientation.z));
				Eigen::Isometry3d target_pose = default_target_pose * Eigen::AngleAxisd(current_angle, rotation_axis);
				current_angle += props.get<double>("angle_delta");
				target_pose_msg.pose = tf2::toMsg(target_pose);
			} else {
				target_pose_msg = transformed_pose_msg;
				
			}

			target_poses[group_name] = target_pose_msg;
		}
		// loop end here for single joint group

		InterfaceState state(scene); // Interface state for IK solver
		state.properties().set("target_poses", target_poses);
		props.exposeTo(state.properties(), { "pregrasps", "grasp" });
		ROS_WARN_STREAM("Target pose: " << target_poses[generate_group].pose);

		SubTrajectory trajectory;
		trajectory.setCost(0.0);
		trajectory.setComment(std::to_string(current_angle));

		// add frame at target pose
		for (auto& group_name : group_names_) {
			rviz_marker_tools::appendFrame(trajectory.markers(), target_poses[group_name], 0.1, "grasp frame");
		}

		ROS_WARN_STREAM("GenerateGraspPoseDual: spawn" << current_angle << "for group: " << generate_group);
		spawn(std::move(state), std::move(trajectory));
	}
}

void GenerateGraspPoseDual::get_exploration_axis(Eigen::Vector3d& rotation_axis) {
    const auto& props = properties();
    std::string axis = props.get<std::string>("explr_axis");

    // Convert axis string to lowercase
    std::transform(axis.begin(), axis.end(), axis.begin(), ::tolower);
    switch(axis[0]){
        case 'x':
            rotation_axis = Eigen::Vector3d::UnitX();
            break;
        case 'y':
            rotation_axis = Eigen::Vector3d::UnitY();
            break;
        case 'z':
            rotation_axis = Eigen::Vector3d::UnitZ();
            break;
        case '0':
            rotation_axis = Eigen::Vector3d::Zero();
        default:
            throw std::runtime_error("Exploration axis must be one of x, y, or z");
    }
    // if (axis == "x") {
    //     rotation_axis = Eigen::Vector3d::UnitX();
    // } else if (axis == "y") {
    //     rotation_axis = Eigen::Vector3d::UnitY();
    // } else if (axis == "z") {
    //     rotation_axis = Eigen::Vector3d::UnitZ();
    // } else {
    //     throw std::runtime_error("exploration axis must be one of x, y, or z");
    // }
}

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
