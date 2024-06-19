#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose_follower.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/compute_ik_multiple.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/fixed_cartesian_poses.h>
#include <moveit/task_constructor/stages/fixed_cartesian_poses_multiple.h>

#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gtest/gtest.h>
#include <math.h>
#include <tf2_eigen/tf2_eigen.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <moveit_msgs/MTCPlanGoal.h>

#include <moveit/task_constructor/cost_terms.h>

using namespace moveit::task_constructor;
using GroupPoseDict = std::map<std::string, geometry_msgs::PoseStamped>;
using GroupPoseMatrixDict = std::map<std::string, Eigen::Isometry3d>;
using GroupStringDict = std::map<std::string, std::string>;

ros::Publisher pub;
ros::Publisher marker_pub;
std::string hand_open_pose_ = "open";
std::string hand_close_pose_ = "close"; 

// in clip frame
std::vector<double> clip_size = {0.04, 0.04, 0.06};
double hold_x_offset = 0.03;
std::vector<double> leader_pre_clip = {-(clip_size[0]/2+hold_x_offset), -clip_size[1]/2, clip_size[2]/2}; 
std::vector<double> follower_pre_clip = {clip_size[0]/2+hold_x_offset, -clip_size[1]/2, clip_size[2]/2}; 
std::vector<double> leader_post_clip = {-0.05, 0, 0}; // {0.575, -0.081, 1.128}; //{0.552, 0.069, 1.128};
std::vector<double> follower_post_clip = {0.05, 0, 0}; // {0.552, 0.069, 1.128}; //{0.575, -0.081, 1.128};

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
	if (!psi.applyCollisionObject(object))
		throw std::runtime_error("Failed to spawn object: " + object.id);
}

void convertToEigen(const std::vector<double>& values, Eigen::Isometry3d& transform) {
    // This version is correct RPY
    Eigen::AngleAxisd roll_angle(values[3], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(values[4], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(values[5], Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> quaternion = roll_angle * pitch_angle * yaw_angle;

    transform = Eigen::Translation3d(values[0], values[1], values[2]) * quaternion;
}

geometry_msgs::PoseStamped createClipGoal(std::string goal_frame, const std::vector<double>& goal_translation_vector){
	geometry_msgs::PoseStamped goal_pose;
	goal_pose.pose.position.x = goal_translation_vector[0]; // 0.05
	goal_pose.pose.position.y = goal_translation_vector[1]; // 0.0
	goal_pose.pose.position.z = goal_translation_vector[2]; // 0.0
	// orientation from clip frame to robot ee frame
	goal_pose.pose.orientation.x = 0.0;
	goal_pose.pose.orientation.y = 0.9999997;
	goal_pose.pose.orientation.z = 0.0;
	goal_pose.pose.orientation.w = 0.0007963;
	// pose_stamped.pose = tf2::toMsg(object_pose);
	// lead_goal_pose.header.frame_id = "object";
	goal_pose.header.frame_id = goal_frame;

	return goal_pose;
}

moveit_msgs::CollisionObject createTable() {
	std::string table_name="table";
    std::string table_reference_frame = "world";
	std::vector<double> table_dimensions = {0.4, 0.5, 0.1};
	geometry_msgs::Pose pose;
    pose.position.x = 0.5;
    pose.position.y = -0.25;
    pose.position.z = 1.0;
    pose.orientation.w = 1.0;

	moveit_msgs::CollisionObject object;
	object.id = table_name;
	object.header.frame_id = table_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	object.primitives[0].dimensions = table_dimensions;
	pose.position.z -= 0.5 * table_dimensions[2];  // align surface with world
	object.primitive_poses.push_back(pose);
	return object;
}

moveit_msgs::CollisionObject createObject() {
	std::string object_name="object";
    std::string object_reference_frame = "world";
	std::vector<double> object_dimensions = {0.25, 0.02};
	geometry_msgs::Pose pose;
	pose.position.x = 0.5;
    pose.position.y = -0.25;
    pose.position.z = 1.0;
    pose.orientation.w = 1.0;

	moveit_msgs::CollisionObject object;
	object.id = object_name;
	object.header.frame_id = object_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	object.primitives[0].dimensions = object_dimensions;
	pose.position.z += 0.5 * object_dimensions[0];
	object.primitive_poses.push_back(pose);
	return object;
}

void appendFrameMarkers(ros::Publisher& marker_pub, geometry_msgs::PoseStamped& pose_stamped, std::string frame_name) {
	// Append frame markers
	std::deque<visualization_msgs::Marker> object_markers;
	// geometry_msgs::PoseStamped pose_stamped;
	// pose_stamped.pose = pose;
    // pose_stamped.header.frame_id = "world";
	rviz_marker_tools::appendFrame(object_markers, pose_stamped, 0.1, frame_name);

	visualization_msgs::MarkerArray marker_array_msg;
	int id = 0;
    for (const auto& marker : object_markers) {
    visualization_msgs::Marker mutable_marker = marker;  // Make a non-const copy
    mutable_marker.id = id++;
    marker_array_msg.markers.push_back(mutable_marker);
	}

	double timeout = 2.0;
	ros::WallTime start = ros::WallTime::now();
    ros::WallTime end = start;
    while ((end - start).toSec() < timeout && ros::ok()) {
        marker_pub.publish(marker_array_msg);
        ros::WallDuration(1.0).sleep();
        end = ros::WallTime::now();
    }
}

moveit_msgs::CollisionObject createCubeObject(ros::Publisher& marker_pub) {
	std::string object_name= "object";
    std::string object_reference_frame = "world";
	std::vector<double> object_dimensions = {0.1, 0.05, 0.03};
	std::vector<double> object_position_vector = {0.50702, -0.2, 1.285};
	std::vector<double> object_pose_vector = {0.50702, -0.2, 1.285, -M_PI, 0.0, 0.0};
	geometry_msgs::Pose pose;
	// pose.position.x = 0.50702;
    // pose.position.y = -0.2;
    // pose.position.z = 1.285;
    // pose.orientation.w = 1.0;
	Eigen::AngleAxisd gripper_default_rotation(-M_PI, Eigen::Vector3d::UnitX());

	// rotation of the object
	double rotation_angle = -M_PI*0.5; // additional rotation around Z axis
	Eigen::Vector3d rotation_axis = Eigen::Vector3d::UnitZ();
	Eigen::Isometry3d default_object_pose; // By default there should be -PI rotation around X axis
	convertToEigen(object_pose_vector, default_object_pose);
	Eigen::Isometry3d object_pose = default_object_pose * Eigen::AngleAxisd(rotation_angle, rotation_axis);

	pose = tf2::toMsg(object_pose);

	moveit_msgs::CollisionObject object;
	object.id = object_name;
	object.header.frame_id = object_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	object.primitives[0].dimensions = object_dimensions;
	// pose.position.z += 0.5 * object_dimensions[0];
	object.primitive_poses.push_back(pose);

	// // Append frame markers
	// std::deque<visualization_msgs::Marker> object_markers;

	    geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.pose = pose;
	// // pose_stamped.pose = tf2::toMsg(object_pose);
    pose_stamped.header.frame_id = "world";
	// rviz_marker_tools::appendFrame(object_markers, pose_stamped, 0.1, "object frame");

	// visualization_msgs::MarkerArray marker_array_msg;
	// int id = 0;
    // for (const auto& marker : object_markers) {
    // visualization_msgs::Marker mutable_marker = marker;  // Make a non-const copy
    // mutable_marker.id = id++;
    // marker_array_msg.markers.push_back(mutable_marker);
	// }

	// double timeout = 2.0;
	// ros::WallTime start = ros::WallTime::now();
    // ros::WallTime end = start;
    // while ((end - start).toSec() < timeout && ros::ok()) {
    //     marker_pub.publish(marker_array_msg);
    //     ros::WallDuration(1.0).sleep();
    //     end = ros::WallTime::now();
    // }

	appendFrameMarkers(marker_pub, pose_stamped, "object frame");

	return object;
}


void setupDemoScene() {
	// Add table and object to planning scene
	ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	spawnObject(psi, createTable());
	spawnObject(psi, createObject());
}

Task createTaskRaw(std::string& goal_frame_name) {
    std::string dual_arm_group = "dual_arm";
    std::string follow_arm_group = "panda_2"; // "left_arm"; //
    std::string lead_arm_group = "panda_1"; // "right_arm"; //
    std::string follow_hand_group = "hand_2"; // "left_hand"; //
    std::string lead_hand_group = "hand_1"; // "right_hand"; //

	Task t;
    t.stages()->setName("Pick");
    t.loadRobotModel();

	// TODO: @Kejia: change initial stage to stage after moving of the leader
	Stage* initial_stage = new stages::CurrentState("current state");
	t.add(std::unique_ptr<Stage>(initial_stage));

	// planner used for connect
	auto lead_pipeline = std::make_shared<solvers::PipelinePlanner>();
	lead_pipeline->setPlannerId("RRTConnectkConfigDefault");

	auto follow_pipeline = std::make_shared<solvers::PipelinePlanner>();
	follow_pipeline->setPlannerId("RRTConnectkConfigDefault");

	auto dual_pipeline = std::make_shared<solvers::PipelinePlanner>();
	dual_pipeline->setPlannerId("RRTConnectkConfigDefault");

	// // create Cartesian interpolation "planner" to be used in various stages
	// auto cartesian_interpolation = std::make_shared<solvers::CartesianPath>();
	// // create a joint-space interpolation "planner" to be used in various stages
	// auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

    // // Set task properties
	// // t.setProperty("group", dual_arm_group);
	// t.setProperty("group", follow_arm_group);
	// t.setProperty("eef", follow_hand_group); 
	// t.setProperty("hand", follow_hand_group); 
	// t.setProperty("ik_frame", "panda_1_link8"); 
	
	// transform from flange to gripper
    std::vector<double> grasp_frame_transform_vector = {0, 0, 0.1, 1.571, 0.785, 1.571};
    Eigen::Isometry3d grasp_frame_transform; 
    convertToEigen(grasp_frame_transform_vector, grasp_frame_transform);

	Stage* move_stage_ptr = nullptr;
	Stage* pre_move_stage_ptr = nullptr;
	{	
		auto grasp = std::make_unique<SerialContainer>("pick object");
		/****************************************************
  ---- *               Close leader hand                     *
		***************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand", lead_pipeline);
			stage->setGroup(lead_hand_group);
			stage->setGoal(hand_close_pose_);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Open follower hand                     *
		***************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("open hand", follow_pipeline);
			stage->setGroup(follow_hand_group);
			stage->setGoal(hand_open_pose_);

			pre_move_stage_ptr = stage.get();

			grasp->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Connect to next stage                 *
		***************************************************/
		{
		stages::Connect::GroupPlannerVector planners = {{lead_arm_group, lead_pipeline}, {follow_arm_group, follow_pipeline}};
		// {follow_hand_group, follow_pipeline}, {lead_hand_group, lead_pipeline}
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		connect->properties().set("max_distance", 1e-3); // max allowable distance between end and goal position
		
		grasp->insert(std::move(connect));
		// t.add(std::move(connect));
		}

		/****************************************************
  ---- *       Spawn IK on fixed pose for dual arm        *
		***************************************************/
		{
		/* Set fixed Pose*/
		geometry_msgs::PoseStamped lead_goal_pose;
		lead_goal_pose = createClipGoal(goal_frame_name, leader_pre_clip);
		appendFrameMarkers(marker_pub, lead_goal_pose, "leader_goal_frame");

		geometry_msgs::PoseStamped follow_goal_pose;
		follow_goal_pose = createClipGoal(goal_frame_name, follower_pre_clip);
		appendFrameMarkers(marker_pub, follow_goal_pose, "follower_goal_frame");

		// auto lead_fixed_pose = std::make_unique<stages::FixedCartesianPoses>("lead_fixed_pose");
		// lead_fixed_pose->addPose(lead_goal_pose);
		// lead_fixed_pose->setMonitoredStage(pre_move_stage_ptr);

		Eigen::Isometry3d ik_frame_1 = Eigen::Isometry3d::Identity();
    	ik_frame_1.translation().z() = 0.1034;

		// auto follow_fixed_pose = std::make_unique<stages::FixedCartesianPoses>("follow_fixed_pose");
		// follow_fixed_pose->addPose(follow_goal_pose);
		// follow_fixed_pose->setMonitoredStage(pre_move_stage_ptr);

		Eigen::Isometry3d ik_frame_2 = Eigen::Isometry3d::Identity();
    	ik_frame_2.translation().z() = 0.1034;

		auto dual_fixed_pose = std::make_unique<stages::FixedCartesianPosesMultiple>("dual_fixed_pose");
		GroupPoseDict pose_pairs = {{lead_arm_group, lead_goal_pose}, {follow_arm_group, follow_goal_pose}};
		dual_fixed_pose->addPosePair(pose_pairs);
		dual_fixed_pose->setMonitoredStage(pre_move_stage_ptr);

		std::vector<std::string> ik_groups = {lead_arm_group, follow_arm_group};
		GroupStringDict ik_endeffectors = {{lead_arm_group, lead_hand_group}, {follow_arm_group, follow_hand_group}};
		GroupStringDict ik_links = {{lead_arm_group, "panda_1_hand"}, {follow_arm_group, "panda_2_hand"}};
		// GroupStringDict ik_links = {{lead_arm_group, "right_arm_hand"}, {follow_arm_group, "left_arm_hand"}};
		GroupPoseMatrixDict ik_frames = {{lead_arm_group, ik_frame_1}, {follow_arm_group, ik_frame_2}};

		auto ik_wrapper = std::make_unique<stages::ComputeIKMultiple>("move IK", std::move(dual_fixed_pose), ik_groups, dual_arm_group);
		ik_wrapper->setSubGroups(ik_groups);
		ik_wrapper->setGroup(dual_arm_group);
		ik_wrapper->setEndEffector(ik_endeffectors);
		// ik_wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
		ik_wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_poses"});
		// ik_wrapper->properties().set("object", "object");
		ik_wrapper->setMaxIKSolutions(100);
		ik_wrapper->setMinSolutionDistance(1.0);

		ik_wrapper->setIKFrame(ik_frames, ik_links);

		auto cl_cost{ std::make_unique<cost::Clearance>() };
		cl_cost->cumulative = true;  // sum up pairwise distances between the robot to all objects?
		cl_cost->with_world = false;  // consider distance of the robot to world objects?
		ik_wrapper->setCostTerm(std::move(cl_cost));
			
		grasp->insert(std::move(ik_wrapper));

		// t.add(std::move(move_to_object));
		}

// 		/****************************************************
//   ---- *       Spawn IK on fixed pose for dual arm        *
// 		***************************************************/
// 		{
// 		/* Set fixed Pose*/
// 		geometry_msgs::PoseStamped lead_goal_pose;
// 		lead_goal_pose = createClipGoal(goal_frame_name, leader_post_clip);
// 		appendFrameMarkers(marker_pub, lead_goal_pose, "leader_goal_frame");

// 		geometry_msgs::PoseStamped follow_goal_pose;
// 		follow_goal_pose = createClipGoal(goal_frame_name, follower_post_clip);
// 		appendFrameMarkers(marker_pub, follow_goal_pose, "follower_goal_frame");

// 		// auto lead_fixed_pose = std::make_unique<stages::FixedCartesianPoses>("lead_fixed_pose");
// 		// lead_fixed_pose->addPose(lead_goal_pose);
// 		// lead_fixed_pose->setMonitoredStage(pre_move_stage_ptr);

// 		Eigen::Isometry3d ik_frame_1 = Eigen::Isometry3d::Identity();
//     	ik_frame_1.translation().z() = 0.1034;

// 		// auto follow_fixed_pose = std::make_unique<stages::FixedCartesianPoses>("follow_fixed_pose");
// 		// follow_fixed_pose->addPose(follow_goal_pose);
// 		// follow_fixed_pose->setMonitoredStage(pre_move_stage_ptr);

// 		Eigen::Isometry3d ik_frame_2 = Eigen::Isometry3d::Identity();
//     	ik_frame_2.translation().z() = 0.1034;

// 		auto post_lead = std::make_unique<stages::MoveTo>("post_grasp_lead", lead_pipeline);
// 		post_lead->setGroup(lead_arm_group);
// 		post_lead->setIKFrame(ik_frame_1, "panda_1_hand");
// 		post_lead->setGoal(lead_goal_pose);
// 		grasp->insert(std::move(post_lead));

// 		auto post_follow = std::make_unique<stages::MoveTo>("post_grasp_follow", follow_pipeline);
// 		post_follow->setGroup(follow_arm_group);
// 		post_follow->setIKFrame(ik_frame_2, "panda_2_hand");
// 		post_follow->setGoal(follow_goal_pose);
// 		grasp->insert(std::move(post_follow));


// 		// t.add(std::move(move_to_object));
// 		}


		/****************************************************
  ---- *               Move leder to object                  *
		***************************************************/
		// {
		
		// geometry_msgs::PoseStamped lead_goal_pose;
		// lead_goal_pose.pose.position.x = leader_goal_pose_vector[0]; // 0.05
		// lead_goal_pose.pose.position.y = leader_goal_pose_vector[1]; // 0.0
		// lead_goal_pose.pose.position.z = leader_goal_pose_vector[2]; // 0.0
		// // orientation from clip frame to robot ee frame
		// lead_goal_pose.pose.orientation.x = 0.0;
		// lead_goal_pose.pose.orientation.y = 0.9999997;
		// lead_goal_pose.pose.orientation.z = 0.0;
		// lead_goal_pose.pose.orientation.w = 0.0007963;
		// // pose_stamped.pose = tf2::toMsg(object_pose);
		// // lead_goal_pose.header.frame_id = "object";
		// lead_goal_pose.header.frame_id = goal_frame_name;
		// appendFrameMarkers(marker_pub, lead_goal_pose, "leader_goal_frame");

		// Eigen::Isometry3d ik_frame_2 = Eigen::Isometry3d::Identity();
    	// ik_frame_2.translation().z() = 0.1034;

		// /*Option 1: move directly to object*/
		// // auto move_to_object = std::make_unique<stages::MoveTo>("move to object", lead_pipeline);
		// // move_to_object->setGroup(lead_arm_group);
		// // move_to_object->setGoal(lead_goal_pose);
		// // move_to_object->setIKFrame(ik_frame_2, "panda_1_hand");
		// // move_stage_ptr = move_to_object.get();  // remember for monitoring follower pose generator
		// // grasp->insert(std::move(move_to_object));
		
		// /*Option 2: force multiple IK solutions given a certain pose*/
		// auto lead_fixed_pose = std::make_unique<stages::FixedCartesianPoses>("lead_fixed_pose");
		// lead_fixed_pose->addPose(lead_goal_pose);
		// lead_fixed_pose->setMonitoredStage(pre_move_stage_ptr);

		// auto ik_wrapper = std::make_unique<stages::ComputeIK>("move IK", std::move(lead_fixed_pose));
		// ik_wrapper->setGroup(lead_arm_group);
		// ik_wrapper->setEndEffector(lead_hand_group);
		// // ik_wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
		// ik_wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose" });
		// // ik_wrapper->properties().set("object", "object");
		// ik_wrapper->setMaxIKSolutions(30);
		// ik_wrapper->setMinSolutionDistance(1.0);

		// ik_wrapper->setIKFrame(ik_frame_2, "panda_1_hand");

		// auto cl_cost{ std::make_unique<cost::Clearance>() };
		// cl_cost->cumulative = true;  // sum up pairwise distances between the robot to all objects?
		// cl_cost->with_world = true;  // consider distance of the robot to world objects?
		// ik_wrapper->setCostTerm(std::move(cl_cost));
			
		// grasp->insert(std::move(ik_wrapper));

		// // t.add(std::move(move_to_object));
		// }

		/****************************************************
  ---- *               Connect to next stage                 *
		***************************************************/
		// {
		// stages::Connect::GroupPlannerVector planners = {{follow_arm_group, follow_pipeline}, {lead_arm_group, lead_pipeline}};
		// // {follow_hand_group, follow_pipeline}, {lead_hand_group, lead_pipeline}
		// auto connect = std::make_unique<stages::Connect>("connect", planners);
		// connect->properties().configureInitFrom(Stage::PARENT);
		// connect->properties().set("max_distance", 1e-3); // max allowable distance between end and goal position
		
		// grasp->insert(std::move(connect));
		// // t.add(std::move(connect));
		// }
	
		/****************************************************
  ---- *            Grasp POSE generator for follower     *
		***************************************************/
		// {
		// auto grasp_generator = std::make_unique<stages::GenerateGraspPoseFollow>("generate grasp pose");
		// grasp_generator->properties().set("group", follow_arm_group);
		// grasp_generator->setEndEffector(follow_hand_group);
		// grasp_generator->properties().set("hand", follow_hand_group);
		// // grasp_generator->properties().configureInitFrom(Stage::PARENT);
		// grasp_generator->properties().set("marker_ns", "grasp_pose");
		// grasp_generator->properties().set("explr_axis", "y");
		// grasp_generator->setAngleDelta(.2); // enumerate over angles from 0 to 6.4 (less then 2 PI)
		// grasp_generator->setPreGraspPose("open");
		// grasp_generator->setGraspPose("close");
		// // grasp_generator->setObject("object"); // object sets target pose frame
		// grasp_generator->setObject(goal_frame_name); // object sets target pose frame

		// // std::vector<double> grasp_target = {-0.05, 0.0, 0.0}; // target pose in object frame
		// // grasp_generator->setTarget(grasp_target);
		// grasp_generator->setTarget(follower_goal_pose_vector);
		// 		// grasp_generator->setEndEffector("hand");
		// grasp_generator->setMonitoredStage(move_stage_ptr);

		// auto ik_wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(grasp_generator));
		// ik_wrapper->setGroup(follow_arm_group);
		// ik_wrapper->setEndEffector(follow_hand_group);
		// // ik_wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
		// ik_wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose" });
		// // grasp->properties().set("eef", "hand");
		// // ik_wrapper->properties().set("object", "object");
		// ik_wrapper->setMaxIKSolutions(5);
		// ik_wrapper->setMinSolutionDistance(1.0);

		// // ik_wrapper->setIKFrame(grasp_frame_transform, "panda_link8");
		
		// Eigen::Isometry3d ik_frame = Eigen::Isometry3d::Identity();
		// // // T_FtoTCP_3d = np.array([[0.707, 0.707, 0, 0],
		// // //                     [-0.707, 0.707, 0, 0],
		// // //                     [0, 0, 1, 0.1034],
		// // //                     [0, 0, 0, 1]])
		// ik_frame.translation().z() = 0.1034;
		// // // TODO: should it be "panda_hand" or "panda_link8"?
		// ik_wrapper->setIKFrame(ik_frame, "panda_1_hand");
		
		// grasp->insert(std::move(ik_wrapper));
		// // t.add(std::move(ik_wrapper));
		// }

		/****************************************************
  ---- *               Close follower hand                     *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand", follow_pipeline);
			stage->setGroup(follow_hand_group);
			stage->setGoal(hand_close_pose_);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  ---- *                        Homing                        *
		 ***************************************************/
		// {
		// 	auto stage = std::make_unique<stages::MoveTo>("back home", dual_pipeline);
		// 	stage->setGroup(dual_arm_group);
		// 	stage->setGoal("home");
		// 	grasp->insert(std::move(stage));
		// }

	t.add(std::move(grasp));
	}
	
	return t;
}

Task createTaskHome() {
    std::string dual_arm_group = "dual_arm";
    std::string follow_arm_group = "panda_2"; // "left_arm"; //
    std::string lead_arm_group = "panda_1"; // "right_arm"; //
    std::string follow_hand_group = "hand_2"; // "left_hand"; //
    std::string lead_hand_group = "hand_1"; // "right_hand"; //

	Task t;
    t.stages()->setName("Pick");
    t.loadRobotModel();

	// TODO: @Kejia: change initial stage to stage after moving of the leader
	Stage* initial_stage = new stages::CurrentState("current state");
	t.add(std::unique_ptr<Stage>(initial_stage));

	// planner used for connect
	auto lead_pipeline = std::make_shared<solvers::PipelinePlanner>();
	lead_pipeline->setPlannerId("RRTConnectkConfigDefault");

	auto follow_pipeline = std::make_shared<solvers::PipelinePlanner>();
	follow_pipeline->setPlannerId("RRTConnectkConfigDefault");

	auto dual_pipeline = std::make_shared<solvers::PipelinePlanner>();
	dual_pipeline->setPlannerId("RRTConnectkConfigDefault");

	/****************************************************
  ---- *                        Homing                        *
	***************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("back home", dual_pipeline);
		stage->setGroup(dual_arm_group);
		stage->setGoal("home");
		t.add(std::move(stage));
	}
	return t;
}
// Subscriber callback function
// void receiveGoals(const moveit_msgs::MTCPlanGoal::ConstPtr& msg) {
//     // Log the received message
// 	// ROS_INFO("Receive leader goal id: %d", msg->goal_id);
//     ROS_INFO("Receive leader goal position: [%.2f, %.2f, %.2f]",
//              msg->robot1_goal_pose.pose.position.x, msg->robot1_goal_pose.pose.position.y, msg->robot1_goal_pose.pose.position.z);

// 	std::string goal_frame_name = msg->robot1_goal_pose.header.frame_id;
// 	std::vector<double> leader_goal_pose_vector = {msg->robot1_goal_pose.pose.position.x, msg->robot1_goal_pose.pose.position.y, msg->robot1_goal_pose.pose.position.z};
// 	std::vector<double> follower_goal_pose_vector = {msg->robot2_goal_pose.pose.position.x, msg->robot2_goal_pose.pose.position.y, msg->robot2_goal_pose.pose.position.z};
// 	auto task = createTaskRaw(goal_frame_name, leader_goal_pose_vector, follower_goal_pose_vector);

// 	std_msgs::Bool success;
// 	success.data = false;

// 		try {
// 		if (task.plan())
//             ROS_INFO("planning succeeded");
// 			task.introspection().publishSolution(*task.solutions().front());
// 				} catch (const InitStageException& ex) {
// 		std::cerr << "planning failed with exception" << std::endl << ex << task;
// 	}
// }


int main(int argc, char** argv) {
	Task task;

	ros::init(argc, argv, "mtc_tutorial");
	ros::NodeHandle nh;
		ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("interactive_robot_marray", 10);
	// run an asynchronous spinner to communicate with the move_group node and rviz
	ros::AsyncSpinner spinner(1);
	spinner.start();

    // setupDemoScene();
    ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	// spawnObject(psi, createCubeObject(marker_pub));
	
	std::vector<std::string> clip_names = {"clip7", "clip6", "clip9"}; //, "clip7", "clip6", "clip9"
	std::map<std::string, geometry_msgs::Pose> clip_poses = psi.getObjectPoses(clip_names);

	for (const auto& clip_id : clip_names) {
        const auto& it = clip_poses.find(clip_id); // Find pose for the clip name
        if (it != clip_poses.end()) {
            const geometry_msgs::Pose& pose = it->second;

			ROS_WARN("planning for clip %s started", clip_id.c_str());
			
			// append visual frames
			geometry_msgs::PoseStamped clip_pose_stamped;
			clip_pose_stamped.pose = pose;
			clip_pose_stamped.header.frame_id = "world";
			appendFrameMarkers(marker_pub, clip_pose_stamped, clip_id + "_frame");

			// motion plan to the clip
			std::string goal_frame_name = clip_id;
			
			task = createTaskRaw(goal_frame_name);
			try {
				if (task.plan())
					// ROS_WARN("planning for clip %s succeeded", clip_id.c_str());
					task.introspection().publishSolution(*task.solutions().front());

					ROS_WARN("Executing solution trajectory");
					moveit_msgs::MoveItErrorCodes execute_result;

					execute_result = task.execute(*task.solutions().front());
					// // If you want to inspect the goal message, use this instead:
					// actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>
					// execute("execute_task_solution", true); execute.waitForServer();
					// moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
					// task_->solutions().front()->toMsg(execute_goal.solution);
					// execute.sendGoalAndWait(execute_goal);
					// execute_result = execute.getResult()->error_code;

					if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
						ROS_ERROR_STREAM("Task execution failed and returned: " << execute_result.val);
					}
				} catch (const InitStageException& ex) {
					std::cerr << "planning failed with exception" << std::endl << ex << task;
				}
		}
	}

	task = createTaskHome();
	try {
		if (task.plan())
			// ROS_WARN("planning for clip %s succeeded", clip_id.c_str());
			task.introspection().publishSolution(*task.solutions().front());

			ROS_WARN("Executing solution trajectory");
			moveit_msgs::MoveItErrorCodes execute_result;

			execute_result = task.execute(*task.solutions().front());
			if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
				ROS_ERROR_STREAM("Task execution failed and returned: " << execute_result.val);
			}
		} catch (const InitStageException& ex) {
			std::cerr << "planning failed with exception" << std::endl << ex << task;
		}
		
    // task.printState();

    // Stage* generateGraspPoseStage = task.stages()->findChild("pick/grasp/compute ik");
    
    // if (generateGraspPoseStage) {
    // // Proceed with accessing solutions
    // // Iterate through solutions and retrieve costs
    // for (const auto& solution : generateGraspPoseStage->solutions()) {
    // double cost = solution->cost();
    // const std::string comment = solution->comment();
    // std::cout << "cost for angle " << comment << " is " << cost << std::endl;
    // }
    // } else {
    // // Handle the case where the stage is not found
    // std::cerr << "Error: 'generate grasp pose' stage not found." << std::endl;
    // }

	ros::waitForShutdown();  // keep alive for interactive inspection in rviz
	return 0;
}

