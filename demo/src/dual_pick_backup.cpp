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
#include <moveit/task_constructor/stages/move_to.h>

#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gtest/gtest.h>
#include <math.h>
#include <tf2_eigen/tf2_eigen.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <moveit_msgs/MTCPlanGoal.h>

using namespace moveit::task_constructor;

ros::Publisher pub;
ros::Publisher marker_pub;
std::string hand_open_pose_ = "open";
std::string hand_close_pose_ = "close"; 

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
	// TODO: append in object frame does not work
	// pose_stamped.header.frame_id = "object";

	// append in world frame
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

Task createTaskRaw(std::string& goal_frame_name, std::vector<double>& leader_goal_pose_vector, std::vector<double>& follower_goal_pose_vector) {
    std::string dual_arm_group = "dual_arm";
    std::string follow_arm_group = "panda_1";
    std::string lead_arm_group = "panda_2";
    std::string follow_hand_group = "hand_1";
    std::string lead_hand_group = "hand_2";

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
			grasp->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Move leder to object                  *
		***************************************************/
		{
		auto move_to_object = std::make_unique<stages::MoveTo>("move to object", lead_pipeline);
		move_to_object->setGroup(lead_arm_group);
		geometry_msgs::PoseStamped lead_goal_pose;
		lead_goal_pose.pose.position.x = leader_goal_pose_vector[0]; // 0.05
		lead_goal_pose.pose.position.y = leader_goal_pose_vector[1]; // 0.0
		lead_goal_pose.pose.position.z = leader_goal_pose_vector[2]; // 0.0
		// orientation from clip frame to robot ee frame
		lead_goal_pose.pose.orientation.x = 0.0;
		lead_goal_pose.pose.orientation.y = 0.9999997;
		lead_goal_pose.pose.orientation.z = 0.0;
		lead_goal_pose.pose.orientation.w = 0.0007963;
		// pose_stamped.pose = tf2::toMsg(object_pose);
		// lead_goal_pose.header.frame_id = "object";
		lead_goal_pose.header.frame_id = goal_frame_name;

		appendFrameMarkers(marker_pub, lead_goal_pose, "leader_goal_frame");

		move_to_object->setGoal(lead_goal_pose);
		Eigen::Isometry3d ik_frame_2 = Eigen::Isometry3d::Identity();
    	ik_frame_2.translation().z() = 0.1034;
		move_to_object->setIKFrame(ik_frame_2, "panda_2_hand");

		move_stage_ptr = move_to_object.get();  // remember for monitoring follower pose generator

		grasp->insert(std::move(move_to_object));
		// t.add(std::move(move_to_object));
		}

		/****************************************************
  ---- *               Connect to next stage                 *
		***************************************************/
		{
		stages::Connect::GroupPlannerVector planners = {{follow_arm_group, follow_pipeline}, {lead_arm_group, lead_pipeline}, {follow_hand_group, follow_pipeline}, {lead_hand_group, lead_pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		connect->properties().set("max_distance", 1e-3); // max allowable distance between end and goal position

		grasp->insert(std::move(connect));
		// t.add(std::move(connect));
		}
	
		/****************************************************
  ---- *            Grasp POSE generator for follower     *
		***************************************************/
		{
		auto grasp_generator = std::make_unique<stages::GenerateGraspPoseFollow>("generate grasp pose");
		grasp_generator->properties().set("group", follow_arm_group);
		grasp_generator->setEndEffector(follow_hand_group);
		grasp_generator->properties().set("hand", follow_hand_group);
		// grasp_generator->properties().configureInitFrom(Stage::PARENT);
		grasp_generator->properties().set("marker_ns", "grasp_pose");
		grasp_generator->properties().set("explr_axis", "y");
		grasp_generator->setAngleDelta(.2); // enumerate over angles from 0 to 6.4 (less then 2 PI)
		grasp_generator->setPreGraspPose("open");
		grasp_generator->setGraspPose("close");
		// grasp_generator->setObject("object"); // object sets target pose frame
		grasp_generator->setObject(goal_frame_name); // object sets target pose frame

		// std::vector<double> grasp_target = {-0.05, 0.0, 0.0}; // target pose in object frame
		// grasp_generator->setTarget(grasp_target);
		grasp_generator->setTarget(follower_goal_pose_vector);
		// grasp_generator->setEndEffector("hand");
		grasp_generator->setMonitoredStage(move_stage_ptr);

		auto ik_wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(grasp_generator));
		ik_wrapper->setGroup(follow_arm_group);
		ik_wrapper->setEndEffector(follow_hand_group);
		// ik_wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
		ik_wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose" });
		// grasp->properties().set("eef", "hand");
		// ik_wrapper->properties().set("object", "object");
		ik_wrapper->setMaxIKSolutions(5);
		ik_wrapper->setMinSolutionDistance(1.0);

		// ik_wrapper->setIKFrame(grasp_frame_transform, "panda_link8");
		
		Eigen::Isometry3d ik_frame = Eigen::Isometry3d::Identity();
		// // T_FtoTCP_3d = np.array([[0.707, 0.707, 0, 0],
		// //                     [-0.707, 0.707, 0, 0],
		// //                     [0, 0, 1, 0.1034],
		// //                     [0, 0, 0, 1]])
		ik_frame.translation().z() = 0.1034;
		// // TODO: should it be "panda_hand" or "panda_link8"?
		ik_wrapper->setIKFrame(ik_frame, "panda_1_hand");
		
		grasp->insert(std::move(ik_wrapper));
		// t.add(std::move(ik_wrapper));
		}

		/****************************************************
  ---- *               Close follower hand                     *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand", follow_pipeline);
			stage->setGroup(follow_hand_group);
			stage->setGoal(hand_close_pose_);
			grasp->insert(std::move(stage));
		}

		t.add(std::move(grasp));
	}
	
	return t;
}

// Subscriber callback function
void receiveGoals(const moveit_msgs::MTCPlanGoal::ConstPtr& msg) {
    // Log the received message
	// ROS_INFO("Receive leader goal id: %d", msg->goal_id);
    ROS_INFO("Receive leader goal position: [%.2f, %.2f, %.2f]",
             msg->robot1_goal_pose.pose.position.x, msg->robot1_goal_pose.pose.position.y, msg->robot1_goal_pose.pose.position.z);

	std::string goal_frame_name = msg->robot1_goal_pose.header.frame_id;
	std::vector<double> leader_goal_pose_vector = {msg->robot1_goal_pose.pose.position.x, msg->robot1_goal_pose.pose.position.y, msg->robot1_goal_pose.pose.position.z};
	std::vector<double> follower_goal_pose_vector = {msg->robot2_goal_pose.pose.position.x, msg->robot2_goal_pose.pose.position.y, msg->robot2_goal_pose.pose.position.z};
	auto task = createTaskRaw(goal_frame_name, leader_goal_pose_vector, follower_goal_pose_vector);
	
	std_msgs::Bool success;
	success.data = false;

	try {
		if (task.plan())
            ROS_INFO("planning succeeded");
			task.introspection().publishSolution(*task.solutions().front());
			
			ROS_INFO("Executing solution trajectory");
			moveit_msgs::MoveItErrorCodes execute_result;

			execute_result = task.execute(*task.solutions().front());

			if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
				ROS_ERROR_STREAM("Task execution failed and returned: " << execute_result.val);
			} 

			success.data = true;

	} catch (const InitStageException& ex) {
		std::cerr << "planning failed with exception" << std::endl << ex << task;
	}

	pub.publish(success);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");
	ros::NodeHandle nh;
	// std::string clip_id;
	// if (!nh.getParam("clip_id", clip_id)){
    //     ROS_WARN("Failed to get parameter 'clip_id', using default value clip7");
    //     clip_id = "clip7";
    // }

	ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("interactive_robot_marray", 10);
	// run an asynchronous spinner to communicate with the move_group node and rviz
	// ros::AsyncSpinner spinner(1);
	// spinner.start();

	/*load parameters*/
	constexpr char LOGNAME[] = "moveit_task_constructor_demo";
	ROS_INFO_NAMED(LOGNAME, "Loading task parameters");

    // setupDemoScene();
    ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	// spawnObject(psi, createCubeObject(marker_pub));
	
	std::map<std::string, moveit_msgs::CollisionObject> objects = psi.getObjects();
    // Iterate over the map of objects and print their information
    ROS_INFO("List of known objects:");
    for (const auto& object_pair : objects) {
        const std::string& object_id = object_pair.first;
        const moveit_msgs::CollisionObject& object_msg = object_pair.second;

        ROS_INFO("- Object ID: %s", object_id.c_str());
        // Print other information about the object, such as pose, geometry, etc.
        // You can access object_msg fields like object_msg.id, object_msg.primitive_poses, etc.
    }

	// // listening on goal positions
	// ros::Publisher pub =  nh.advertise<std_msgs::Bool>("/mtc_back2front_chatter", 10);
	
	// ROS_INFO("start subscription");
	// ros::Subscriber sub = nh.subscribe("/mtc_front2back_chatter", 10, &receiveGoals);

	// "clip7", "clip6", "clip9"
	std::vector<std::string> clip_names = {"clip7"};
	std::map<std::string, geometry_msgs::Pose> clip_poses = psi.getObjectPoses(clip_names);

	// for (auto it = clip_poses.begin(); it != clip_poses.end(); ++it) {
    //     const std::string& clip_id = it->first;
    //     const geometry_msgs::Pose& pose = it->second;

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
			std::vector<double> leader_goal_pose_vector = {-0.05, 0, 0}; // {0.575, -0.081, 1.128}; //{0.552, 0.069, 1.128};
			std::vector<double> follower_goal_pose_vector = {0.05, 0, 0}; // {0.552, 0.069, 1.128}; //{0.575, -0.081, 1.128};
			auto task = createTaskRaw(goal_frame_name, leader_goal_pose_vector, follower_goal_pose_vector);

			try {
				if (task.plan())
					ROS_WARN("planning for clip %s succeeded", clip_id.c_str());
					// task.introspection().publishSolution(*task.solutions().front());
					
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
		}else{
			ROS_ERROR("Pose of %s not found", clip_id.c_str());
		}
	}


	// if (clip_poses.find(clip_id) != clip_poses.end()) {
    //     // Retrieve the pose of "object1" from the map
	// 	geometry_msgs::PoseStamped clip_pose_stamped;
    //     clip_pose_stamped.pose = clip_poses[clip_id];
	// 	clip_pose_stamped.header.frame_id = "world";
	// 	appendFrameMarkers(marker_pub, clip_pose_stamped, clip_id + "_frame");        
    // } else {
    //     ROS_ERROR("Pose of %s not found", clip_id.c_str());
    // }

	// std::string goal_frame_name = clip_id;
	// std::vector<double> leader_goal_pose_vector = {-0.05, 0, 0}; // {0.575, -0.081, 1.128}; //{0.552, 0.069, 1.128};
	// std::vector<double> follower_goal_pose_vector = {0.05, 0, 0}; // {0.552, 0.069, 1.128}; //{0.575, -0.081, 1.128};
	// auto task = createTaskRaw(goal_frame_name, leader_goal_pose_vector, follower_goal_pose_vector);
	// try {
	// 	if (task.plan())
    //         ROS_INFO("planning succeeded");
	// 		task.introspection().publishSolution(*task.solutions().front());
			
	// 		ROS_INFO("Executing solution trajectory");
	// 		moveit_msgs::MoveItErrorCodes execute_result;

	// 		execute_result = task.execute(*task.solutions().front());
	// 		// // If you want to inspect the goal message, use this instead:
	// 		// actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>
	// 		// execute("execute_task_solution", true); execute.waitForServer();
	// 		// moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
	// 		// task_->solutions().front()->toMsg(execute_goal.solution);
	// 		// execute.sendGoalAndWait(execute_goal);
	// 		// execute_result = execute.getResult()->error_code;

	// 		if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
	// 			ROS_ERROR_STREAM("Task execution failed and returned: " << execute_result.val);
	// 		}

	// 		// if (nh.param("execute", true)) {
	// 		// task.execute();
	// 		// 	ROS_INFO_NAMED(LOGNAME, "Execution complete");
	// 		// } else {
	// 		// 	ROS_INFO_NAMED(LOGNAME, "Execution disabled");
	// 		// }
	// } catch (const InitStageException& ex) {
	// 	std::cerr << "planning failed with exception" << std::endl << ex << task;
	// }

    // // task.printState();

    // Stage* generateGraspPoseStage = task.stages()->findChild("pick/grasp/compute ik");
    
    // if (generateGraspPoseStage) {
    //     // Proceed with accessing solutions
    //     // Iterate through solutions and retrieve costs
    //     for (const auto& solution : generateGraspPoseStage->solutions()) {
    //         double cost = solution->cost();
    //         const std::string comment = solution->comment();
    //         std::cout << "cost for angle " << comment << " is " << cost << std::endl;
    //     }
    // } else {
    //     // Handle the case where the stage is not found
    //     std::cerr << "Error: 'generate grasp pose' stage not found." << std::endl;
    // }

	ros::AsyncSpinner spinner(2);
	spinner.start();

	ros::waitForShutdown();  // keep alive for interactive inspection in rviz
	return 0;
}

