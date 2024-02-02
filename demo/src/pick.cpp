#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_grasp_pose_follower.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gtest/gtest.h>

using namespace moveit::task_constructor;

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
    pose.position.z = 0.0;
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
    pose.position.z = 0.0;
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

moveit_msgs::CollisionObject createCubeObject(ros::Publisher& marker_pub) {
	std::string object_name="object";
    std::string object_reference_frame = "world";
	std::vector<double> object_dimensions = {0.1, 0.05, 0.03};
	geometry_msgs::Pose pose;
	pose.position.x = 0.30702;
    pose.position.y = 0.2;
    pose.position.z = 0.285;
    pose.orientation.x = 1.0;

	moveit_msgs::CollisionObject object;
	object.id = object_name;
	object.header.frame_id = object_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	object.primitives[0].dimensions = object_dimensions;
	// pose.position.z += 0.5 * object_dimensions[0];
	object.primitive_poses.push_back(pose);

	// Append frame markers
	std::deque<visualization_msgs::Marker> object_markers;
    geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.pose = pose;
    pose_stamped.header.frame_id = "world";
	rviz_marker_tools::appendFrame(object_markers, pose_stamped, 0.1, "object frame");

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
	return object;
}

void setupDemoScene() {
	// Add table and object to planning scene
	ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	spawnObject(psi, createTable());
	spawnObject(psi, createObject());
}

Task createTask() {
    std::string arm_group = "panda_arm";
    std::string eef_group = "hand";

	Task t;
    t.stages()->setName("Pick");
    t.loadRobotModel();

	Stage* initial_stage = new stages::CurrentState("current state");
	t.add(std::unique_ptr<Stage>(initial_stage));

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");

	// connect to pick
	stages::Connect::GroupPlannerVector planners = {{arm_group, pipeline}};
	auto connect = std::make_unique<stages::Connect>("connect", planners);
	connect->properties().configureInitFrom(Stage::PARENT);
	t.add(std::move(connect));

	// grasp generator
	auto grasp_generator = new stages::GenerateGraspPose("generate grasp pose");
	grasp_generator->setAngleDelta(.2); // enumerate over angles from 0 to 6.4 (less then 2 PI)
	grasp_generator->setPreGraspPose("open");
	grasp_generator->setGraspPose("close");
	grasp_generator->setMonitoredStage(initial_stage);

	auto grasp = std::make_unique<stages::SimpleGrasp>(std::unique_ptr<MonitoringGenerator>(grasp_generator));
	Eigen::Isometry3d ik_frame = Eigen::Isometry3d::Identity();
    // T_FtoTCP_3d = np.array([[0.707, 0.707, 0, 0],
    //                     [-0.707, 0.707, 0, 0],
    //                     [0, 0, 1, 0.1034],
    //                     [0, 0, 0, 1]])
    ik_frame.translation().z() = 0.1034;
    grasp->setIKFrame(ik_frame, "panda_hand");

	// pick stage
	auto pick = std::make_unique<stages::Pick>(std::move(grasp));
	pick->setProperty("eef", std::string("hand"));
	pick->setProperty("object", std::string("object"));
	geometry_msgs::TwistStamped approach;
	approach.header.frame_id = "world";
	approach.twist.linear.z = -1.0;
	pick->setApproachMotion(approach, 0.03, 0.1);

	geometry_msgs::TwistStamped lift;
	lift.header.frame_id = "panda_hand";
	lift.twist.linear.z = -1.0;
	pick->setLiftMotion(lift, 0.03, 0.1);

	t.add(std::move(pick));

	return t;
}

Task createTaskSimpleGrasp() {
    std::string arm_group = "panda_arm";
    std::string eef_group = "hand";

	Task t;
    t.stages()->setName("Pick");
    t.loadRobotModel();

	Stage* initial_stage = new stages::CurrentState("current state");
	t.add(std::unique_ptr<Stage>(initial_stage));

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");

    // Set task properties
	t.setProperty("group", "panda_arm");
	t.setProperty("eef", "hand"); // 
	t.setProperty("hand", "hand"); //hand
	t.setProperty("hand_grasping_frame", "panda_link8");
	t.setProperty("ik_frame", "panda_link8"); 

	// connect to pick
	stages::Connect::GroupPlannerVector planners = {{arm_group, pipeline}};
	auto connect = std::make_unique<stages::Connect>("connect", planners);
	connect->properties().configureInitFrom(Stage::PARENT);
	t.add(std::move(connect));

	// grasp generator
	auto grasp_generator = new stages::GenerateGraspPose("generate grasp pose");
    grasp_generator->properties().configureInitFrom(Stage::PARENT);
	grasp_generator->properties().set("marker_ns", "grasp_pose");
	grasp_generator->setAngleDelta(.2); // enumerate over angles from 0 to 6.4 (less then 2 PI)
	grasp_generator->setPreGraspPose("open");
	grasp_generator->setGraspPose("close");
    grasp_generator->setObject("object");
    // grasp_generator->setEndEffector("hand");
	grasp_generator->setMonitoredStage(initial_stage);

	auto grasp = std::make_unique<stages::SimpleGrasp>(std::unique_ptr<MonitoringGenerator>(grasp_generator));
	// grasp->properties().set("eef", "hand");
    grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
	grasp->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
    grasp->properties().set("object", "object");
    Eigen::Isometry3d ik_frame = Eigen::Isometry3d::Identity();
    // T_FtoTCP_3d = np.array([[0.707, 0.707, 0, 0],
    //                     [-0.707, 0.707, 0, 0],
    //                     [0, 0, 1, 0.1034],
    //                     [0, 0, 0, 1]])
    ik_frame.translation().z() = 0.1034;
    grasp->setIKFrame(ik_frame, "panda_hand");

    t.add(std::move(grasp));

	return t;
}

Task createTaskRaw() {
    std::string arm_group = "panda_arm";
    std::string eef_group = "hand";

	Task t;
    t.stages()->setName("Pick");
    t.loadRobotModel();

	Stage* initial_stage = new stages::CurrentState("current state");
	t.add(std::unique_ptr<Stage>(initial_stage));

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");

    // Set task properties
	t.setProperty("group", "panda_arm");
	t.setProperty("eef", "hand"); // 
	t.setProperty("hand", "hand"); //hand
	t.setProperty("hand_grasping_frame", "panda_link8");
	t.setProperty("ik_frame", "panda_link8"); 

	// transfomr from flange to gripper
    std::vector<double> grasp_frame_transform_vector = {0, 0, 0.1, 1.571, 0.785, 1.571};
    Eigen::Isometry3d grasp_frame_transform; 
    convertToEigen(grasp_frame_transform_vector, grasp_frame_transform);

	// connect to pick
	stages::Connect::GroupPlannerVector planners = {{arm_group, pipeline}};
	auto connect = std::make_unique<stages::Connect>("connect", planners);
	connect->properties().configureInitFrom(Stage::PARENT);
	t.add(std::move(connect));

	// grasp generator
	std::vector<double> grasp_target = {-0.05, 0.0, 0.0};
	auto grasp_generator = std::make_unique<stages::GenerateGraspPoseFollow>("generate grasp pose");
    grasp_generator->properties().configureInitFrom(Stage::PARENT);
	grasp_generator->properties().set("marker_ns", "grasp_pose");
	grasp_generator->properties().set("explr_axis", "y");
	grasp_generator->setAngleDelta(.5); // enumerate over angles from 0 to 6.4 (less then 2 PI)
	grasp_generator->setPreGraspPose("open");
	grasp_generator->setGraspPose("close");
    grasp_generator->setObject("object");
	grasp_generator->setTarget(grasp_target);
    // grasp_generator->setEndEffector("hand");
	grasp_generator->setMonitoredStage(initial_stage);

	auto ik_wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(grasp_generator));
	ik_wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
	ik_wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
    // grasp->properties().set("eef", "hand");
    // ik_wrapper->properties().set("object", "object");
    ik_wrapper->setMaxIKSolutions(2);
	ik_wrapper->setMinSolutionDistance(1.0);

    // ik_wrapper->setIKFrame(grasp_frame_transform, "panda_link8");
	
    Eigen::Isometry3d ik_frame = Eigen::Isometry3d::Identity();
    // // T_FtoTCP_3d = np.array([[0.707, 0.707, 0, 0],
    // //                     [-0.707, 0.707, 0, 0],
    // //                     [0, 0, 1, 0.1034],
    // //                     [0, 0, 0, 1]])
    ik_frame.translation().z() = 0.1034;
    // // TODO: should it be "panda_hand" or "panda_link8"?
    ik_wrapper->setIKFrame(ik_frame, "panda_hand");

    t.add(std::move(ik_wrapper));

	return t;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");
	ros::NodeHandle nh;
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("interactive_robot_marray", 10);
	// run an asynchronous spinner to communicate with the move_group node and rviz
	ros::AsyncSpinner spinner(1);
	spinner.start();

    // setupDemoScene();
    ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	spawnObject(psi, createCubeObject(marker_pub));

	auto task = createTaskRaw();
	try {
		if (task.plan())
            
			task.introspection().publishSolution(*task.solutions().front());
	} catch (const InitStageException& ex) {
		std::cerr << "planning failed with exception" << std::endl << ex << task;
	}
	

    // task.printState();

    Stage* generateGraspPoseStage = task.stages()->findChild("pick/grasp/compute ik");
    
    if (generateGraspPoseStage) {
        // Proceed with accessing solutions
        // Iterate through solutions and retrieve costs
        for (const auto& solution : generateGraspPoseStage->solutions()) {
            double cost = solution->cost();
            const std::string comment = solution->comment();
            std::cout << "cost for angle " << comment << " is " << cost << std::endl;
        }
    } else {
        // Handle the case where the stage is not found
        std::cerr << "Error: 'generate grasp pose' stage not found." << std::endl;
    }

	ros::waitForShutdown();  // keep alive for interactive inspection in rviz
	return 0;
}

