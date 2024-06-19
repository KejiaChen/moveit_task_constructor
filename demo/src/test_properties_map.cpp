#include <map>
#include <string>
#include <memory>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/properties.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>

// namespace moveit {
// namespace task_constructor {
// namespace stages {

using namespace moveit::task_constructor;

int main(int argc, char** argv) {

    ros::init(argc, argv, "mtc_tutorial");
	ros::NodeHandle nh;

    // Define a map to hold properties for each robot
    std::map<std::string, PropertyMap> robot_properties_map;

    // Define properties for each robot using a loop
    for (int i = 1; i <= 2; ++i) { // Assuming two robots
        std::string robot_name = "robot" + std::to_string(i);
        auto& robot_properties = robot_properties_map[robot_name];

        // Define properties for the current robot
        robot_properties.declare<std::string>("eef", "name of end-effector group for " + robot_name);
        robot_properties.declare<std::string>("group", "name of active group for " + robot_name + " (derived from eef if not provided)");
        robot_properties.declare<std::string>("default_pose", "", "default joint pose of active group for " + robot_name + " (defines cost of IK)");
        robot_properties.declare<uint32_t>("max_ik_solutions", 1, "maximum number of IK solutions for " + robot_name);
        robot_properties.declare<bool>("ignore_collisions", false, "whether to ignore collisions for " + robot_name);
        robot_properties.declare<double>("min_solution_distance", 0.1,
                                           "minimum distance between separate IK solutions for the same target for " + robot_name);
        robot_properties.declare<geometry_msgs::PoseStamped>("ik_frame", "frame to be moved towards goal pose for " + robot_name);
        robot_properties.declare<geometry_msgs::PoseStamped>("target_pose", "goal pose for ik frame for " + robot_name);
    }

    // Access and use the properties for each robot from the map as needed
    for (const auto& entry : robot_properties_map) {
        std::string robot_name = entry.first;
        auto& properties = entry.second;
        // Use properties for the current robot
        std::cout << "Properties for " << robot_name << ":" << std::endl;
        // Access properties as needed
    }

    ros::waitForShutdown();

    return 0;
}

// }
// }
// }