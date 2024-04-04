#include <string>
#include <iostream>

// ROS2 libraries
#include "rclcpp/rclcpp.hpp"

#include "../include/schunk_eip_interface.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    // Gripper parameters
    std::string node_name = "egk_40";
    std::string ip = "192.168.125.160";

    std::shared_ptr<SchunkGripper> p_gripper = std::make_shared<SchunkGripper>(node_name, ip);

    executor.add_node(p_gripper);
    executor.spin();
    
    rclcpp::shutdown();
    
    return 0;
}