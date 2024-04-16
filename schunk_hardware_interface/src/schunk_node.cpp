#include <string>
#include <iostream>

// ROS2 libraries
#include "rclcpp/rclcpp.hpp"

#include "../include/schunk_eip_interface.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<SchunkGripper> p_gripper = std::make_shared<SchunkGripper>();

    executor.add_node(p_gripper);
    executor.spin();
    
    rclcpp::shutdown();
    
    return 0;
}