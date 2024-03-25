// Standard libraries
#include <chrono>
#include <functional>
#include <memory>
#include <iomanip>

// ROS2 libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// EIPScanner libraries
#include "utils/Logger.h"
#include "utils/Buffer.h"

#include "schunk_eip_parameters.hpp"

using namespace eipScanner::cip;
using namespace std::chrono_literals;

using eipScanner::utils::Logger;
using eipScanner::utils::LogLevel;
using eipScanner::utils::Buffer;

class SchunkGripper : public rclcpp::Node
{
    public:
        SchunkGripper(std::string node_name, std::string ip): Node(node_name)
        {
            // Publishers
            state_publisher = this->create_publisher<std_msgs::msg::String>(node_name + "_state", 10);

            // Timers callbacks
            timer = this->create_wall_timer(500ms, std::bind(&SchunkGripper::publishStateUpdate, this));

            // Enstablish connection
            this->si = std::make_shared<eipScanner::SessionInfo>(ip, 0xAF12);

        }

    private:
        void getInitialEipData();
        void getEipData();
        void publishStateUpdate();

        eipScanner::cip::MessageRouterResponse readEipData(CipUint instance_id, CipUint attribute_name);

        // Publishers
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher;

        // Timers
        rclcpp::TimerBase::SharedPtr timer;

        // State variables

        // EIPScanner variables
        eipScanner::MessageRouter messageRouter;
        std::shared_ptr<eipScanner::SessionInfo> si;

};