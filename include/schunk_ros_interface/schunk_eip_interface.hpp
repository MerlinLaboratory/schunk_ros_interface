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

            // Getting the initial data
            this->getInitialEipData();
            this->getEipData();
        }

    private:
        void getInitialEipData();
        void getEipData();
        void publishStateUpdate();

        eipScanner::cip::MessageRouterResponse readEipData(eipScanner::cip::CipUint instance_id, eipScanner::cip::CipUint attribute_name);

        // Publishers
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher;

        // Timers
        rclcpp::TimerBase::SharedPtr timer;

        // State variables

        // EIPScanner variables and data
        eipScanner::MessageRouter messageRouter;
        std::shared_ptr<eipScanner::SessionInfo> si;

        eipScanner::cip::CipReal actual_pos;
        eipScanner::cip::CipReal actual_vel;
        eipScanner::cip::CipWord grp_prehold_time;
        eipScanner::cip::CipReal dead_load_kg;
        std::vector<eipScanner::cip::CipReal> tool_cent_point;
        std::vector<eipScanner::cip::CipReal> cent_of_mass;
        eipScanner::cip::CipReal wp_lost_dst;
        eipScanner::cip::CipReal wp_release_delta;
        eipScanner::cip::CipReal grp_pos_margin;
        eipScanner::cip::CipReal max_phys_stroke;
        eipScanner::cip::CipReal grp_prepos_delta;
        eipScanner::cip::CipReal min_pos;
        eipScanner::cip::CipReal max_pos;
        eipScanner::cip::CipReal zero_pos_ofs;
        eipScanner::cip::CipReal min_vel;
        eipScanner::cip::CipReal max_vel;
        eipScanner::cip::CipReal max_grp_vel;
        eipScanner::cip::CipReal min_grp_force;
        eipScanner::cip::CipReal max_grp_force;
        std::vector<eipScanner::cip::CipByte> serial_no_num;
        eipScanner::cip::CipUsint mac_addr;
        eipScanner::cip::CipBool enable_softreset;
   
};