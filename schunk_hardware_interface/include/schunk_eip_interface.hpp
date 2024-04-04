// Standard libraries
#include <chrono>
#include <functional>
#include <memory>
#include <iomanip>
#include <thread>

// ROS2 libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "schunk_interface/msg/schunk_gripper_msg.hpp"

// EIPScanner libraries
#include "utils/Logger.h"
#include "utils/Buffer.h"
#include "SessionInfo.h"
#include <cip/connectionManager/NetworkConnectionParams.h>
#include "ConnectionManager.h"

#include "schunk_eip_parameters.hpp"

// using namespace eipScanner::cip;
using namespace std::chrono_literals;

using eipScanner::utils::Logger;
using eipScanner::utils::LogLevel;

class SchunkGripper : public rclcpp::Node
{
public:
    SchunkGripper(std::string node_name, std::string ip) : Node(node_name)
    {
        // ROS
        // Publishers
        state_publisher = this->create_publisher<schunk_interface::msg::SchunkGripperMsg>(node_name + "_state", 10);

        // Timers callbacks
        timer = this->create_wall_timer(100ms, std::bind(&SchunkGripper::publishStateUpdate, this));

        // Ethernet/IP
        // Setting Loggin level
        Logger::setLogLevel(LogLevel::INFO);

        // Enstablish explicit connection and getting the initial data
        this->si = std::make_shared<eipScanner::SessionInfo>(ip, 0xAF12);
        this->getExplicitEipData();

        // Enstablish implicit connection (check EDS file)
        eipScanner::cip::connectionManager::ConnectionParameters parameters;

        parameters.transportTypeTrigger |= eipScanner::cip::connectionManager::NetworkConnectionParams::CLASS1;
        parameters.transportTypeTrigger |= eipScanner::cip::connectionManager::NetworkConnectionParams::TRIG_CYCLIC;
        parameters.transportTypeTrigger |= eipScanner::cip::connectionManager::NetworkConnectionParams::OWNED;

        parameters.connectionTimeoutMultiplier = 3;

        parameters.t2oRealTimeFormat = false;
        parameters.t2oRPI = 10000;
        parameters.t2oNetworkConnectionParams |= eipScanner::cip::connectionManager::NetworkConnectionParams::P2P;
        parameters.t2oNetworkConnectionParams |= eipScanner::cip::connectionManager::NetworkConnectionParams::SCHEDULED_PRIORITY;
        parameters.t2oNetworkConnectionParams |= eipScanner::cip::connectionManager::NetworkConnectionParams::FIXED;
        parameters.t2oNetworkConnectionParams |= 16;

        parameters.o2tRealTimeFormat = true;
        parameters.o2tRPI = 10000;
        parameters.o2tNetworkConnectionParams |= eipScanner::cip::connectionManager::NetworkConnectionParams::P2P;
        parameters.o2tNetworkConnectionParams |= eipScanner::cip::connectionManager::NetworkConnectionParams::SCHEDULED_PRIORITY;
        parameters.o2tNetworkConnectionParams |= eipScanner::cip::connectionManager::NetworkConnectionParams::FIXED;
        parameters.o2tNetworkConnectionParams |= 16;

        parameters.connectionPath = {0x20, 0x04, 0x24, 0x00, 0x2C, 0x96, 0x2C, 0x64};
        parameters.originatorVendorId = 900;

        this->io = this->connectionManager.forwardOpen(si, parameters);

        this->SetDataToSend(this->dataToSend);

        // Setting the handlers
        eipScanner::IOConnection::SendDataHandle
            sendDataHandler = [](std::vector<uint8_t> data)
        {
            std::ostringstream ss;
            for (auto &byte : data)
                ss << "[" << std::hex << (int)byte << "]";
            Logger(LogLevel::INFO) << "Sent: " << ss.str();
        };
        eipScanner::IOConnection::ReceiveDataHandle receiveHandler = [this](auto realTimeHeader, auto sequence, auto data) { this->dataReceived = data; };
        eipScanner::IOConnection::CloseHandle closeConnectionHandler = []() { Logger(LogLevel::INFO) << "Closed"; };
        this->SetHandlers(sendDataHandler, receiveHandler, closeConnectionHandler);

        // Launching the communication in a different thread
        auto
            thread_function = [this]()
        {
            while (this->connectionManager.hasOpenConnections() && runningThread)
            {
                connectionManager.handleConnections(std::chrono::milliseconds(100));
            }
            Logger(LogLevel::ERROR) << "Connection has been closed";
        };
        this->communication_thread = std::thread(thread_function);
    }

    ~SchunkGripper()
    {
        Logger(LogLevel::INFO) << "Destructor called";

        // Waiting for communication thread for finishing
        this->runningThread = false;
        if (communication_thread.joinable())
        {
            communication_thread.join();
        }

        // Closing EIP connection
        this->connectionManager.forwardClose(this->si, this->io);
    }

private:
    // Getters
    void getExplicitEipData();

    // Setters
    void SetDataToSend(std::vector<uint8_t> data);
    void SetHandlers(eipScanner::IOConnection::SendDataHandle sendHandler,
                     eipScanner::IOConnection::ReceiveDataHandle receiveHandler,
                     eipScanner::IOConnection::CloseHandle closeHandler);

    // Ros callbacks
    void publishStateUpdate();

    // Functions
    eipScanner::cip::MessageRouterResponse readEipData(eipScanner::cip::CipUint instance_id, eipScanner::cip::CipUint attribute_name);

    // --------------------------------------------------------------------------------- //
    // ----------------------------------- Variables ----------------------------------- //
    // --------------------------------------------------------------------------------- //

    // ------- Ros variables ------- //

    // Publishers
    rclcpp::Publisher<schunk_interface::msg::SchunkGripperMsg>::SharedPtr state_publisher;

    // Timers
    rclcpp::TimerBase::SharedPtr timer;

    // ------- Eip variables ------- //

    // EIPScanner variables and data
    std::shared_ptr<eipScanner::SessionInfo> si;
    eipScanner::MessageRouter messageRouter;
    eipScanner::ConnectionManager connectionManager;
    eipScanner::IOConnection::WPtr io;

    std::vector<uint8_t> dataToSend = std::vector<uint8_t>(16);
    std::vector<uint8_t> dataReceived = std::vector<uint8_t>(16);

    // Async EIP data
    eipScanner::cip::CipReal actual_pos; // TODO: Move to cyclical
    eipScanner::cip::CipReal actual_vel; // TODO: Move to cyclical
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

    // ------- Class variables ------- //
    bool runningThread = true;
    std::thread communication_thread;
};