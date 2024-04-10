#ifndef SCHUNK_GRIPPER_H
#define SCHUNK_GRIPPER_H

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
#include "schunk_interface/srv/jog_to.hpp"

// EIPScanner libraries
#include "utils/Logger.h"
#include "utils/Buffer.h"
#include "SessionInfo.h"
#include <cip/connectionManager/NetworkConnectionParams.h>
#include "ConnectionManager.h"

#include "debug_codes.hpp"
#include "schunk_eip_parameters.hpp"

using namespace std::chrono_literals;
using eipScanner::cip::CipBool;
using eipScanner::cip::CipReal;
using eipScanner::cip::CipUsint;
using eipScanner::cip::CipDint;
using eipScanner::utils::Logger;
using eipScanner::utils::LogLevel;

using std::placeholders::_1;
using std::placeholders::_2;

using schunk_interface::msg::SchunkGripperMsg;

using schunk_interface::srv::JogTo;
using JogToRequestPtr = std::shared_ptr<schunk_interface::srv::JogTo::Request>;
using JogToResponsePtr = std::shared_ptr<schunk_interface::srv::JogTo::Response>;

class SchunkGripper : public rclcpp::Node
{
public:
    SchunkGripper(std::string node_name, std::string ip) : Node(node_name)
    {
        // ROS
        this->callback_group_reentrant = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Publishers
        state_publisher = this->create_publisher<SchunkGripperMsg>(node_name + "_state", 10);

        // Services
        this->jog_to_srv = this->create_service<JogTo>("jog_to", std::bind(&SchunkGripper::JogToSrv, this, _1, _2), rmw_qos_profile_services_default, this->callback_group_reentrant);

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

        std::vector<uint8_t> bytes = std::vector<uint8_t>(16);
        bytes[0] |= SET_1 << FAST_STOP_BIT_POS;
        bytes[0] |= SET_1 << ACKNOWLEDGE_BIT_POS;
        this->SetDataToSend(bytes);

        // Setting the handlers
        eipScanner::IOConnection::SendDataHandle
            sendDataHandler = [](std::vector<uint8_t> data)
        {
            std::ostringstream ss;
            for (auto &byte : data)
                ss << "[" << std::hex << (int)byte << "]";
            // Logger(LogLevel::INFO) << "Sent: " << ss.str();
        };
        eipScanner::IOConnection::ReceiveDataHandle receiveHandler = [this](auto realTimeHeader, auto sequence, auto data)
        { this->dataReceived = data; };
        eipScanner::IOConnection::CloseHandle closeConnectionHandler = []()
        { Logger(LogLevel::INFO) << "Closed"; };
        this->SetHandlers(sendDataHandler, receiveHandler, closeConnectionHandler);

        // Launching the communication in a different thread
        auto
            thread_function = [this]()
        {
            while (this->connectionManager.hasOpenConnections() && runningThread)
            {
                connectionManager.handleConnections(std::chrono::milliseconds(100)); // TODO: make timeout a parameter
                std::this_thread::sleep_for(100ms);
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
    // eipScanner::cip::MessageRouterResponse readEipData(eipScanner::cip::CipUint instance_id, eipScanner::cip::CipUint attribute_name);
    void DecodeImplicitData();

    // --------------------------------------------------------------------------------- //
    // ----------------------------------- Variables ----------------------------------- //
    // --------------------------------------------------------------------------------- //

    // ------- Ros variables ------- //
    rclcpp::CallbackGroup::SharedPtr callback_group_reentrant;

    // Publishers
    rclcpp::Publisher<schunk_interface::msg::SchunkGripperMsg>::SharedPtr state_publisher;

    // Services (server)
    rclcpp::Service<JogTo>::SharedPtr jog_to_srv;

    // Timers
    rclcpp::TimerBase::SharedPtr timer;

    // Callbacks
    void JogToSrv(const JogToRequestPtr req, JogToResponsePtr res);

    // ------- Eip variables ------- //

    // EIPScanner variables and data
    std::shared_ptr<eipScanner::SessionInfo> si;
    eipScanner::MessageRouter messageRouter;
    eipScanner::ConnectionManager connectionManager;
    eipScanner::IOConnection::WPtr io;

    // std::vector<uint8_t> dataToSend = std::vector<uint8_t>(16);
    std::vector<uint8_t> dataReceived = std::vector<uint8_t>(16);

    // Explicit data
    CipReal actual_vel;
    eipScanner::cip::CipWord grp_prehold_time;
    CipReal dead_load_kg;
    std::vector<CipReal> tool_cent_point;
    std::vector<CipReal> cent_of_mass;
    CipReal wp_lost_dst;
    CipReal wp_release_delta;
    CipReal grp_pos_margin;
    CipReal max_phys_stroke;
    CipReal grp_prepos_delta;
    CipReal min_pos;
    CipReal max_pos;
    CipReal zero_pos_ofs;
    CipReal min_vel;
    CipReal max_vel;
    CipReal max_grp_vel;
    CipReal min_grp_force;
    CipReal max_grp_force;
    std::vector<eipScanner::cip::CipByte> serial_no_num;
    CipUsint mac_addr;
    CipBool enable_softreset;

    // ------------ Implicit data ------------ //

    // Status
    CipDint ready_for_operation_bit;
    CipDint control_authority_fieldbus_bit;
    CipDint ready_for_shutdown_bit;
    CipDint not_feasible_bit;
    CipDint command_succesfully_processed_bit;
    CipDint command_received_toggle_bit;
    CipDint warning_bit;
    CipDint error_bit;

    CipDint released_for_manual_movement_bit;
    CipDint software_limit_reached_bit;
    CipDint no_workpiece_detected_bit;
    CipDint workpiece_gripped_bit;
    CipDint position_reached_bit;
    CipDint workpiece_pre_grip_started_bit;

    CipDint workpiece_lost_bit;
    CipDint wrong_workpiece_gripped_bit;

    CipDint grip_force_and_position_maintainance_activated_bit;

    CipReal actual_pos;

    // ------------ Diagnostic ------------ //
    CipDint error_code = 0;
    CipDint warning_code = 0;
    CipDint additional_code;

    // ------- Class variables ------- //
    bool runningThread = true;
    std::thread communication_thread;
};
#endif