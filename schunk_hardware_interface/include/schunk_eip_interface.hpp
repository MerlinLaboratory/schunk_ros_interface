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
#include "std_srvs/srv/trigger.hpp"
#include "schunk_interfaces/msg/schunk_gripper_msg.hpp"
#include "schunk_interfaces/srv/jog_to.hpp"
#include "schunk_interfaces/srv/simple_grip.hpp"
#include "schunk_interfaces/srv/release.hpp"

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
using eipScanner::cip::CipDint;
using eipScanner::cip::CipReal;
using eipScanner::cip::CipUsint;
using eipScanner::utils::Logger;
using eipScanner::utils::LogLevel;

using std::placeholders::_1;
using std::placeholders::_2;

using schunk_interfaces::msg::SchunkGripperMsg;

using schunk_interfaces::srv::JogTo;
using JogToRequestPtr = std::shared_ptr<schunk_interfaces::srv::JogTo::Request>;
using JogToResponsePtr = std::shared_ptr<schunk_interfaces::srv::JogTo::Response>;

using schunk_interfaces::srv::SimpleGrip;
using SimpleGripRequestPtr = std::shared_ptr<schunk_interfaces::srv::SimpleGrip::Request>;
using SimpleGripResponsePtr = std::shared_ptr<schunk_interfaces::srv::SimpleGrip::Response>;

using schunk_interfaces::srv::Release;
using ReleaseRequestPtr = std::shared_ptr<schunk_interfaces::srv::Release::Request>;
using ReleaseResponsePtr = std::shared_ptr<schunk_interfaces::srv::Release::Response>;

using std_srvs::srv::Trigger;
using TriggerRequestPtr = std::shared_ptr<std_srvs::srv::Trigger::Request>;
using TriggerResponsePtr = std::shared_ptr<std_srvs::srv::Trigger::Response>;

class SchunkGripper : public rclcpp::Node
{
public:
    SchunkGripper();
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
    void readExplicitData();

    // Setters
    void setDataToSend(std::vector<uint8_t> data);
    void setHandlers(eipScanner::IOConnection::SendDataHandle sendHandler,
                     eipScanner::IOConnection::ReceiveDataHandle receiveHandler,
                     eipScanner::IOConnection::CloseHandle closeHandler);

    // Ros callbacks
    void publishStateUpdate();
    void acknowledgeSrv(const TriggerRequestPtr, TriggerResponsePtr res);
    void jogToSrv(const JogToRequestPtr req, JogToResponsePtr res);
    void simpleGripSrv(const SimpleGripRequestPtr req, SimpleGripResponsePtr res);
    void releaseSrv(const ReleaseRequestPtr req, ReleaseResponsePtr res);

    // Parameters
    void declareParameters();

    // Functions
    void readImplicitData();
    void sendDefaultData();
    void sendAcknowledgeGripper();
    bool waitForCommandReceivedToggle(int32_t prev_command_received_toggle_bit, int timeInSeconds, std::stringstream &debug_ss);
    bool waitForActionFinish(int32_t &feedback_bit, int timeInSeconds, std::stringstream &debug_ss);

    // --------------------------------------------------------------------------------- //
    // ----------------------------------- Variables ----------------------------------- //
    // --------------------------------------------------------------------------------- //

    // ------- Ros variables ------- //
    rclcpp::CallbackGroup::SharedPtr callback_group_reentrant;

    // Publishers
    rclcpp::Publisher<schunk_interfaces::msg::SchunkGripperMsg>::SharedPtr state_publisher;

    // Services (server)
    rclcpp::Service<Trigger>::SharedPtr acknowledge_srv;
    rclcpp::Service<JogTo>::SharedPtr jog_to_srv;
    rclcpp::Service<SimpleGrip>::SharedPtr simple_grip_srv;
    rclcpp::Service<Release>::SharedPtr release_srv;

    // Timers
    rclcpp::TimerBase::SharedPtr timer;

    // ------- Eip variables ------- //

    // EIPScanner variables and data
    std::shared_ptr<eipScanner::SessionInfo> si;
    eipScanner::MessageRouter messageRouter;
    eipScanner::ConnectionManager connectionManager;
    eipScanner::IOConnection::WPtr io;

    std::vector<uint8_t> dataSent = std::vector<uint8_t>(16);
    std::vector<uint8_t> dataReceived = std::vector<uint8_t>(16);

    // Explicit data
    eipScanner::cip::CipWord grp_prehold_time; // Read and write
    CipReal dead_load_kg; // Read
    std::vector<double> tool_cent_point; // Read
    std::vector<double> cent_of_mass; // Read
    CipReal wp_lost_dst; // Read and write
    CipReal wp_release_delta; // Read and write
    CipReal grp_pos_margin; // Read and write
    CipReal max_phys_stroke; // Read
    CipReal grp_prepos_delta; // Read and write
    CipReal min_pos; // Read and write
    CipReal max_pos; // Read and write
    CipReal zero_pos_ofs; // Read and write
    CipReal min_vel; // Read
    CipReal max_vel; // Read
    CipReal max_grp_vel; // Read
    CipReal min_grp_force; // Read
    CipReal max_grp_force; // Read

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
    std::string node_name;
    std::string gripper_ip;
    bool runningThread = true;
    std::thread communication_thread;
    bool repeat_command_toggle_high = false;
};
#endif