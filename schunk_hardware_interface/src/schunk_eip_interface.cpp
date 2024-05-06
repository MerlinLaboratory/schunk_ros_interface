#include "../include/schunk_eip_interface.hpp"

// ------------------------------------------------------------------------------------ //
// --------------------------------- Utils Functions ---------------------------------- //
// ------------------------------------------------------------------------------------ //

void setBit(uint8_t &byte, const uint8_t bit_position)
{
    byte |= 1 << bit_position;
}

void resetBit(uint8_t &byte, const uint8_t bit_position)
{
    byte &= ~(1 << bit_position);
}

bool isBitHigh(uint8_t byte, const uint8_t bit_position)
{
    byte >> bit_position & 0x01;
    return byte ? true : false;
}

// ------------------------------------------------------------------------------------ //
// ----------------------- Functions for Implicit Communication ----------------------- //
// ------------------------------------------------------------------------------------ //

void SchunkGripper::setHandlers(eipScanner::IOConnection::SendDataHandle sendHandler,
                                eipScanner::IOConnection::ReceiveDataHandle receiveHandler,
                                eipScanner::IOConnection::CloseHandle closeHandler)
{
    if (auto ptr = this->io.lock())
    {
        if (sendHandler != nullptr)
            ptr->setSendDataListener(sendHandler);
        if (receiveHandler != nullptr)
            ptr->setReceiveDataListener(receiveHandler);
        if (closeHandler != nullptr)
            ptr->setCloseListener(closeHandler);
    }
    else
    {
        Logger(LogLevel::ERROR) << "Failed to open connection";
        throw std::runtime_error("Failed to open connection");
    }
}

void SchunkGripper::setDataToSend(std::vector<uint8_t> data)
{
    if (auto ptr = this->io.lock())
    {
        this->dataSent = data;
        ptr->setDataToSend(data);
    }
    else
    {
        Logger(LogLevel::ERROR) << "Failed to open connection";
        throw std::runtime_error("Failed to open connection");
    }
}

void SchunkGripper::readImplicitData()
{
    // Check Schunk Documentation Pag 17/120
    // Deconding Status
    int status_start = 0;
    int status_end = 4;

    std::vector<uint8_t> status_bytes = std::vector<uint8_t>(this->dataReceived.begin() + status_start,
                                                             this->dataReceived.begin() + status_end);

    this->ready_for_operation_bit = (status_bytes[0] >> READY_FOR_OPERATION_BIT_POS) & 0x01;
    this->control_authority_fieldbus_bit = (status_bytes[0] >> CONTROL_AUTHORITY_FIELDBUS_BIT_POS) & 0x01;
    this->ready_for_shutdown_bit = (status_bytes[0] >> READY_FOR_SHUTDOWN_BIT_POS) & 0x01;
    this->not_feasible_bit = (status_bytes[0] >> NOT_FEASIBLE_BIT_POS) & 0x01;
    this->command_succesfully_processed_bit = (status_bytes[0] >> COMMAND_SUCCESFULLY_PROCESSED_BIT_POS) & 0x01;
    this->command_received_toggle_bit = (status_bytes[0] >> COMMAND_RECEIVED_TOGGLE_BIT_POS) & 0x01;
    this->warning_bit = (status_bytes[0] >> WARNING_BIT_POS) & 0x01;
    this->error_bit = (status_bytes[0] >> ERROR_BIT_POS) & 0x01;

    this->released_for_manual_movement_bit = (status_bytes[1] >> RELEASED_FOR_MANUAL_MOVEMENT_BIT_POS) & 0x01;
    this->software_limit_reached_bit = (status_bytes[1] >> SOFTWARE_LIMIT_REACHED_BIT_POS) & 0x01;
    this->no_workpiece_detected_bit = (status_bytes[1] >> NO_WORKPIECE_DETECTED_BIT_POS) & 0x01;
    this->workpiece_gripped_bit = (status_bytes[1] >> WORKPIECE_GRIPPED_BIT_POS) & 0x01;
    this->position_reached_bit = (status_bytes[1] >> POSITION_REACHED_BIT_POS) & 0x01;
    this->workpiece_pre_grip_started_bit = (status_bytes[1] >> WORKPIECE_PRE_GRIP_STARTED_BIT_POS) & 0x01;

    // Decoding Actual pos
    int actual_pos_start = 4;
    int actual_pos_end = 8;

    std::vector<uint8_t> actual_pos_bytes = std::vector<uint8_t>(this->dataReceived.begin() + actual_pos_start,
                                                                 this->dataReceived.begin() + actual_pos_end);

    int actual_pos_um;
    memcpy(&actual_pos_um, actual_pos_bytes.data(), sizeof(float));
    this->actual_pos = actual_pos_um / 1000.0f; // from micrometers to mm

    // Deconding Diagnostic
    int diagnostic_pos_start = 12;
    int diagnostic_pos_end = 16;

    std::vector<uint8_t> diagnostic_pos_bytes = std::vector<uint8_t>(this->dataReceived.begin() + diagnostic_pos_start,
                                                                     this->dataReceived.begin() + diagnostic_pos_end);

    this->error_code = diagnostic_pos_bytes[0];
    this->warning_code = diagnostic_pos_bytes[2];
    this->additional_code = diagnostic_pos_bytes[3];
}

// ------------------------------------------------------------------------------------ //
// ----------------------- Functions for Explicit Communication ----------------------- //
// ------------------------------------------------------------------------------------ //

template <typename dataType>
dataType getExplicitData(std::shared_ptr<eipScanner::SessionInfo> si_async, eipScanner::cip::CipUint instance_id, eipScanner::cip::CipUint attribute_name)
{
    // Read Parameter Class Descriptor
    eipScanner::MessageRouter messageRouter;
    eipScanner::cip::MessageRouterResponse response = messageRouter.sendRequest(si_async, eipScanner::cip::ServiceCodes::GET_ATTRIBUTE_SINGLE, eipScanner::cip::EPath(CLASS, instance_id, attribute_name));

    if (response.getGeneralStatusCode() != eipScanner::cip::GeneralStatusCodes::SUCCESS)
    {
        Logger(LogLevel::ERROR) << "Failed to read";
        logGeneralAndAdditionalStatus(response);

        throw std::runtime_error("Failed to read attribute id:" + std::to_string(attribute_name) + " instance id:" + std::to_string(instance_id));
    }

    eipScanner::utils::Buffer buffer = eipScanner::utils::Buffer(response.getData());
    dataType data;
    buffer >> data;

    return data;
}

void SchunkGripper::readExplicitData()
{
    try
    {
        this->dead_load_kg = getExplicitData<CipReal>(this->si, DEAD_LOAD_KG_ID, VALUE_ATTRIBUTE);
        this->max_phys_stroke = getExplicitData<CipReal>(this->si, MAX_PHYS_STROKE_ID, VALUE_ATTRIBUTE);
        this->min_vel = getExplicitData<CipReal>(this->si, MIN_VEL_ID, VALUE_ATTRIBUTE);
        this->max_vel = getExplicitData<CipReal>(this->si, MAX_VEL_ID, VALUE_ATTRIBUTE);
        this->max_grp_vel = getExplicitData<CipReal>(this->si, MAX_GRP_VEL_ID, VALUE_ATTRIBUTE);
        this->min_grp_force = getExplicitData<CipReal>(this->si, MIN_GRP_FORCE_ID, VALUE_ATTRIBUTE);
        this->max_grp_force = getExplicitData<CipReal>(this->si, MAX_GRP_FORCE_ID, VALUE_ATTRIBUTE);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

void writeExplicitEipData(std::shared_ptr<eipScanner::SessionInfo> si_async, CipUint instance_id, CipUint attribute_name, CipReal input_data)
{
    std::vector<uint8_t> serialised_data(sizeof(CipReal));
    memcpy(serialised_data.data(), &input_data, sizeof(CipReal));

    // Read Parameter Class Descriptor
    eipScanner::MessageRouter messageRouter;
    eipScanner::cip::EPath path(CLASS, instance_id, attribute_name);
    eipScanner::cip::MessageRouterResponse response = messageRouter.sendRequest(si_async, eipScanner::cip::ServiceCodes::SET_ATTRIBUTE_SINGLE, path, serialised_data);

    if (response.getGeneralStatusCode() != eipScanner::cip::GeneralStatusCodes::SUCCESS)
    {
        Logger(LogLevel::ERROR) << "Failed to write";
        logGeneralAndAdditionalStatus(response);

        throw std::runtime_error("Failed to write attribute id:" + std::to_string(attribute_name) + " instance id:" + std::to_string(instance_id));
    }

    return;
}

// ------------------------------------------------------------------------------------ //
// ---------------------------------- Class Functions --------------------------------- //
// ------------------------------------------------------------------------------------ //

void SchunkGripper::sendDefaultData()
{
    // Sending the default vector first
    std::vector<uint8_t> starting_bytes = std::vector<uint8_t>(16);
    setBit(starting_bytes[0], FAST_STOP_BIT_POS);
    this->setDataToSend(starting_bytes);
    std::this_thread::sleep_for(300ms);
}

void SchunkGripper::sendAcknowledgeGripper()
{
    this->sendDefaultData();
    std::vector<uint8_t> bytes = this->dataSent;

    // Resetting the ACKNOWLEDGE_BIT first if necessary
    if (isBitHigh(bytes[0], ACKNOWLEDGE_BIT_POS) == true)
    {
        resetBit(bytes[0], ACKNOWLEDGE_BIT_POS);
        this->setDataToSend(bytes);
        std::this_thread::sleep_for(500ms);
    }

    setBit(bytes[0], ACKNOWLEDGE_BIT_POS);
    this->setDataToSend(bytes);
}

bool SchunkGripper::waitForCommandReceivedToggle(int32_t prev_command_received_toggle_bit, int timeInSeconds, std::stringstream &debug_ss)
{
    std::chrono::duration<int> timeout(timeInSeconds);

    auto start = std::chrono::system_clock::now();
    while (prev_command_received_toggle_bit == this->command_received_toggle_bit)
    {
        auto now = std::chrono::system_clock::now();
        auto elapesed_time = now - start;

        if (elapesed_time > timeout)
        {
            debug_ss << "Command not received by gripper";
            return false;
        }

        std::this_thread::sleep_for(300ms);
    }

    return true;
}

bool SchunkGripper::waitForActionFinish(int32_t &feedback_bit, int timeInSeconds, std::stringstream &debug_ss)
{
    std::chrono::duration<int> timeout(timeInSeconds);

    auto start = std::chrono::system_clock::now();
    while (feedback_bit == false)
    {
        auto now = std::chrono::system_clock::now();
        auto elapesed_time = now - start;

        if (elapesed_time > timeout)
        {
            debug_ss << "Cannot finish within timeout";
            RCLCPP_ERROR(this->get_logger(), debug_ss.str().c_str());
            return false;
        }

        std::this_thread::sleep_for(300ms);
    }

    return true;
}

// ------------------------------------------------------------------------------------ //
// ---------------------------------- ROS Functions ----------------------------------- //
// ------------------------------------------------------------------------------------ //

void SchunkGripper::declareParameters()
{
    this->readExplicitData();

    // Declaration
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.read_only = false;
    param_desc.description = "This parameter can be used to read and write the time span for the re-gripping. [ms]";
    this->declare_parameter("grp_prehold_time", 0.0, param_desc);
    this->grp_prehold_time = this->get_parameter("grp_prehold_time").as_double();

    param_desc.read_only = true;
    param_desc.description = "The mass of the module can be read out and written with this parameter. [kg]";
    this->declare_parameter("dead_load_kg", this->dead_load_kg, param_desc);

    // param_desc.read_only = true;
    // param_desc.description = "The tool center point (TCP) of the module can be read out and written with this parameter. x [mm], y [mm], z [mm], a [°], b [°], c [°]";
    // this->declare_parameter("tool_cent_point", std::vector<int>(6), param_desc);

    // param_desc.read_only = true;
    // param_desc.description = "The center of mass and the mass moments of inertia of the module can be read out with this parameter. a [kg*m2], b [kg*m2], c [kg*m2]";
    // this->declare_parameter("cent_of_mass", std::vector<int>(6), param_desc);

    param_desc.read_only = false;
    param_desc.description = "This parameter can be used to set the traverse path from which a workpiece loss is detected. [mm]";
    this->declare_parameter("wp_lost_dst", 0.0, param_desc);
    this->wp_lost_dst = this->get_parameter("wp_lost_dst").as_double();

    param_desc.read_only = false;
    param_desc.description = "With this parameter the relative position delta between the gripping position and release position can be read out and written. [mm]";
    this->declare_parameter("wp_release_delta", 0.0, param_desc);
    this->wp_release_delta = this->get_parameter("wp_release_delta").as_double();

    param_desc.read_only = false;
    param_desc.description = "With this parameter the tolerance value of the workpiece position window can be read and written. [mm]";
    this->declare_parameter("grp_pos_margin", 0.0, param_desc);
    this->grp_pos_margin = this->get_parameter("grp_pos_margin").as_double();

    param_desc.read_only = true;
    param_desc.description = "With this parameter the maximum physical stroke of the module can be read. The parameter value does not take into account any stroke restrictions resulting from the fingers used. [mm]";
    this->declare_parameter("max_phys_stroke", this->max_phys_stroke, param_desc);

    param_desc.read_only = false;
    param_desc.description = "With this parameter the relative position delta between the pre-position and gripping position can be read out and written. [rad]";
    this->declare_parameter("grp_prepos_delta", 0.0, param_desc);
    this->grp_prepos_delta = this->get_parameter("grp_prepos_delta").as_double();

    param_desc.read_only = false;
    param_desc.description = "The smallest position value that can be approached by the module can be read out and written with this parameter. [mm]";
    this->declare_parameter("min_pos", 0.0, param_desc);
    this->min_pos = this->get_parameter("min_pos").as_double();

    param_desc.read_only = false;
    param_desc.description = "The largest position value that can be approached by the module can be read out and written with this parameter. [mm]";
    this->declare_parameter("max_pos", 0.0, param_desc);
    this->max_pos = this->get_parameter("max_pos").as_double();

    param_desc.read_only = false;
    param_desc.description = "The zero point can be adapted to the application with this parameter. [mm]";
    this->declare_parameter("zero_pos_ofs", 0.0, param_desc);
    this->zero_pos_ofs = this->get_parameter("zero_pos_ofs").as_double();

    param_desc.read_only = true;
    param_desc.description = "The minimum movement/gripping velocity with which the module can be moved can be read out with this parameter. [mm/s]";
    this->declare_parameter("min_vel", this->min_vel, param_desc);

    param_desc.read_only = true;
    param_desc.description = "The maximum positioning speed with which the module can be moved can be read out with this parameter. [mm/s]";
    this->declare_parameter("max_vel", this->max_vel, param_desc);

    param_desc.read_only = true;
    param_desc.description = "The maximum gripping velocity with which the module can be moved can be read out with this parameter. [mm/s]";
    this->declare_parameter("max_grp_vel", this->max_grp_vel, param_desc);

    param_desc.read_only = true;
    param_desc.description = "The minimum gripping force can be read out with this parameter. [N]";
    this->declare_parameter("min_grp_force", this->min_grp_force, param_desc);

    param_desc.read_only = true;
    param_desc.description = "The maximum gripping force can be read out with this parameter. [N]";
    this->declare_parameter("max_grp_force", this->max_grp_force, param_desc);

    try
    {
        writeExplicitEipData(this->si, WP_LOST_DST_ID, VALUE_ATTRIBUTE, this->wp_lost_dst);
        writeExplicitEipData(this->si, WP_RELEASE_DELTA_ID, VALUE_ATTRIBUTE, this->wp_release_delta);
        writeExplicitEipData(this->si, GRP_POS_MARGIN_ID, VALUE_ATTRIBUTE, this->grp_pos_margin);
        writeExplicitEipData(this->si, GRP_PREPOS_DELTA_ID, VALUE_ATTRIBUTE, this->grp_prepos_delta);
        writeExplicitEipData(this->si, MIN_POS_ID, VALUE_ATTRIBUTE, this->min_pos);
        writeExplicitEipData(this->si, MAX_POS_ID, VALUE_ATTRIBUTE, this->max_pos);
        writeExplicitEipData(this->si, ZERO_POS_OFS_ID, VALUE_ATTRIBUTE, this->zero_pos_ofs);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

void SchunkGripper::publishStateUpdate()
{
    this->readImplicitData();

    // Filling msg and publishing it
    schunk_interfaces::msg::SchunkGripperMsg message;
    message.actual_pos = this->actual_pos;

    if (this->warning_bit)
    {
        message.warn_code = this->warning_code;
        message.warn_msg = mapper_warning.at(this->warning_code);
    }

    if (this->error_bit)
    {
        message.error_code = this->error_code;
        message.error_msg = mapper_error.at(this->error_code);
    }

    state_publisher->publish(message);
}

void SchunkGripper::acknowledgeSrv(const TriggerRequestPtr, TriggerResponsePtr res)
{
    this->sendAcknowledgeGripper();
    res->success = true;
}

void SchunkGripper::jogToSrv(const JogToRequestPtr req, JogToResponsePtr res)
{
    int32_t desired_position = static_cast<int32_t>(req->position * 1000);
    int32_t desired_velocity = static_cast<int32_t>(req->velocity * 1000);
    uint8_t motion_type = req->motion_type;
    auto ABSOLUTE_MOTION = schunk_interfaces::srv::JogTo::Request::ABSOLUTE_MOTION;
    auto RELATIVE_MOTION = schunk_interfaces::srv::JogTo::Request::RELATIVE_MOTION;

    res->success = false;

    // Consistencies checks
    if (motion_type != ABSOLUTE_MOTION && motion_type != RELATIVE_MOTION)
    {
        RCLCPP_ERROR(this->get_logger(), "Only available motions are ABSOLUTE or RELATIVE");
        return;
    }
    if (motion_type == ABSOLUTE_MOTION)
    {
        if (desired_position < this->min_pos * 1000 || desired_position > this->max_pos * 1000)
        {
            std::stringstream ss_debug;
            ss_debug << "Desired position is out of bounds. ";
            ss_debug << "min=" << this->min_pos << ", max=" << this->max_pos << ", requested:" << desired_position / 1000;
            RCLCPP_ERROR(this->get_logger(), ss_debug.str().c_str());
            res->debug = ss_debug.str();
            return;
        }
    }
    else
    {
        int32_t relative_position = this->actual_pos * 1000 + desired_position;
        if (relative_position < this->min_pos * 1000 || relative_position > this->max_pos * 1000)
        {
            std::stringstream ss_debug;
            ss_debug << "Desired position is out of bounds. ";
            ss_debug << "min=" << this->min_pos << ", max=" << this->max_pos << ", requested:" << relative_position / 1000;
            RCLCPP_ERROR(this->get_logger(), ss_debug.str().c_str());
            res->debug = ss_debug.str();
            return;
        }
    }
    if (desired_velocity < this->min_vel * 1000 || desired_velocity > this->max_vel * 1000)
    {
        std::stringstream ss_debug;
        ss_debug << "Desired velocity is out of bounds. ";
        ss_debug << "min=" << this->min_vel << ", max=" << this->max_vel << ", requested:" << desired_velocity / 1000;
        RCLCPP_ERROR(this->get_logger(), ss_debug.str().c_str());
        res->debug = ss_debug.str();
        return;
    }
    if (!this->ready_for_operation_bit)
    {
        std::stringstream ss_debug;
        ss_debug << "Gripper is not ready for operation bit";
        RCLCPP_ERROR(this->get_logger(), ss_debug.str().c_str());
        res->success = false;
        res->debug = ss_debug.str();
        return;
    }

    // Setting the bytes in EIP data
    sendDefaultData();

    std::vector<uint8_t> old_bytes = this->dataSent;
    std::vector<uint8_t> new_bytes = std::vector<uint8_t>(16);
    setBit(new_bytes[0], FAST_STOP_BIT_POS);
    setBit(new_bytes[1], motion_type == ABSOLUTE_MOTION ? MOVE_TO_ABSOLUTE_POSITION_BIT_POS : MOVE_TO_RELATIVE_POSITION_BIT_POS);

    std::vector<uint8_t> commands_bytes_new = {new_bytes.begin(), new_bytes.begin() + 4};
    std::vector<uint8_t> commands_bytes_old = {old_bytes.begin(), old_bytes.begin() + 4};

    if (commands_bytes_new == commands_bytes_old) // Whether the command is the same toggle the repeat command toggle -> see documentation for clarifications
    {
        this->repeat_command_toggle_high ? resetBit(new_bytes[0], REPEAT_COMMAND_TOGGLE_BIT_POS) : setBit(new_bytes[0], REPEAT_COMMAND_TOGGLE_BIT_POS);
        this->repeat_command_toggle_high = !this->repeat_command_toggle_high;
    }

    for (int index = 0; index < 4; index++)
    {
        new_bytes[4 + index] = desired_position >> (index * 8) & 0xFF;
        new_bytes[8 + index] = desired_velocity >> (index * 8) & 0xFF;
    }

    // Saving current state of input bits that are going to change after Send
    int32_t prev_command_received_toggle_bit = this->command_received_toggle_bit;

    this->setDataToSend(new_bytes);

    // Waiting from EIP that command has been received
    std::stringstream ss_debug;
    if (this->waitForCommandReceivedToggle(prev_command_received_toggle_bit, 10, ss_debug) == false)
    {
        RCLCPP_ERROR(this->get_logger(), ss_debug.str().c_str());
        res->debug = ss_debug.str();
        return;
    }

    // Waiting for the command to be finished target position reached
    if (this->waitForActionFinish(this->position_reached_bit, 10, ss_debug) == false)
    {
        RCLCPP_ERROR(this->get_logger(), ss_debug.str().c_str());
        res->debug = ss_debug.str();
        return;
    }

    res->success = true;
}

void SchunkGripper::simpleGripSrv(const SimpleGripRequestPtr req, SimpleGripResponsePtr res)
{
    int8_t gripping_force = req->gripping_force;
    int8_t gripping_direction = req->gripping_direction;
    constexpr uint8_t INWARD_GRIP = schunk_interfaces::srv::SimpleGrip::Request::INWARD;
    constexpr uint8_t OUTWARD_GRIP = schunk_interfaces::srv::SimpleGrip::Request::OUTWARD;

    res->success = false;

    // Consistencies checks
    if (gripping_force < 50 || gripping_force > 100)
    {
        std::stringstream ss_debug;
        ss_debug << "Desired gripping_force is out of bounds. ";
        ss_debug << "min=" << 50 << "%, max=" << 100 << "%, requested:" << gripping_force << "%";
        RCLCPP_ERROR(this->get_logger(), ss_debug.str().c_str());
        res->debug = ss_debug.str();
        return;
    }
    if (gripping_direction != INWARD_GRIP && gripping_direction != OUTWARD_GRIP)
    {
        std::stringstream ss_debug;
        ss_debug << "Gripping direction is neither INWARD_GRIP=0 or OUTWARD_GRIP=1";
        RCLCPP_ERROR(this->get_logger(), ss_debug.str().c_str());
        res->debug = ss_debug.str();
        return;
    }

    // Setting the bytes in EIP data
    this->sendDefaultData();

    std::vector<uint8_t> new_bytes = std::vector<uint8_t>(16);

    setBit(new_bytes[0], FAST_STOP_BIT_POS);
    setBit(new_bytes[1], GRIP_WORKPIECE_BIT_POS);

    switch (gripping_direction)
    {
    case INWARD_GRIP:
        setBit(new_bytes[0], GRIP_DIRECTION_BIT_POS);
        break;

    case OUTWARD_GRIP:
        resetBit(new_bytes[0], GRIP_DIRECTION_BIT_POS);
        break;
    }

    for (int index = 0; index < 4; index++)
    {
        new_bytes[4 + index] = 0x00; // Setting position to 0
        new_bytes[8 + index] = 0x00; // Setting velocity to 0
        new_bytes[12 + index] = gripping_force >> (index * 8) & 0xFF;
    }

    int32_t prev_command_received_toggle_bit = this->command_received_toggle_bit;
    this->setDataToSend(new_bytes);

    // Waiting from EIP that command has been received
    std::stringstream ss_debug;
    if (this->waitForCommandReceivedToggle(prev_command_received_toggle_bit, 10, ss_debug) == false)
    {
        RCLCPP_ERROR(this->get_logger(), ss_debug.str().c_str());
        res->debug = ss_debug.str();
        return;
    }

    // Waiting for the command to be finished
    if (this->waitForActionFinish(this->workpiece_gripped_bit, 10, ss_debug) == false)
    {
        RCLCPP_ERROR(this->get_logger(), ss_debug.str().c_str());
        res->debug = ss_debug.str();
        return;
    }

    res->success = true;
}

void SchunkGripper::releaseSrv(const ReleaseRequestPtr, ReleaseResponsePtr res)
{
    res->success = false;

    this->sendDefaultData();

    std::vector<uint8_t> new_bytes = std::vector<uint8_t>(16);

    setBit(new_bytes[0], FAST_STOP_BIT_POS);
    setBit(new_bytes[1], RELEASE_WORKPIECE_BIT_POS);

    this->setDataToSend(new_bytes);

    int32_t prev_command_received_toggle_bit = this->command_received_toggle_bit;
    this->setDataToSend(new_bytes);

    // Waiting from EIP that command has been received
    std::stringstream ss_debug;
    if (this->waitForCommandReceivedToggle(prev_command_received_toggle_bit, 10, ss_debug) == false)
    {
        RCLCPP_ERROR(this->get_logger(), ss_debug.str().c_str());
        res->debug = ss_debug.str();
        return;
    }

    // Waiting for the command to be finished
    if (this->waitForActionFinish(this->position_reached_bit, 10, ss_debug) == false)
    {
        RCLCPP_ERROR(this->get_logger(), ss_debug.str().c_str());
        res->debug = ss_debug.str();
        return;
    }

    res->success = true;
    return;
}

// ------------------------------------------------------------------------------------ //
// ----------------------------------- Constructor ------------------------------------ //
// ------------------------------------------------------------------------------------ //

SchunkGripper::SchunkGripper() : Node("gripper")
{
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.read_only = false;

    this->declare_parameter("gripper_ip", std::string("0.0.0.0"), param_desc);
    this->declare_parameter("gripper_name", std::string("gripper"), param_desc);

    this->node_name = this->get_parameter("gripper_name").as_string();
    this->gripper_ip = this->get_parameter("gripper_ip").as_string();

    // --- Ethernet/IP --- //
    // Setting Loggin level
    Logger::setLogLevel(LogLevel::WARNING);

    // Enstablish explicit connection and getting the initial data
    this->si = std::make_shared<eipScanner::SessionInfo>(this->gripper_ip, 0xAF12);

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

    // Setting the handlers
    eipScanner::IOConnection::SendDataHandle
        sendDataHandler = [](std::vector<uint8_t> data)
    {
        std::ostringstream ss;
        for (auto &byte : data)
            ss << "[" << std::hex << (int)byte << "]";
    };
    eipScanner::IOConnection::ReceiveDataHandle receiveHandler = [this](eipScanner::cip::CipUdint, eipScanner::cip::CipUdint, std::vector<uint8_t> data)
    { this->dataReceived = data; };
    eipScanner::IOConnection::CloseHandle closeConnectionHandler = []()
    { Logger(LogLevel::INFO) << "Closed"; };
    this->setHandlers(sendDataHandler, receiveHandler, closeConnectionHandler);

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

    // --- ROS --- //
    // Callback Group
    this->callback_group_reentrant = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Parameters declaration + getting
    this->declareParameters();

    // Publishers
    state_publisher = this->create_publisher<SchunkGripperMsg>(this->node_name + "_state", 10);

    // Services
    this->acknowledge_srv = this->create_service<Trigger>(this->node_name + "/acknowledge", std::bind(&SchunkGripper::acknowledgeSrv, this, _1, _2), rmw_qos_profile_services_default, this->callback_group_reentrant);
    this->jog_to_srv = this->create_service<JogTo>(this->node_name + "/jog_to", std::bind(&SchunkGripper::jogToSrv, this, _1, _2), rmw_qos_profile_services_default, this->callback_group_reentrant);
    this->simple_grip_srv = this->create_service<SimpleGrip>(this->node_name + "/simple_grip", std::bind(&SchunkGripper::simpleGripSrv, this, _1, _2), rmw_qos_profile_services_default, this->callback_group_reentrant);
    this->release_srv = this->create_service<Release>(this->node_name + "/release", std::bind(&SchunkGripper::releaseSrv, this, _1, _2), rmw_qos_profile_services_default, this->callback_group_reentrant);

    // Timers callbacks
    timer = this->create_wall_timer(100ms, std::bind(&SchunkGripper::publishStateUpdate, this));

    // Resetting the gripper to default settings (acknowledge)
    this->sendAcknowledgeGripper();
}