#include "../include/schunk_eip_interface.hpp"

// ------------------------------------------------------------------------------------ //
// ----------------------- Functions for Implicit Communication ----------------------- //
// ------------------------------------------------------------------------------------ //
void SchunkGripper::SetDataToSend(std::vector<uint8_t> data)
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

void SchunkGripper::SetHandlers(eipScanner::IOConnection::SendDataHandle sendHandler,
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

// ------------------------------------------------------------------------------------ //
// ----------------------- Functions for Explicit Communication ----------------------- //
// ------------------------------------------------------------------------------------ //

template <typename dataType>
dataType ReadExplicitEipData(std::shared_ptr<eipScanner::SessionInfo> si_async, eipScanner::cip::CipUint instance_id, eipScanner::cip::CipUint attribute_name)
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

// TODO: place it in a service and check that it works (turn it into a template?)
void WriteExplicitEipData(std::shared_ptr<eipScanner::SessionInfo> si_async, eipScanner::cip::CipUint instance_id, eipScanner::cip::CipUint attribute_name)
{
    // Read Parameter Class Descriptor
    eipScanner::MessageRouter messageRouter;
    eipScanner::cip::MessageRouterResponse response = messageRouter.sendRequest(si_async, eipScanner::cip::ServiceCodes::SET_ATTRIBUTE_SINGLE, eipScanner::cip::EPath(CLASS, instance_id, attribute_name));

    if (response.getGeneralStatusCode() != eipScanner::cip::GeneralStatusCodes::SUCCESS)
    {
        Logger(LogLevel::ERROR) << "Failed to write";
        logGeneralAndAdditionalStatus(response);

        throw std::runtime_error("Failed to write attribute id:" + std::to_string(attribute_name) + " instance id:" + std::to_string(instance_id));
    }

    return;
}

void SchunkGripper::getExplicitEipData()
{
    try
    {
        this->grp_prehold_time = ReadExplicitEipData<eipScanner::cip::CipWord>(this->si, GRP_PREHOLD_TIME_ID, VALUE_ATTRIBUTE);
        this->dead_load_kg = ReadExplicitEipData<CipReal>(this->si, DEAD_LOAD_KG_ID, VALUE_ATTRIBUTE);
        // this->tool_cent_point = ReadExplicitEipData<std::vector<CipReal>>(this->si, DEAD_LOAD_KG_ID, VALUE_ATTRIBUTE); <-- not working
        // this->cent_of_mass = ReadExplicitEipData<std::vector<CipReal>>(this->si, CENT_OF_MASS_ID, VALUE_ATTRIBUTE); <-- not working
        this->wp_lost_dst = ReadExplicitEipData<CipReal>(this->si, WP_LOST_DST_ID, VALUE_ATTRIBUTE);
        this->wp_release_delta = ReadExplicitEipData<CipReal>(this->si, WP_RELEASE_DELTA_ID, VALUE_ATTRIBUTE);
        this->grp_pos_margin = ReadExplicitEipData<CipReal>(this->si, GRP_POS_MARGIN_ID, VALUE_ATTRIBUTE);
        this->max_phys_stroke = ReadExplicitEipData<CipReal>(this->si, MAX_PHYS_STROKE_ID, VALUE_ATTRIBUTE);
        this->grp_prepos_delta = ReadExplicitEipData<CipReal>(this->si, GRP_PREPOS_DELTA_ID, VALUE_ATTRIBUTE);
        this->min_pos = ReadExplicitEipData<CipReal>(this->si, MIN_POS_ID, VALUE_ATTRIBUTE);
        this->max_pos = ReadExplicitEipData<CipReal>(this->si, MAX_POS_ID, VALUE_ATTRIBUTE);
        this->zero_pos_ofs = ReadExplicitEipData<CipReal>(this->si, ZERO_POS_OFS_ID, VALUE_ATTRIBUTE);
        this->min_vel = ReadExplicitEipData<CipReal>(this->si, MIN_VEL_ID, VALUE_ATTRIBUTE);
        this->max_vel = ReadExplicitEipData<CipReal>(this->si, MAX_VEL_ID, VALUE_ATTRIBUTE);
        this->max_grp_vel = ReadExplicitEipData<CipReal>(this->si, MAX_GRP_VEL_ID, VALUE_ATTRIBUTE);
        this->min_grp_force = ReadExplicitEipData<CipReal>(this->si, MIN_GRP_FORCE_ID, VALUE_ATTRIBUTE);
        this->max_grp_force = ReadExplicitEipData<CipReal>(this->si, MAX_GRP_FORCE_ID, VALUE_ATTRIBUTE);
        // this->serial_no_num = ReadExplicitEipData<std::vector<eipScanner::cip::CipByte>>(this->si_async, SERIAL_NO_NUM_ID, VALUE_ATTRIBUTE); <-- not working
        this->mac_addr = ReadExplicitEipData<CipReal>(this->si, MAC_ADDR_ID, VALUE_ATTRIBUTE);
        this->enable_softreset = ReadExplicitEipData<CipReal>(this->si, ENABLE_SOFTRESET_ID, VALUE_ATTRIBUTE);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

void SetBit(uint8_t& byte, const uint8_t bit_position)
{
    byte |= 1 << bit_position;
}

void ResetBit(uint8_t& byte, const uint8_t bit_position)
{
    byte &= ~(1 << bit_position);
}

bool isBitHigh(uint8_t byte, const uint8_t bit_position)
{
    byte >> bit_position & 0x01;
    return byte? true:false;
}

// ------------------------------------------------------------------------------------ //
// ---------------------------------- ROS Functions ----------------------------------- //
// ------------------------------------------------------------------------------------ //

void SchunkGripper::DecodeImplicitData()
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

void SchunkGripper::publishStateUpdate()
{
    this->DecodeImplicitData();

    // Filling msg and publishing it
    schunk_interface::msg::SchunkGripperMsg message;
    message.actual_pos = this->actual_pos;

    if(this->warning_bit){
        message.warn_code = this->warning_code;
        message.warn_msg = mapper_warning.at(this->warning_code);
    }

    if(this->error_bit){
        message.error_code = this->error_code;
        message.error_msg = mapper_error.at(this->error_code);
    }

    // message.actual_vel.data = this->actual_vel;
    // message.grp_prehold_time.data = this->grp_prehold_time; // TODO: Create a ROSparam
    // message.dead_load_kg.data = this->dead_load_kg;         // TODO: Create a ROSparam
    // message.tool_cent_point.data = this->tool_cent_point; <- not working
    // message.cent_of_mass.data = this->cent_of_mass; <- not working
    // message.wp_lost_dst.data = this->wp_lost_dst;           // TODO: Create a ROSparam
    // message.wp_release_delta.data = this->wp_release_delta; // TODO: Create a ROSparam
    // message.grp_pos_margin.data = this->grp_pos_margin;     // TODO: Create a ROSparam
    // message.max_phys_stroke.data = this->max_phys_stroke;   // TODO: Create a ROSparam
    // message.grp_prepos_delta.data = this->grp_prepos_delta; // TODO: Create a ROSparam
    // message.min_pos.data = this->min_pos;                   // TODO: Create a ROSparam
    // message.max_pos.data = this->max_pos;                   // TODO: Create a ROSparam
    // message.zero_pos_ofs.data = this->zero_pos_ofs;         // TODO: Create a ROSparam
    // message.min_vel.data = this->min_vel;                   // TODO: Create a ROSparam
    // message.max_vel.data = this->max_vel;                   // TODO: Create a ROSparam
    // message.max_grp_vel.data = this->max_grp_vel;           // TODO: Create a ROSparam
    // message.min_grp_force.data = this->min_grp_force;       // TODO: Create a ROSparam
    // message.max_grp_force.data = this->max_grp_force;       // TODO: Create a ROSparam
    // message.serial_no_num.data = this->serial_no_num; <- not working
    // message.mac_addr.data = this->mac_addr; <- not working
    // message.enable_softreset.data = this->enable_softreset; // TODO: Create a ROSparam

    state_publisher->publish(message);
}

void SchunkGripper::AcknowledgeSrv(const TriggerRequestPtr req, TriggerResponsePtr res)
{
    std::vector<uint8_t> bytes = this->dataSent;

    // Resetting the ACKNOWLEDGE_BIT first if necessary
    if( isBitHigh(bytes[0], ACKNOWLEDGE_BIT_POS) == true )
    {
        ResetBit(bytes[0], ACKNOWLEDGE_BIT_POS);
        this->SetDataToSend(bytes);
        std::this_thread::sleep_for(500ms);
    }

    SetBit(bytes[0], ACKNOWLEDGE_BIT_POS);
    this->SetDataToSend(bytes);

    res->success = true;
}

void SchunkGripper::JogToSrv(const JogToRequestPtr req, JogToResponsePtr res)
{
    int32_t desired_position = static_cast<int32_t>(req->position.data * 1000);
    int32_t desired_velocity = static_cast<int32_t>(req->velocity.data * 1000);
    uint8_t motion_type = req->motion_type.data;

    res->success = false;

    // Initial check
    if (desired_position < this->min_pos * 1000 || desired_position > this->max_pos * 1000)
    {
        RCLCPP_ERROR(this->get_logger(), "Desired position is out of bounds");
        RCLCPP_ERROR(this->get_logger(), "min=%f and max=%f", this->min_pos, this->max_pos);
        return;
    }
    if (desired_velocity < this->min_vel * 1000 || desired_velocity > this->max_vel * 1000)
    {
        RCLCPP_ERROR(this->get_logger(), "Desired velocity is out of bounds");
        RCLCPP_ERROR(this->get_logger(), "min=%f and max=%f", this->min_vel, this->max_vel);
        return;
    }
    auto ABSOLUTE_MOTION = schunk_interface::srv::JogTo::Request::ABSOLUTE_MOTION;
    auto RELATIVE_MOTION = schunk_interface::srv::JogTo::Request::RELATIVE_MOTION;
    if (motion_type != ABSOLUTE_MOTION && motion_type != RELATIVE_MOTION)
    {
        RCLCPP_ERROR(this->get_logger(), "Only available motions are ABSOLUTE or RELATIVE");
        return;
    }

    // Check if gripper is ready for operation
    if(!this->ready_for_operation_bit)
    {
        RCLCPP_ERROR(this->get_logger(), "Ready for operation bit");
        res->success = false;
        return;
    }

    // Setting the bytes in EIP data
    std::vector<uint8_t> output_bytes = std::vector<uint8_t>(16);
    SetBit(output_bytes[0], FAST_STOP_BIT_POS);
    SetBit(output_bytes[1], motion_type == ABSOLUTE_MOTION? MOVE_TO_ABSOLUTE_POSITION_BIT_POS:MOVE_TO_RELATIVE_POSITION_BIT_POS);

    std::vector<uint8_t> commands_bytes = {output_bytes.begin(), output_bytes.begin() + 4};
    std::vector<uint8_t> commands_dataSent = {this->dataSent.begin(), this->dataSent.begin() + 4};
    
    if(commands_dataSent == commands_bytes) // Whether the command is the same toggle the repeat command toggle -> see documentation for clarifications
    {
        this->repeat_command_toggle_high? ResetBit(output_bytes[0], REPEAT_COMMAND_TOGGLE_BIT_POS) : SetBit(output_bytes[0], REPEAT_COMMAND_TOGGLE_BIT_POS);
        this->repeat_command_toggle_high =! this->repeat_command_toggle_high;
    }

    for (int index = 0; index < 4; index++)
    {
        output_bytes[4 + index] = desired_position >> (index * 8) & 0xFF;
        output_bytes[8 + index] = desired_velocity >> (index * 8) & 0xFF;
    }

    // Saving current state of input bits that are going to change after Send
    uint8_t prev_command_received_toggle_bit = this->command_received_toggle_bit;

    this->SetDataToSend(output_bytes);

    // Waiting from EIP that command has been received
    std::chrono::duration<int> timeout(10);

    auto start = std::chrono::system_clock::now();
    while(prev_command_received_toggle_bit == this->command_received_toggle_bit)
    {
        auto now = std::chrono::system_clock::now();
        auto elapesed_time = now - start;

        if(elapesed_time > timeout)
        {
            RCLCPP_ERROR(this->get_logger(), "Command not received by gripper");
            res->success = false;
            return;
        }

        std::this_thread::sleep_for(300ms);
    }

    // Waiting for the command to be finished target position reached setting the control bit "stop"
    start = std::chrono::system_clock::now();
    while(this->position_reached_bit == false)
    {
        auto now = std::chrono::system_clock::now();
        auto elapesed_time = now - start;

        if(elapesed_time > timeout)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot finish withing timeout");
            res->success = false;
            return;
        }

        std::this_thread::sleep_for(300ms);
    }

    res->success = true;
}
