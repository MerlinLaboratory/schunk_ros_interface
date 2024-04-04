#include "../include/schunk_eip_interface.hpp"

// ------------------------------------------------------------------------------------ //
// ----------------------- Functions for Implicit Communication ----------------------- //
// ------------------------------------------------------------------------------------ //
void SchunkGripper::SetDataToSend(std::vector<uint8_t> data)
{
    if (auto ptr = this->io.lock())
    {
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

// ------------------------------------------------------------------------------------ //
// ---------------------------------- ROS Functions ----------------------------------- //
// ------------------------------------------------------------------------------------ //

void SchunkGripper::publishStateUpdate()
{
    // Testing TODO: Remove me plz 
    std::ostringstream ss;
    for (auto &byte : this->dataReceived)
        ss << "[" << std::hex << (int)byte << "]";
    // Logger(LogLevel::INFO) << "Received: " << ss.str();
    // -------

    // Filling msg and publishing it
    schunk_interface::msg::SchunkGripperMsg message;
    message.actual_pos.data = this->actual_pos;
    message.actual_vel.data = this->actual_vel;
    message.grp_prehold_time.data = this->grp_prehold_time;
    message.dead_load_kg.data = this->dead_load_kg;
    // message.tool_cent_point.data = this->tool_cent_point; <- not working
    // message.cent_of_mass.data = this->cent_of_mass; <- not working
    message.wp_lost_dst.data = this->wp_lost_dst;
    message.wp_release_delta.data = this->wp_release_delta;
    message.grp_pos_margin.data = this->grp_pos_margin;
    message.max_phys_stroke.data = this->max_phys_stroke;
    message.grp_prepos_delta.data = this->grp_prepos_delta;
    message.min_pos.data = this->min_pos;
    message.max_pos.data = this->max_pos;
    message.zero_pos_ofs.data = this->zero_pos_ofs;
    message.min_vel.data = this->min_vel;
    message.max_vel.data = this->max_vel;
    message.max_grp_vel.data = this->max_grp_vel;
    message.min_grp_force.data = this->min_grp_force;
    message.max_grp_force.data = this->max_grp_force;
    // message.serial_no_num.data = this->serial_no_num; <- not working
    // message.mac_addr.data = this->mac_addr; <- not working
    message.enable_softreset.data = this->enable_softreset;

    state_publisher->publish(message);
}

void SchunkGripper::JogToSrv(const JogToRequestPtr req, JogToResponsePtr res)
{
    int32_t desired_position = static_cast<int32_t>(req->position.data * 1000);
    int32_t desired_velocity = static_cast<int32_t>(req->velocity.data * 1000);
    uint8_t motion_type = req->motion_type.data;

    res->success = false;

    // Check whether requested position and velocities are outside of bounds
    if(desired_position < this->min_pos * 1000 || desired_position > this->max_pos * 1000)
    {
        RCLCPP_WARN(this->get_logger(), "Desired position is out of bounds");
        RCLCPP_WARN(this->get_logger(), "min=%f and max=%f", this->min_pos, this->max_pos);
        return;
    }
    if(desired_velocity < this->min_vel * 1000 || desired_velocity > this->max_vel * 1000)
    {
        RCLCPP_WARN(this->get_logger(), "Desired velocity is out of bounds");
        RCLCPP_WARN(this->get_logger(), "min=%f and max=%f", this->min_vel, this->max_vel);
        return;
    }

    // Check that motion_type is ABSOLUTE or RELATIVE
    auto ABSOLUTE_MOTION = schunk_interface::srv::JogTo::Request::ABSOLUTE_MOTION;
    auto RELATIVE_MOTION = schunk_interface::srv::JogTo::Request::RELATIVE_MOTION;
    if(motion_type != ABSOLUTE_MOTION && motion_type != RELATIVE_MOTION)
    {
        RCLCPP_WARN(this->get_logger(), "Only available motions are ABSOLUTE or RELATIVE");
        return;
    }

    std::vector<uint8_t> bytes = std::vector<uint8_t>(16);

    // Setting the bytes in EIP data
    bytes[0] |= SET_1 << FAST_STOP_BIT_POS;
    bytes[1] |= SET_1 << MOVE_TO_ABSOLUTE_POSITION_BIT_POS;
    // bytes[0] |= (repeat_command_toggle? SET_0:SET_1 << REPEAT_COMMAND_TOGGLE_BIT_POS); // TODO: check me

    for(int index=0; index<4; index++)
    {
        bytes[4 + index] = desired_position >> (index * 8) & 0xFF;
        bytes[8 + index] = desired_velocity >> (index * 8) & 0xFF;
    }

    this->SetDataToSend(bytes);

    // Waiting from EIP that command has been received

    // Testing
    auto start = std::chrono::system_clock::now();
    auto end = start + 5s;

    // Waiting from EIP that command is finished
    while(std::chrono::system_clock::now() < end)
    {
        RCLCPP_INFO(this->get_logger(), "Stucked in the service server!");
        std::this_thread::sleep_for(100ms);
    }

    res->success = true;
}
