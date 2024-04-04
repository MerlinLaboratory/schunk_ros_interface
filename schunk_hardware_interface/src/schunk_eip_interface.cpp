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
dataType ReadEipData(std::shared_ptr<eipScanner::SessionInfo> si_async, eipScanner::cip::CipUint instance_id, eipScanner::cip::CipUint attribute_name)
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
void WriteEipData(std::shared_ptr<eipScanner::SessionInfo> si_async, eipScanner::cip::CipUint instance_id, eipScanner::cip::CipUint attribute_name)
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
        this->grp_prehold_time = ReadEipData<eipScanner::cip::CipWord>(this->si, GRP_PREHOLD_TIME_ID, VALUE_ATTRIBUTE);
        this->dead_load_kg = ReadEipData<eipScanner::cip::CipReal>(this->si, DEAD_LOAD_KG_ID, VALUE_ATTRIBUTE);
        // this->tool_cent_point = ReadEipData<std::vector<eipScanner::cip::CipReal>>(this->si, DEAD_LOAD_KG_ID, VALUE_ATTRIBUTE); <-- not working
        // this->cent_of_mass = ReadEipData<std::vector<eipScanner::cip::CipReal>>(this->si, CENT_OF_MASS_ID, VALUE_ATTRIBUTE); <-- not working
        this->wp_lost_dst = ReadEipData<eipScanner::cip::CipReal>(this->si, WP_LOST_DST_ID, VALUE_ATTRIBUTE);
        this->wp_release_delta = ReadEipData<eipScanner::cip::CipReal>(this->si, WP_RELEASE_DELTA_ID, VALUE_ATTRIBUTE);
        this->grp_pos_margin = ReadEipData<eipScanner::cip::CipReal>(this->si, GRP_POS_MARGIN_ID, VALUE_ATTRIBUTE);
        this->max_phys_stroke = ReadEipData<eipScanner::cip::CipReal>(this->si, MAX_PHYS_STROKE_ID, VALUE_ATTRIBUTE);
        this->grp_prepos_delta = ReadEipData<eipScanner::cip::CipReal>(this->si, GRP_PREPOS_DELTA_ID, VALUE_ATTRIBUTE);
        this->min_pos = ReadEipData<eipScanner::cip::CipReal>(this->si, MIN_POS_ID, VALUE_ATTRIBUTE);
        this->max_pos = ReadEipData<eipScanner::cip::CipReal>(this->si, MAX_POS_ID, VALUE_ATTRIBUTE);
        this->zero_pos_ofs = ReadEipData<eipScanner::cip::CipReal>(this->si, ZERO_POS_OFS_ID, VALUE_ATTRIBUTE);
        this->min_vel = ReadEipData<eipScanner::cip::CipReal>(this->si, MIN_VEL_ID, VALUE_ATTRIBUTE);
        this->max_vel = ReadEipData<eipScanner::cip::CipReal>(this->si, MAX_VEL_ID, VALUE_ATTRIBUTE);
        this->max_grp_vel = ReadEipData<eipScanner::cip::CipReal>(this->si, MAX_GRP_VEL_ID, VALUE_ATTRIBUTE);
        this->min_grp_force = ReadEipData<eipScanner::cip::CipReal>(this->si, MIN_GRP_FORCE_ID, VALUE_ATTRIBUTE);
        this->max_grp_force = ReadEipData<eipScanner::cip::CipReal>(this->si, MAX_GRP_FORCE_ID, VALUE_ATTRIBUTE);
        // this->serial_no_num = ReadEipData<std::vector<eipScanner::cip::CipByte>>(this->si_async, SERIAL_NO_NUM_ID, VALUE_ATTRIBUTE); <-- not working
        this->mac_addr = ReadEipData<eipScanner::cip::CipReal>(this->si, MAC_ADDR_ID, VALUE_ATTRIBUTE);
        this->enable_softreset = ReadEipData<eipScanner::cip::CipReal>(this->si, ENABLE_SOFTRESET_ID, VALUE_ATTRIBUTE);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

// ------------------------------------------------------------------------------------ //
// ------------------------------ Schunk Class Functions ------------------------------ //
// ------------------------------------------------------------------------------------ //

void SchunkGripper::publishStateUpdate()
{
    // Testing TODO: Remove me plz 
    std::ostringstream ss;
    for (auto &byte : this->dataReceived)
        ss << "[" << std::hex << (int)byte << "]";
    Logger(LogLevel::INFO) << "Received: " << ss.str();
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
