#include "../include/schunk_eip_interface.hpp"

template <typename dataType>
dataType ReadEipData(std::shared_ptr<eipScanner::SessionInfo> si_async, eipScanner::cip::CipUint instance_id, eipScanner::cip::CipUint attribute_name)
{
    // Read Parameter Class Descriptor
    eipScanner::MessageRouter messageRouter;
    MessageRouterResponse response = messageRouter.sendRequest(si_async, ServiceCodes::GET_ATTRIBUTE_SINGLE, EPath(CLASS, instance_id, attribute_name));

    if (response.getGeneralStatusCode() != GeneralStatusCodes::SUCCESS) {
        Logger(LogLevel::ERROR) << "Failed to read";
        logGeneralAndAdditionalStatus(response);

        throw std::runtime_error("Failed to read attribute id:" + std::to_string(attribute_name) + " instance id:" + std::to_string(instance_id));
    }

    Buffer buffer = Buffer(response.getData());
    dataType data;
    buffer >> data;

    return data;
}

void WriteEipData(std::shared_ptr<eipScanner::SessionInfo> si_async, eipScanner::cip::CipUint instance_id, eipScanner::cip::CipUint attribute_name)
{
    // Read Parameter Class Descriptor
    eipScanner::MessageRouter messageRouter;
    MessageRouterResponse response = messageRouter.sendRequest(si_async, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(CLASS, instance_id, attribute_name));

    if (response.getGeneralStatusCode() != GeneralStatusCodes::SUCCESS) {
        Logger(LogLevel::ERROR) << "Failed to write";
        logGeneralAndAdditionalStatus(response);

        throw std::runtime_error("Failed to write attribute id:" + std::to_string(attribute_name) + " instance id:" + std::to_string(instance_id));
    }

    return;
}

// ------------------------------------------------------------------------------------ //
// ------------------------------ Schunk Class Functions ------------------------------ //
// ------------------------------------------------------------------------------------ //

void SchunkGripper::publishStateUpdate()
{
    // Update class data
    this->getEipData();

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

void SchunkGripper::getInitialEipData()
{
    try
    {
        this->grp_prehold_time = ReadEipData<eipScanner::cip::CipWord>(this->si_async, GRP_PREHOLD_TIME_ID, VALUE_ATTRIBUTE);
        this->dead_load_kg = ReadEipData<eipScanner::cip::CipReal>(this->si_async, DEAD_LOAD_KG_ID, VALUE_ATTRIBUTE);
        // this->tool_cent_point = ReadEipData<std::vector<eipScanner::cip::CipReal>>(this->si_async, DEAD_LOAD_KG_ID, VALUE_ATTRIBUTE); <-- not working
        // this->cent_of_mass = ReadEipData<std::vector<eipScanner::cip::CipReal>>(this->si_async, CENT_OF_MASS_ID, VALUE_ATTRIBUTE); <-- not working
        this->wp_lost_dst = ReadEipData<eipScanner::cip::CipReal>(this->si_async, WP_LOST_DST_ID, VALUE_ATTRIBUTE); 
        this->wp_release_delta = ReadEipData<eipScanner::cip::CipReal>(this->si_async, WP_RELEASE_DELTA_ID, VALUE_ATTRIBUTE);
        this->grp_pos_margin = ReadEipData<eipScanner::cip::CipReal>(this->si_async, GRP_POS_MARGIN_ID, VALUE_ATTRIBUTE);
        this->max_phys_stroke = ReadEipData<eipScanner::cip::CipReal>(this->si_async, MAX_PHYS_STROKE_ID, VALUE_ATTRIBUTE);
        this->grp_prepos_delta = ReadEipData<eipScanner::cip::CipReal>(this->si_async, GRP_PREPOS_DELTA_ID, VALUE_ATTRIBUTE);
        this->min_pos = ReadEipData<eipScanner::cip::CipReal>(this->si_async, MIN_POS_ID, VALUE_ATTRIBUTE);
        this->max_pos = ReadEipData<eipScanner::cip::CipReal>(this->si_async, MAX_POS_ID, VALUE_ATTRIBUTE);
        this->zero_pos_ofs = ReadEipData<eipScanner::cip::CipReal>(this->si_async, ZERO_POS_OFS_ID, VALUE_ATTRIBUTE);
        this->min_vel = ReadEipData<eipScanner::cip::CipReal>(this->si_async, MIN_VEL_ID, VALUE_ATTRIBUTE);
        this->max_vel = ReadEipData<eipScanner::cip::CipReal>(this->si_async, MAX_VEL_ID, VALUE_ATTRIBUTE);
        this->max_grp_vel = ReadEipData<eipScanner::cip::CipReal>(this->si_async, MAX_GRP_VEL_ID, VALUE_ATTRIBUTE);
        this->min_grp_force = ReadEipData<eipScanner::cip::CipReal>(this->si_async, MIN_GRP_FORCE_ID, VALUE_ATTRIBUTE);
        this->max_grp_force = ReadEipData<eipScanner::cip::CipReal>(this->si_async, MAX_GRP_FORCE_ID, VALUE_ATTRIBUTE);
        // this->serial_no_num = ReadEipData<std::vector<eipScanner::cip::CipByte>>(this->si_async, SERIAL_NO_NUM_ID, VALUE_ATTRIBUTE); <-- not working
        this->mac_addr = ReadEipData<eipScanner::cip::CipReal>(this->si_async, MAC_ADDR_ID, VALUE_ATTRIBUTE);
        this->enable_softreset = ReadEipData<eipScanner::cip::CipReal>(this->si_async, ENABLE_SOFTRESET_ID, VALUE_ATTRIBUTE);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

void SchunkGripper::getEipData()
{
    try
    {
        this->actual_pos = ReadEipData<CipReal>(this->si_async, ACTUAL_POS_ID, VALUE_ATTRIBUTE);
        this->actual_vel = ReadEipData<CipReal>(this->si_async, ACTUAL_VEL_ID, VALUE_ATTRIBUTE);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    // -------- Testing ---------- // 

    if(temp)
    {
        // WriteEipData(this->si_async, );
        temp = !temp;
    }
    else
    {
        // WriteEipData(this->si_async, );
        temp = !temp;
    }
}

