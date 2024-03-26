#include "../include/schunk_eip_interface.hpp"

template <typename dataType>
dataType ReadEipData(std::shared_ptr<eipScanner::SessionInfo> si, eipScanner::cip::CipUint instance_id, eipScanner::cip::CipUint attribute_name)
{
    // Read Parameter Class Descriptor
    eipScanner::MessageRouter messageRouter;
    MessageRouterResponse response = messageRouter.sendRequest(si, ServiceCodes::GET_ATTRIBUTE_SINGLE, EPath(CLASS, instance_id, attribute_name));

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

void SchunkGripper::publishStateUpdate()
{
    // Update class data
    this->getEipData();

    schunk_interface::msg::SchunkGripperMsg message;
    // state_publisher->publish(message);
}

void SchunkGripper::getInitialEipData()
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
        // this->serial_no_num = ReadEipData<std::vector<eipScanner::cip::CipByte>>(this->si, SERIAL_NO_NUM_ID, VALUE_ATTRIBUTE); <-- not working
        this->mac_addr = ReadEipData<eipScanner::cip::CipReal>(this->si, MAC_ADDR_ID, VALUE_ATTRIBUTE);
        this->enable_softreset = ReadEipData<eipScanner::cip::CipReal>(this->si, ENABLE_SOFTRESET_ID, VALUE_ATTRIBUTE);
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
        this->actual_pos = ReadEipData<CipReal>(this->si, ACTUAL_POS_ID, VALUE_ATTRIBUTE);
        this->actual_vel = ReadEipData<CipReal>(this->si, ACTUAL_VEL_ID, VALUE_ATTRIBUTE);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

