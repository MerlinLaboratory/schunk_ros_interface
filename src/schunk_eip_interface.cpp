#include "../include/schunk_ros_interface/schunk_eip_interface.hpp"

void SchunkGripper::publishStateUpdate()
{
    std_msgs::msg::String message;
    message.data = "Ano Anale!";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // state_publisher->publish(message);
}

MessageRouterResponse SchunkGripper::readEipData(eipScanner::cip::CipUint instance_id, eipScanner::cip::CipUint attribute_name)
{
    // Read Parameter Class Descriptor
    MessageRouterResponse response = this->messageRouter.sendRequest(this->si, ServiceCodes::GET_ATTRIBUTE_SINGLE, EPath(CLASS, instance_id, attribute_name));

    if (response.getGeneralStatusCode() != GeneralStatusCodes::SUCCESS) {
        Logger(LogLevel::ERROR) << "Failed to read";
        logGeneralAndAdditionalStatus(response);

        throw std::runtime_error("Failed to read attribute id:" + std::to_string(attribute_name) + " instance id:" + std::to_string(instance_id));
    }

    return response;
}

void SchunkGripper::getInitialEipData()
{



}