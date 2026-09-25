#include "zemo_hardware/servo_system/servo_interface.hpp"


namespace zemo_hardware{

hardware_interface::CallbackReturn ServoInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params){
    if(SystemInterface::on_init(params) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;

    const auto& hw_params = params.hardware_info.hardware_parameters;

    port_ = hw_params.at("port");
    baudrate_ = std::stoul(hw_params.at("baudrate"));
    protocol_type_ = hw_params.at("protocol_type");
    tx_enable_pin_ = (uint8_t)std::stoul(hw_params.at("tx_en_pin"));
    rx_enable_pin_ = (uint8_t)std::stoul(hw_params.at("rx_en_pin"));

    // This part of the code find the protocol from the allowed list

    for(auto& x : protocol_type_) x = tolower(x);   // c++ couldn't find a better way to make all charachters lower case
    auto it = allowed_protocols.find(protocol_type_);

    if(it == allowed_protocols.end()){
        RCLCPP_ERROR(rclcpp::get_logger("zemo_hardware"), "Invalid protocol type %s", protocol_type_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    protocol_ = it->second;

    size_t number_of_joints = params.hardware_info.joints.size();
    joint_list_.resize(number_of_joints);

    for(size_t i = 0; i < number_of_joints; i++){
        const auto& joint_info = params.hardware_info.joints[i];

        auto mode = std::stoul(joint_info.parameters.at("mode"));
        uint8_t id = (uint8_t)std::stoul(joint_info.parameters.at("id"));

        joint_list_[i].id = id;
        joint_list_[i].mode = static_cast<ControlMode>(mode);

        if(mode > 1 || id <= 0){
            RCLCPP_ERROR(rclcpp::get_logger("zemo_hardware"), "Invalid control mode %lu or id %u for joint %s", mode, id, joint_info.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // Fancy union typization
    switch(protocol_){
        case ProtocolType::SMS_STS:
            driver_.emplace<SMS_STS>();
            break;

        case ProtocolType::HLSCL:
            driver_.emplace<HLSCL>();
            break;

        case ProtocolType::SCS0009:
            driver_.emplace<SCS0009>();
            break;

        case ProtocolType::SCSCL:
            driver_.emplace<SCSCL>();
            break;
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/){
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/){
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/){
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/){
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ServoInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/){
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ServoInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/){
    return hardware_interface::return_type::OK;
}

} // namespace zemo_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(zemo_hardware::ServoInterface, hardware_interface::SystemInterface)