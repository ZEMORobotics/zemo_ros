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

    size_t number_of_joints = params.hardware_info.joints.size();
    joint_list_.resize(number_of_joints);

    for(size_t i = 0; i < number_of_joints; i++){
        const auto& joint_info = params.hardware_info.joints[i];

        joint_list_[i].id = (uint8_t)std::stoul(joint_info.parameters.at("id"));
        auto mode = std::stoul(joint_info.parameters.at("mode"));

        if(mode > 1){
            RCLCPP_ERROR(rclcpp::get_logger("zemo_hardware"), "Invalid control mode %lu for joint %s", mode, joint_info.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
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