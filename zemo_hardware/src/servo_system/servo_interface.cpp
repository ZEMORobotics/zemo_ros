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

    // This part of the code finds the protocol from the allowed list.

    for(auto& x : protocol_type_) x = tolower(x);   // c++ couldn't find a better way to make all characters lower case
    auto it = allowed_protocols.find(protocol_type_);

    if(it == allowed_protocols.end()){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Invalid protocol type %s", protocol_type_.c_str());
        return CallbackReturn::ERROR;
    }
    protocol_ = it->second;

    size_t number_of_joints = params.hardware_info.joints.size();
    joint_list_.resize(number_of_joints);

    for(size_t i = 0; i < number_of_joints; i++){
        const auto& joint_info = params.hardware_info.joints[i];

        auto mode = std::stoul(joint_info.parameters.at("mode"));
        auto id = std::stoul(joint_info.parameters.at("id"));

        joint_list_[i].id = (uint8_t)id;
        joint_list_[i].mode = static_cast<ControlMode>(mode);

        // In most protocols 0xfe (254) is broadcast id, so we allow 253 and below.
        if(mode > 1 || id > 253){
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Invalid control mode %lu or id %u for joint %s", mode, (uint8_t)id, joint_info.name.c_str());
            return CallbackReturn::ERROR;
        }
    }

    // Fancy union typization
    switch(protocol_){
        case ProtocolType::SMS_STS: driver_.emplace<SMS_STS>(); break;
        case ProtocolType::HLSCL: driver_.emplace<HLSCL>(); break;
        case ProtocolType::SCS0009: driver_.emplace<SCS0009>(); break;
        case ProtocolType::SCSCL: driver_.emplace<SCSCL>(); break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Invalid protocol type %s", protocol_type_.c_str());
            return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/){
    
    // fancy variant use one method on all posible types. Works in this case but be carefull with adding more driver protocols.
    bool success = std::visit([&](auto& driver) -> bool{
            return driver.begin(baudrate_, port_.c_str(), tx_enable_pin_, rx_enable_pin_);
        },
        driver_
    );

    if(!success){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to initialize port %s", port_.c_str());
        return CallbackReturn::ERROR;
    }
    
    // Ping all servo motors and log results
    bool all_servos_online = std::visit([&](auto& driver) -> bool{
        bool online = true;
        for(auto& joint : joint_list_){
            if(driver.Ping(joint.id) == -1){
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Servo with %u ID not available", joint.id);
                online = false;
            }else{
                RCLCPP_INFO(rclcpp::get_logger(LOGGER), "Servo with %u ID online", joint.id);
            }
        }
        return online;
    }, driver_);

    if(!all_servos_online){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Could not see some servo motors with PING!");
        return CallbackReturn::ERROR;
    }

    bool motor_initialized = true;
    if(std::holds_alternative<SMS_STS>(driver_)){
        SMS_STS& driver = std::get<SMS_STS>(driver_);
        
        for(auto& joint : joint_list_){
            if(driver.InitMotor(joint.id, joint.mode, 0) != 0){
                RCLCPP_INFO(rclcpp::get_logger(LOGGER), "Successfully initialized motor with %u ID.", joint.id);
            }else{
                motor_initialized = false;
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to initialize motor with %u ID.", joint.id);
            }
        }
    }else if(std::holds_alternative<HLSCL>(driver_)){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Motor initialization not implemented for selected protocol.");
        return CallbackReturn::ERROR;
    }else if(std::holds_alternative<SCS0009>(driver_)){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Motor initialization not implemented for selected protocol.");
        return CallbackReturn::ERROR;
    }else if(std::holds_alternative<SCSCL>(driver_)){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Motor initialization not implemented for selected protocol.");
        return CallbackReturn::ERROR;
    }

    if(!motor_initialized){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Could not initialize some servo motors!");
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/){
    std::visit([&](auto& driver){
            driver.end();
        },
        driver_
    );

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/){
    bool motor_torque = true;
    if(std::holds_alternative<SMS_STS>(driver_)){
        SMS_STS& driver = std::get<SMS_STS>(driver_);
        for(auto& joint : joint_list_){
            if(driver.EnableTorque(joint.id, 1) != 1){
                motor_torque = false;
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to enable torque motor with %u ID.", joint.id);
            }
        }
    }

    if(!motor_torque) 
        return CallbackReturn::ERROR;
    

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/){
    bool motor_torque = true;
    if(std::holds_alternative<SMS_STS>(driver_)){
        SMS_STS& driver = std::get<SMS_STS>(driver_);
        for(auto& joint : joint_list_){
            if(driver.EnableTorque(joint.id, 0) != 1){
                motor_torque = false;
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to disable torque motor with %u ID.", joint.id);
            }
        }
    }

    if(!motor_torque) 
        return CallbackReturn::ERROR;

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