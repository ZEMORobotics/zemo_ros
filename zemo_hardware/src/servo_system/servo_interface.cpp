#include "zemo_hardware/servo_system/servo_interface.hpp"


namespace zemo_hardware{

hardware_interface::CallbackReturn ServoInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params){
    if(SystemInterface::on_init(params) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;

    const auto& hw_params = params.hardware_info.hardware_parameters;

    port_ = hw_params.at("port");
    baudrate_ = std::stoul(hw_params.at("baudrate"));
    std::string protocol_type = hw_params.at("protocol");
    tx_enable_pin_ = (uint8_t)std::stoul(hw_params.at("tx_en_pin"));
    rx_enable_pin_ = (uint8_t)std::stoul(hw_params.at("rx_en_pin"));

    // This part of the code finds the protocol from the allowed list.

    for(auto& x : protocol_type) x = tolower(x);   // c++ couldn't find a better way to make all characters lower case
    auto it = allowed_protocols.find(protocol_type);

    if(it == allowed_protocols.end()){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Invalid protocol type %s. Check servo_interface.hpp for valid options.", protocol_type.c_str());
        return CallbackReturn::ERROR;
    }
    protocol_ = it->second;

    size_t number_of_joints = params.hardware_info.joints.size();
    joint_list_.resize(number_of_joints);

    // Parse hardware info
    for(size_t i = 0; i < number_of_joints; i++){
        const auto& joint_info = params.hardware_info.joints[i];

        auto mode = std::stoul(joint_info.parameters.at("mode"));
        auto id = std::stoul(joint_info.parameters.at("id"));

        joint_list_[i].name = joint_info.name;
        joint_list_[i].id = (uint8_t)id;
        joint_list_[i].mode = static_cast<ControlMode>(mode);

        // In most protocols 0xfe (254) is broadcast id, so we allow 253 and below.
        if(mode > 1 || id > 253){
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Invalid control mode %lu or id %u for joint %s.\n Mode: 0 or 1, ID: <= 253", mode, (uint8_t)id, joint_info.name.c_str());
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
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Protocol is in allowed list, but it is not unionized %s", protocol_type.c_str());
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
        
        for(Joint& joint : joint_list_){
            int32_t local_count = retry_count_;

            // PING the servo motor, with retry_count_ tries.
            while(local_count > 0){
                if(driver.Ping(joint.id) == joint.id){
                    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "Servo with %s online", joint.name.c_str());
                    break;
                }
                local_count--;
            }

            if(local_count <= 0){
                online = false;
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Servo %s not available", joint.name.c_str());        
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
        
        for(Joint& joint : joint_list_){
            if(driver.InitMotor(joint.id, joint.mode, 0) != 0){
                RCLCPP_INFO(rclcpp::get_logger(LOGGER), "Successfully initialized motor %s.", joint.name.c_str());
            }else{
                motor_initialized = false;
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to initialize motor %s.", joint.name.c_str());
            }
        }
    }else if(std::holds_alternative<HLSCL>(driver_)){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Motor initialization not implemented for HLSCL protocol.");
        return CallbackReturn::ERROR;
    }else if(std::holds_alternative<SCS0009>(driver_)){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Motor initialization not implemented for SCS0009 protocol.");
        return CallbackReturn::ERROR;
    }else if(std::holds_alternative<SCSCL>(driver_)){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Motor initialization not implemented for SCSCL protocol.");
        return CallbackReturn::ERROR;
    }else{
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
    rclcpp::Logger status_logger = rclcpp::get_logger("zemo_hardware.servo_info");
    rcutils_logging_set_logger_level(status_logger.get_name(), RCUTILS_LOG_SEVERITY_INFO);

    if(std::holds_alternative<SMS_STS>(driver_)){
        SMS_STS& driver = std::get<SMS_STS>(driver_);
        for(auto& joint : joint_list_){
            if(driver.EnableTorque(joint.id, 1) != 1){
                motor_torque = false;
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to enable torque on motor %s.", joint.name.c_str());
            }

            if(driver.FeedBack(joint.id) == 1){
                double voltage = driver.ReadVoltage(-1) / 10.0;
                int temperature = driver.ReadTemper(-1);
                int current = driver.ReadCurrent(-1);

                RCLCPP_INFO(status_logger, "Servo %s online. VOLTAGE: %.1lf V, TEMPERATURE: %d C, CURRENT: %d mA", joint.name.c_str(), voltage, temperature, current);
            }else{
                motor_torque = false;
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to read feedback from motor %s.", joint.name.c_str());
            }
        }
    }else{
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to activate servo system: Protocol not implemented!");
        return CallbackReturn::ERROR;
    }

    if(!motor_torque) 
        return CallbackReturn::ERROR;

    for(auto& gpio : gpio_commands_){
        if(gpio->get_interface_name() == "write_type"){
            write_type_cmd_ = gpio;
        }else if(gpio->get_interface_name() == "sequence"){
            sequence_cmd_ = gpio;
        }
    }

    if(!write_type_cmd_ || !sequence_cmd_){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to activate servo system: missing write_type or sequence interface!");
        return CallbackReturn::FAILURE;
    }

    for(command_interface_t& command : joint_commands_){
        const std::string& name = command->get_prefix_name();

        for(Joint& joint : joint_list_){
            if(name == joint.name){
                if(command->get_interface_name() == "position"){
                    joint.position_cmd = command;
                }else if(command->get_interface_name() == "velocity"){
                    joint.velocity_cmd = command;
                }else if(command->get_interface_name() == "acceleration"){
                    joint.acceleration_cmd = command;
                }else if(command->get_interface_name() == "joint_sequence"){
                    joint.joint_sequence_cmd = command;
                }
            }
        }
    }
    for(state_interface_t& state : joint_states_){
        const std::string& name = state->get_prefix_name();

        for(Joint& joint : joint_list_){
            if(name == joint.name){
                if(state->get_interface_name() == "position"){
                    joint.position_state = state;
                }else if(state->get_interface_name() == "velocity"){
                    joint.velocity_state = state;
                }else if(state->get_interface_name() == "moving"){
                    joint.moving_state = state;
                }else if(state->get_interface_name() == "current"){
                    joint.current_state = state;
                }
            }
        }
    }

    for(Joint& joint : joint_list_){
        if(!joint.velocity_cmd || !joint.acceleration_cmd || !joint.joint_sequence_cmd || (joint.mode == ControlMode::POSITION && !joint.position_cmd)){
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to activate servo system: missing command interfaces for motor %s", joint.name.c_str());
            return CallbackReturn::FAILURE;
        }
        if(!joint.position_state || !joint.velocity_state || !joint.moving_state || !joint.current_state){
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to activate servo system: missing state interfaces for motor %s", joint.name.c_str());
            return CallbackReturn::FAILURE;
        }
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/){
    write_type_cmd_.reset();
    sequence_cmd_.reset();

    for(Joint& joint : joint_list_){
        joint.position_cmd.reset();
        joint.velocity_cmd.reset();
        joint.acceleration_cmd.reset();
        joint.joint_sequence_cmd.reset();

        joint.position_state.reset();
        joint.velocity_state.reset();
        joint.moving_state.reset();
        joint.current_state.reset();
    }

    bool motor_torque = true;

    if(std::holds_alternative<SMS_STS>(driver_)){
        SMS_STS& driver = std::get<SMS_STS>(driver_);
        for(auto& joint : joint_list_){
            if(driver.EnableTorque(joint.id, 0) != 1){
                motor_torque = false;
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to disable torque on motor %s.", joint.name.c_str());
            }
        }
    }

    if(!motor_torque) 
        return CallbackReturn::ERROR;

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ServoInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/){
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ServoInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/){
    return hardware_interface::return_type::OK;
}

} // namespace zemo_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(zemo_hardware::ServoInterface, hardware_interface::SystemInterface)