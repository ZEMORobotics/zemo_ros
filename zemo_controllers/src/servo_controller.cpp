#include "zemo_controllers/servo_controller.hpp"


namespace zemo_controllers{


controller_interface::InterfaceConfiguration ServoController::command_interface_configuration() const{
    std::vector<std::string> conf_names;

    for(auto &joint : joints_){
        if(joint.mode == ControlMode::POSITION){
            conf_names.push_back(joint.name + "/position");
        }

        conf_names.push_back(joint.name + "/velocity");
        conf_names.push_back(joint.name + "/acceleration");

        conf_names.push_back(joint.name + "/write_type");
        conf_names.push_back(joint.name + "/sequence");
    }

    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration ServoController::state_interface_configuration() const{
    std::vector<std::string> conf_names;

    for(auto &joint : joints_){
        conf_names.push_back(joint.name + "/position");
        conf_names.push_back(joint.name + "/velocity");
        conf_names.push_back(joint.name + "/load");
        conf_names.push_back(joint.name + "/moving");
        conf_names.push_back(joint.name + "/current");
    }

    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn ServoController::on_init(){

    try{
        // declare .yaml parameters
        auto_declare<std::vector<std::string>>("position_joints", std::vector<std::string>{});
        auto_declare<std::vector<std::string>>("velocity_joints", std::vector<std::string>{});

        auto_declare<double>("max_current", 3000.0);
        auto_declare<double>("tolerance", 0.1);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Exception thrown during init stage with message: %s", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ServoController::on_configure(const rclcpp_lifecycle::State &/*previous_state*/){
    // Parse .yaml parameteres

    position_joints_ = get_node()->get_parameter("position_joints").as_string_array();
    velocity_joints_ = get_node()->get_parameter("velocity_joints").as_string_array();
    max_current_ = get_node()->get_parameter("max_current").as_double();
    tolerance_ = get_node()->get_parameter("tolerance").as_double();

    if(position_joints_.empty() && velocity_joints_.empty()){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "No servo joints configured! Don't waste CPU, turn off the controller!");
        return CallbackReturn::FAILURE;
    }

    joints_.clear();
    size_t num_of_servos = position_joints_.size() + velocity_joints_.size();

    joints_.reserve(num_of_servos);
        
    active_commands_.reserve(num_of_servos);

    for(auto &name : position_joints_){
        joints_.emplace_back();
        joints_.back().name = name;
        joints_.back().mode = ControlMode::POSITION;
    }

    for(auto &name : velocity_joints_){
        if(std::find(position_joints_.begin(), position_joints_.end(), name) != position_joints_.end()){
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER),"Joint '%s' is in both position_joints and velocity_joints", name.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }
        joints_.emplace_back();
        joints_.back().name = name;
        joints_.back().mode = ControlMode::VELOCITY;
    }

    return CallbackReturn::SUCCESS;
}   

controller_interface::CallbackReturn ServoController::on_activate(const rclcpp_lifecycle::State &/*previous_state*/){

    for(auto& joint : joints_){

        auto bind_command = [&](auto& handle, const char *type) -> bool{
            auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&](auto& interface){
                return interface.get_prefix_name() == joint.name && interface.get_interface_name() == type;
            });

            if(it == command_interfaces_.end()){
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Mising command interface: %s%s", joint.name.c_str(), type);
                return false;
            }

            handle = std::ref(*it);
            return true;
        };

        auto bind_state = [&](auto& handle, const char *type) -> bool{
            auto it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](auto& interface){
                return interface.get_prefix_name() == joint.name && interface.get_interface_name() == type;
            });

            if(it == state_interfaces_.end()){
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Mising state interface: %s%s", joint.name.c_str(), type);
                return false;
            }

            handle = std::ref(*it);
            return true;
        };


        // Position command interface is only avaliable for position mode(mode=0)
        if(joint.mode == ControlMode::POSITION){
            if(!bind_command(joint.position_command_handle, "position")){
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to bind position command interface!");
                return CallbackReturn::ERROR;
            }
        }
        

        if(!bind_command(joint.velocity_command_handle, "velocity") || !bind_command(joint.acceleration_command_handle, "acceleration") ||
           !bind_command(joint.write_type_command_handle, "write_type") || !bind_command(joint.sequence_command_handle, "sequence")){
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to bind on or more: velocity, acceleration, write_type, sequence command interfaces!");
            return CallbackReturn::ERROR;
        }

        if(!bind_state(joint.position_handle, "position") || !bind_state(joint.speed_handle, "velocity") ||
           !bind_state(joint.load_handle, "load") || !bind_state(joint.status_handle, "moving") || !bind_state(joint.current_handle, "current")){
            
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to bind on or more state interfaces!");
            return CallbackReturn::ERROR;
        }
    }

    return CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn ServoController::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/){
    for(auto& joint : joints_){
        joint.position_command_handle.reset();
        joint.velocity_command_handle.reset();
        joint.acceleration_command_handle.reset();
        joint.write_type_command_handle.reset();
        joint.sequence_command_handle.reset();

        joint.position_handle.reset();
        joint.speed_handle.reset();
        joint.load_handle.reset();
        joint.current_handle.reset();
        joint.status_handle.reset();
    }

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type ServoController::update(const rclcpp::Time &time, const rclcpp::Duration &/*period*/){
    check_active_servos(time);
    send_command(time);
    return controller_interface::return_type::OK;
}

void ServoController::send_command(const rclcpp::Time &time){

    // Lock mutex to see pending commands from action server.
    std::unique_lock<std::mutex> ul(command_mutex_, std::try_to_lock);
    // If mutex is already taken or there is no commands skip this func.
    if(!pending_commands_.empty() && ul.owns_lock()) return;

    // This moves ownership of command instead of copying it. Rust memory shenanigans.
    ActiveCommand command = std::move(pending_commands_.front());
    pending_commands_.pop_front();

    ul.unlock();

    const uint64_t sequence = next_sequence_++;

    command.active = true;
    command.start_time_ns = time.nanoseconds();

    for(auto& motor : command.targets){
        // find the servo by name in vector. It stays as vector instead of unordered map beacuse it is a trivial look up,
        // there is a max number of 10 servos per command. On average it would be 1-3 servos. Hashing uses some time in lookup,
        // so in this real time example, vector is better, note that joints_ vector is preallocated in on_configure().
        auto it = std::find_if(joints_.begin(), joints_.end(), [&](const Joint& m){
            return motor.name == m.name;
        });

        if(it == joints_.end()){
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Servo joint: %s, does not exist. Check your control.yaml and control.urdf", motor.name);
            continue;
        }

        Joint& joint = *it; 

        // Default value for interfaces is double.
        if(joint.mode == ControlMode::POSITION)
            joint.position_command_handle->get().set_value(motor.position);

        joint.velocity_command_handle->get().set_value(motor.velocity);
        joint.acceleration_command_handle->get().set_value(motor.acceleration);
        joint.write_type_command_handle->get().set_value(command.sync_write ? 1.0 : 0.0);
        joint.sequence_command_handle->get().set_value((double)sequence);
    }

    active_commands_.push_back(std::move(command));
}

void ServoController::check_active_servos(const rclcpp::Time &time){
    for(auto cmd_it = active_commands_.begin(); cmd_it != active_commands_.end();){
        auto &command = *cmd_it;
        bool all_finished = true;

        int64_t elapsed_ns = time.nanoseconds() - command.start_time_ns;

        if(elapsed_ns > command.timeout_ns){
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Servo command timeout!");
            cmd_it = active_commands_.erase(cmd_it);
            continue;
        }

        for(const auto &motor : command.targets){
            auto it = std::find_if(joints_.begin(), joints_.end(), [&](const Joint &joint){
                return joint.name == motor.name;
            });

            if(it == joints_.end()){
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Servo joint %s does not exist!", motor.name.c_str());
                command.active = false;
                break;
            }

            auto &joint = *it;

            const double current = joint.current_handle->get().get_value();

            if(std::abs(current) > max_current_){
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Servo %s, current error! Reading: %lf mA", joint.name.c_str(), current);
                command.active = false;
                break;
            }

            if(joint.mode == ControlMode::POSITION){
                const double position = joint.position_handle->get().get_value();
                const double moving = joint.status_handle->get().get_value();

                if(moving != 0.0 || std::abs(position - motor.position) > tolerance_){
                    all_finished = false;
                }

            }else if(joint.mode == ControlMode::VELOCITY){
                const double velocity = joint.speed_handle->get().get_value();

                if(std::abs(velocity - motor.velocity) > tolerance_){
                    all_finished = false;
                }
            }
        }
        

        if(command.active && all_finished){
            command.active = false;

            // action goal -> succeed()
        }

        if(!command.active) 
            cmd_it = active_commands_.erase(cmd_it);
        else 
            cmd_it++; 
    }
}


// Not used but still needs to be overwriten.
controller_interface::CallbackReturn ServoController::on_cleanup(const rclcpp_lifecycle::State &/*previous_state*/){
    return CallbackReturn::SUCCESS;
}


}   // namespace zemo_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(zemo_controllers::ServoController, controller_interface::ControllerInterface)