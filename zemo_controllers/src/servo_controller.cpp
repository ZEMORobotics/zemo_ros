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
        conf_names.push_back(joint.name + "/joint_sequence");
    }

    conf_names.push_back(port_ + "/write_type");
    conf_names.push_back(port_ + "/sequence");

    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration ServoController::state_interface_configuration() const{
    std::vector<std::string> conf_names;

    for(auto &joint : joints_){
        conf_names.push_back(joint.name + "/position");
        conf_names.push_back(joint.name + "/velocity");
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
        auto_declare<double>("velocity_tolerance", 3.0);
        auto_declare<std::string>("usb_port", "ttyAMA0");
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
    port_ = get_node()->get_parameter("usb_port").as_string();
    max_current_ = get_node()->get_parameter("max_current").as_double();
    tolerance_ = get_node()->get_parameter("tolerance").as_double();
    velocity_tolerance_ = get_node()->get_parameter("velocity_tolerance").as_double();

    if(position_joints_.empty() && velocity_joints_.empty()){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "No servo joints configured! Don't waste CPU, turn off the controller!");
        return CallbackReturn::FAILURE;
    }

    if(port_.empty()){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Didn't specify the usb port for servo motors!");
        return CallbackReturn::FAILURE;
    }

    joints_.clear();
    size_t num_of_servos = position_joints_.size() + velocity_joints_.size();

    joints_.reserve(num_of_servos);
    
    current_pending_command_.reset();
    current_active_command_.reset();

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
    if(!bind_command(write_type_command_handle_, port_, "write_type") || !bind_command(sequence_command_handle_, port_, "sequence")) 
        return CallbackReturn::ERROR;

    if(!sequence_command_handle_->get().set_value((double)0.0)){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to activate the controller. Can't set sequence to 0");
        return CallbackReturn::ERROR;
    }

    for(auto& joint : joints_){
        // Position command interface is only avaliable for position mode(mode=0)
        if(joint.mode == ControlMode::POSITION){
            if(!bind_command(joint.position_command_handle, joint.name, "position")) 
                return CallbackReturn::ERROR;
        }

        if(!bind_command(joint.velocity_command_handle, joint.name, "velocity") || !bind_command(joint.acceleration_command_handle, joint.name, "acceleration") ||
            !bind_command(joint.joint_sequence_command_handle, joint.name, "joint_sequence")){
                return CallbackReturn::ERROR;
            }

        if(!bind_state(joint.position_handle, joint.name, "position") || !bind_state(joint.speed_handle, joint.name, "velocity") ||
            !bind_state(joint.status_handle, joint.name, "moving") || !bind_state(joint.current_handle, joint.name, "current")){
            return CallbackReturn::ERROR;
        }
    }

    // Create action server
    action_server_ = rclcpp_action::create_server<ServoAction>(
        get_node(), "servo_action_server", 
        std::bind(&ServoController::handle_goal, this, std::placeholders::_1, std::placeholders::_2), 
        std::bind(&ServoController::handle_cancel, this, std::placeholders::_1),
        std::bind(&ServoController::handle_accepted, this, std::placeholders::_1)
    );

    return CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn ServoController::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/){
    write_type_command_handle_.reset();
    sequence_command_handle_.reset();

    for(auto& joint : joints_){
        joint.position_command_handle.reset();
        joint.velocity_command_handle.reset();
        joint.acceleration_command_handle.reset();
        joint.joint_sequence_command_handle.reset();

        joint.position_handle.reset();
        joint.speed_handle.reset();
        joint.current_handle.reset();
        joint.status_handle.reset();
    }

    auto abort_command = [](ActiveCommand& command){
        if(command.goal_handle && command.goal_handle->is_active()){
            auto result = std::make_shared<ServoAction::Result>();
            result->result = ServoAction::Goal::ERROR;
            command.goal_handle->abort(result);
        }
    };

    if(current_active_command_){
        abort_command(*current_active_command_);
        current_active_command_.reset();
    }

    std::optional<ActiveCommand> pending;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending = std::move(current_pending_command_);
        current_pending_command_.reset();
    }

    action_server_.reset();
    command_busy_.store(false);

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type ServoController::update(const rclcpp::Time &time, const rclcpp::Duration &/*period*/){
    check_active_servos(time);
    send_command(time);
    return controller_interface::return_type::OK;
}

void ServoController::send_command(const rclcpp::Time &time){
    // If there is active command ongoing, skip this.
    if(current_active_command_) return;

    // Lock mutex to see pending commands from action server.
    std::unique_lock<std::mutex> ul(command_mutex_, std::try_to_lock);

    // If mutex is already taken or there is no commands skip this func.
    if(!ul.owns_lock() || !current_pending_command_) return;

    ActiveCommand command = std::move(*current_pending_command_);
    current_pending_command_.reset();
    ul.unlock();

    const uint64_t sequence = next_sequence_++;

    command.start_time_ns = time.nanoseconds();
    bool success = true;    // This bool does absolutely nothing, it is here to satisfy ros2 warnings.

    for(ServoTarget& motor : command.targets){
        // find the servo by name in vector. It stays as vector instead of unordered map beacuse it is a trivial look up,
        // there is a practical max number of 10 servos per command. On average it would be 1-3 servos. Hashing uses some time in lookup,
        // so in this real time example, vector is better, note that joints_ vector is preallocated in on_configure().
        Joint &joint = joints_[motor.index]; 

        // Default value for interfaces is double.
        if(joint.mode == ControlMode::POSITION)
            success &= joint.position_command_handle->get().set_value(motor.position);

        success &= joint.velocity_command_handle->get().set_value(motor.velocity);
        success &= joint.acceleration_command_handle->get().set_value(motor.acceleration);
        success &= joint.joint_sequence_command_handle->get().set_value((double)sequence);
    }

    if(success)
        success = write_type_command_handle_->get().set_value(command.sync_write ? 1.0 : 0.0);
    
    if(success)
        success = sequence_command_handle_->get().set_value((double)sequence);

    if(!success){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to set command interfaces.");

        auto result = std::make_shared<ServoAction::Result>();
        result->result = ServoAction::Goal::ERROR;
        command.goal_handle->abort(result);
        command_busy_.store(false);

        return;
    }

    current_active_command_ = std::move(command);
}

void ServoController::check_active_servos(const rclcpp::Time &time){
    // If there is no active command currently, skip this.
    if(!current_active_command_) return;

    ActiveCommand &command = *current_active_command_;
    
    uint64_t elapsed_ns = time.nanoseconds() - command.start_time_ns;

    if(elapsed_ns > command.timeout_ns){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Servo command timeout!");

        auto result = std::make_shared<ServoAction::Result>();
        result->result = ServoAction::Goal::TIMEOUT;
        command.goal_handle->abort(result);

        current_active_command_.reset();
        command_busy_.store(false);
        return;
    }

    bool all_finished = true;
    for(const ServoTarget &motor : command.targets){
        Joint &joint = joints_[motor.index];

        // This is also done to satisfy ros2 colcon build.
        std::optional<double> position_opt = joint.position_handle->get().get_optional();
        std::optional<double> velocity_opt = joint.speed_handle->get().get_optional();
        std::optional<double> moving_opt = joint.status_handle->get().get_optional();
        std::optional<double> current_opt = joint.current_handle->get().get_optional();

        if(!velocity_opt || !position_opt || !moving_opt || !current_opt){
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Failed to parse read from %s servo!", motor.name.c_str());
            all_finished = false;
            break;
        }

        double velocity = *velocity_opt;
        double position = *position_opt;
        double moving = *moving_opt;
        double current = *current_opt;

        if(std::abs(current) > max_current_){
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Servo %s, current error! Reading: %lf mA", joint.name.c_str(), current);

            auto result = std::make_shared<ServoAction::Result>();
            result->result = ServoAction::Goal::CURRENT;
            command.goal_handle->abort(result);

            current_active_command_.reset();
            command_busy_.store(false);
            return;
        }

        if(joint.mode == ControlMode::POSITION){
            if(moving != 0.0 || std::abs(position - motor.position) > tolerance_){
                all_finished = false;
            }

        }else if(joint.mode == ControlMode::VELOCITY){
            if(std::abs(velocity - motor.velocity) > velocity_tolerance_){
                all_finished = false;
            }
        }
    }
    

    if(all_finished){
        auto result = std::make_shared<ServoAction::Result>();
        result->result = ServoAction::Goal::SUCCESS;
        command.goal_handle->succeed(result);

        current_active_command_.reset();
        command_busy_.store(false);
    } 
}


rclcpp_action::GoalResponse ServoController::handle_goal(const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const ServoAction::Goal> goal){
    size_t n = goal->names.size();

    if(n == 0 || goal->positions.size() != n || goal->velocities.size() != n || goal->accelerations.size() != n){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Goal rejected. Positions, velocities and accelerations should be the same size.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if(n > joints_.size()){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Goal rejected. Recieved more motors than the number of motors in parsed from .yaml.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if(!goal->sync_write && n != 1){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Goal rejected. Sync write should be true if there is more than one servo.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    for(const auto& name : goal->names){
        auto it = std::find_if(joints_.begin(), joints_.end(), [&](Joint& joint){
            return joint.name == name;
        });

        if(it == joints_.end()){
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Goal rejected. Can't find servo with id: %s.", name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    for(size_t i = 0; i < goal->names.size(); i++){
        for(size_t j = i + 1; j < goal->names.size(); j++){
            if(goal->names[i] == goal->names[j]){
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Goal rejected. Same motor name not allowed twice: %s", goal->names[i].c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
    }

    if(goal->timeout_ms < 1){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Goal rejected. Timeout should be > 1, got value: %d", goal->timeout_ms);
        return rclcpp_action::GoalResponse::REJECT;
    }

    bool expected = false;
    if(!command_busy_.compare_exchange_strong(expected, true)){
        RCLCPP_WARN(rclcpp::get_logger(LOGGER), "Goal rejected. A servo command is already in progress.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ServoController::handle_cancel(const std::shared_ptr<GoalHandle> /*goal_handle*/){
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Received request to cancel, it is denied.");
    return rclcpp_action::CancelResponse::REJECT;
}

void ServoController::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle){
    const auto goal = goal_handle->get_goal();

    ActiveCommand command;
    
    command.goal_handle = goal_handle;
    command.sync_write = goal->sync_write;
    command.timeout_ns = static_cast<uint64_t>(goal->timeout_ms) * 1000000ULL;

    command.targets.reserve(goal->names.size());

    for(size_t i = 0; i < goal->names.size(); i++){
        ServoTarget s;

        for(size_t j = 0; j < joints_.size(); j++){
            if(joints_[j].name == goal->names[i]){
                s.index = j;
            }
        }

        s.name = goal->names[i];
        s.position = goal->positions[i];
        s.velocity = goal->velocities[i];
        s.acceleration = goal->accelerations[i];
        command.targets.push_back(s);
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    current_pending_command_ = std::move(command);
}

}   // namespace zemo_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(zemo_controllers::ServoController, controller_interface::ControllerInterface)