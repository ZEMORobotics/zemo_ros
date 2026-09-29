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
        conf_names.push_back(joint.name + "/voltage");
        conf_names.push_back(joint.name + "/temperature");
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

        auto_declare<double>("max_voltage", 20.0);
        auto_declare<double>("min_voltage", 3.0);
        auto_declare<double>("max_temperature", 100.0);
        auto_declare<double>("max_current", 3000.0);
        auto_declare<double>("default_timeout", 5.0);
        auto_declare<double>("default_tolerance", 0.1);
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
    max_voltage_ = get_node()->get_parameter("max_voltage").as_double();
    min_voltage_ = get_node()->get_parameter("min_voltage").as_double();
    max_temperature_ = get_node()->get_parameter("max_temperature").as_double();
    max_current_ = get_node()->get_parameter("max_current").as_double();
    default_timeout_ = get_node()->get_parameter("default_timeout").as_double();
    default_tolerance_ = get_node()->get_parameter("default_tolerance").as_double();

    if(position_joints_.empty() && velocity_joints_.empty()){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "No servo joints configured! Don't waste CPU, turn of the controller!");
        return CallbackReturn::FAILURE;
    }

    joints_.clear();
    joints_.reserve(position_joints_.size() + velocity_joints_.size());

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


controller_interface::CallbackReturn ServoController::on_cleanup(const rclcpp_lifecycle::State &/*previous_state*/){
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ServoController::on_activate(const rclcpp_lifecycle::State &/*previous_state*/){
    return CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn ServoController::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/){
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type ServoController::update(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/){
    return controller_interface::return_type::OK;
}


}   // namespace zemo_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(zemo_controllers::ServoController, controller_interface::ControllerInterface)