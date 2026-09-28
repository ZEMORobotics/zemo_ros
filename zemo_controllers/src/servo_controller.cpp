//#include "zemo_controllers/servo_controller.hpp"

#include "zemo_controllers/servo_controller.hpp"

namespace zemo_controllers{

controller_interface::InterfaceConfiguration ServoController::command_interface_configuration() const{

}
controller_interface::InterfaceConfiguration ServoController::state_interface_configuration() const{

}
controller_interface::CallbackReturn ServoController::on_init(){
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ServoController::on_configure(const rclcpp_lifecycle::State &/*previous_state*/){
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