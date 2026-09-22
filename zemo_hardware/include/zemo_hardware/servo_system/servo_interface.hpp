#pragma once

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/hardware_component_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>


namespace zemo_hardware{

class ServoInterface : public hardware_interface::SystemInterface{

public:
    ServoInterface() = default;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
private:
    // gas

};

} // namespace zemo_hardware