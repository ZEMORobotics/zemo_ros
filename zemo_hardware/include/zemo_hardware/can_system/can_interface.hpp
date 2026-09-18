#pragma once

#include "zemo_hardware/can_system/can_protocol.hpp"
#include "zemo_hardware/can_system/diff_drive.hpp"
#include "zemo_hardware/can_system/ir_sensor.hpp"
#include "zemo_hardware/can_system/cinch.hpp"

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/hardware_component_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <thread>
#include <atomic>

// Can communication libraries
extern "C" {
    #include <unistd.h>
    #include <sys/socket.h>
    #include <sys/ioctl.h>
    #include <net/if.h>
    #include <linux/can.h>
    #include <linux/can/raw.h>
    #include <string.h>
}

namespace zemo_hardware{

class CanInterface : public hardware_interface::SystemInterface{

public:
    CanInterface() = default;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
private:
    // Can setup
    std::string can_interface_name_{"can0"};
    int can_socket_{-1};
    sockaddr_can can_address_;
    ifreq can_ifr_;

    std::thread can_rx_thread_;
    std::atomic<bool> can_rx_running_{false};

    // Devices
    DiffDrive diff_drive_system_;
    IRSensor ir_sensor_system_;
    Cinch cinch_system_;

    void can_rx_loop();
    bool send_frame(const can_frame& frame);
    void close_can_socket();
};

}   // namespace zemo_hardware