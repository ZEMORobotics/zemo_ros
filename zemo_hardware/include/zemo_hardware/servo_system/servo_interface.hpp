#pragma once

#include "zemo_hardware/servo_system/waveshare_driver/SMS_STS.h"

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/hardware_component_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <string>
#include <vector>

namespace zemo_hardware{

enum ControlMode : uint8_t {POSITION=0, VELOCITY=1};

struct JointValue{
    double position{0.0};
    double velocity{0.0};
    double acceleration{0.0};
};

struct Joint{
    Joint(){};

    uint8_t id{1};
    ControlMode mode{ControlMode::POSITION};
    
    JointValue state{};
    JointValue command{};
};

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
    SMS_STS driver_{};

    std::string port_{"/dev/ttyAMA0"};
    std::string protocol_type_{"sms_sts"};
    uint8_t tx_enable_pin_{15};
    uint8_t rx_enable_pin_{15};
    uint32_t baudrate_{115200};

    std::vector<Joint> joint_list_;
};

} // namespace zemo_hardware