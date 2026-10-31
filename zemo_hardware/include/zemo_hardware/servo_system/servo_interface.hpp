#pragma once

#include "zemo_hardware/servo_system/servo_protocols.hpp"

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/hardware_component_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <string>
#include <vector>
#include <optional>

namespace zemo_hardware{

using command_interface_t = hardware_interface::CommandInterface::SharedPtr;
using state_interface_t = hardware_interface::StateInterface::SharedPtr;

struct Joint{
    Joint(){};

    command_interface_t position_cmd;
    command_interface_t velocity_cmd;
    command_interface_t acceleration_cmd;
    command_interface_t joint_sequence_cmd;

    state_interface_t position_state;
    state_interface_t velocity_state;
    state_interface_t moving_state;
    state_interface_t current_state;

    uint8_t id{1};
    ControlMode mode{ControlMode::POSITION};
    std::string name;
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
    static constexpr const char* LOGGER = "zemo_hardware_servo";
    static constexpr const int_fast8_t retry_count_ = 10;

    ServoDriver driver_;

    std::string port_{"/dev/ttyAMA0"};
    std::optional<ProtocolType> protocol_{ProtocolType::SMS_STS};
    uint8_t tx_enable_pin_{23};
    uint8_t rx_enable_pin_{24};
    uint32_t baudrate_{115200};

    // This variable is used for async write to the motors.
    // It is very important that it is initialized at 0;
    uint64_t current_sequence_{0};

    std::vector<Joint> joint_list_;
    std::vector<Joint*> active_joints_;
    
    // For sync write
    std::vector<uint8_t> ids_;
    std::vector<int16_t> positions_;
    std::vector<uint16_t> max_speeds_;
    std::vector<int16_t> velocities_;
    std::vector<uint8_t> accelerations_;

    command_interface_t write_type_cmd_;
    command_interface_t sequence_cmd_;

// Functions
private:
    bool send_single(Joint& joint);
    bool parse_parallel_commands(Joint& joint);
    inline void clear_command_buffers();
};

} // namespace zemo_hardware