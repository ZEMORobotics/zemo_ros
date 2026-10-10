#pragma once

#include "zemo_hardware/servo_system/waveshare_driver/SMS_STS.h"

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/hardware_component_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <string>
#include <vector>
#include <variant> // fancy c++ union
#include <unordered_map>

namespace zemo_hardware{

using command_interface_t = hardware_interface::CommandInterface::SharedPtr;
using state_interface_t = hardware_interface::StateInterface::SharedPtr;

enum class ProtocolType{
    SMS_STS, HLSCL, SCS0009, SCSCL
};

// This allows for typos when writing protocol type in urdf
static std::unordered_map<std::string, ProtocolType> allowed_protocols{
    {"sms_sts", ProtocolType::SMS_STS}, {"sts_sms", ProtocolType::SMS_STS},
    {"alibabaisedampatuljaka", ProtocolType::SMS_STS}, {"sms", ProtocolType::SMS_STS},
    {"sts", ProtocolType::SMS_STS}, {"st3215", ProtocolType::SMS_STS}
};

// Fancy c++ union. This means that before runtime we don't know the protocol, therefore don't know
// the type being used by the servo driver. So we apply a pattern with union that will allow us
// to assign the type of driver_ in runtime.
// Strongly recommend to read docs on std::variant.
using ServoDriver = std::variant<SMS_STS, HLSCL, SCS0009, SCSCL>;
enum ControlMode : uint8_t {POSITION=0, VELOCITY=1};


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
    static constexpr const int32_t retry_count_ = 10;

    ServoDriver driver_;

    std::string port_{"/dev/ttyAMA0"};
    ProtocolType protocol_{ProtocolType::SMS_STS};
    uint8_t tx_enable_pin_{23};
    uint8_t rx_enable_pin_{24};
    uint32_t baudrate_{115200};

    std::vector<Joint> joint_list_;

    command_interface_t write_type_cmd_;
    command_interface_t sequence_cmd_;
};

} // namespace zemo_hardware