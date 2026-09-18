#pragma once

#include "zemo_hardware/can_system/can_protocol.hpp"

#include <cstdint>
#include <string>
#include <mutex>
#include <array>
// Ros2 gives warnings if this is not used.
#include <optional>

extern "C" {
    #include <linux/can.h>
}

#include <hardware_interface/hardware_component_interface.hpp>

#include <rclcpp/rclcpp.hpp>

namespace zemo_hardware{
    
    static constexpr uint8_t LEFT_WHEEL = 0;
    static constexpr uint8_t RIGHT_WHEEL = 1;

    struct DiffDriveConfig{
        float kp, ki, kd;
        float kp_angular, ki_angular, kd_angular;

        int ticks_per_rev;

        std::string left_wheel;
        std::string right_wheel;
    };

    class DiffDrive{
        public:
            DiffDrive() = default;

            // Disallow copy
            DiffDrive(const DiffDrive&) = delete;
            DiffDrive& operator=(const DiffDrive&) = delete;

            bool init(const hardware_interface::HardwareInfo& params);
            bool activate(const std::vector<hardware_interface::StateInterface::SharedPtr>& states, const std::vector<hardware_interface::CommandInterface::SharedPtr>& commands);
            bool deactivate();
            
            void can_read_vel(uint8_t *buff);
            void can_read_pos(uint8_t *buff);
            // Ros control update, calculates all variables used by diff drive control.
            // Seperated from can read and write so that it does not block can read.
            bool control_read();
            std::optional<can_frame> control_write();

            std::array<std::pair<canid_t, float>, 6> get_pid_commands() const;
            std::vector<can_frame> generate_activate_msgs();
            std::vector<can_frame> generate_deactivate_msgs();

        private:
            // config
            float kp_, ki_, kd_;
            float kp_angular_, ki_angular_, kd_angular_;
            int ticks_per_rev_;
            std::string left_wheel, right_wheel;

            // Communication
            std::mutex read_mutex_;

            uint8_t velocity_buffer_[8]{};
            uint8_t position_buffer_[8]{};

            // ros2 control
            hardware_interface::CommandInterface::SharedPtr left_wheel_velocity_cmd_;
            hardware_interface::CommandInterface::SharedPtr right_wheel_velocity_cmd_;

            hardware_interface::StateInterface::SharedPtr left_wheel_velocity_state_;
            hardware_interface::StateInterface::SharedPtr left_wheel_position_state_;
            hardware_interface::StateInterface::SharedPtr right_wheel_velocity_state_;
            hardware_interface::StateInterface::SharedPtr right_wheel_position_state_;

            // Odometry
            int32_t previous_encoder_ticks_[2]{0, 0};
            int32_t current_encoder_ticks_[2]{0, 0};
            float tick_to_rad_{0.0};
            bool encoder_initialized_{false};

            float wheel_position_[2]{0.0, 0.0};
            float wheel_velocity_[2]{0.0, 0.0};
            int encoder_sign_[2]{1, 1};

            int64_t encoder_overflow_handle(int32_t current, int32_t previous);
            DiffDriveConfig parse_diff_drive_params(const hardware_interface::HardwareInfo& info);
    };

} // namespace zemo_hardware