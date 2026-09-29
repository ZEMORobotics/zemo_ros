#pragma once

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <vector>
#include <string>

namespace zemo_controllers{

enum class ControlMode : uint8_t{
    POSITION=0,
    VELOCITY=1
};

struct ServoTarget{
    uint8_t joint_id;

    double position;
    double velocity;
    double acceleration;
    double tolerance;
};

struct ActiveCommand{
    bool active{false};

    uint64_t start_time_ns;
    double timeout{3000}; // ms

    bool sync_write{false};
    std::vector<ServoTarget> targets;   // Servos that are executing action goal
};

struct Joint{
    Joint(){};

    std::string name{"djura"};
    ControlMode mode{ControlMode::POSITION};

    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> position_command_handle;
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> velocity_command_handle;
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> acceleration_command_handle;
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> write_type_command_handle;
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> sequence_command_handle; 

    std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> position_handle;
    std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> speed_handle;
    std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> load_handle;
    std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> voltage_handle;
    std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> temperature_handle;
    std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> status_handle;
    std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> current_handle;
};

class ServoController : public controller_interface::ControllerInterface{

public:
    ServoController() = default;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_init() override;
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

private:
    static constexpr const char* LOGGER = "zemo_controllers_servo";


    std::vector<std::string> position_joints_;
    std::vector<std::string> velocity_joints_;
    std::vector<Joint> joints_;

    double min_voltage_{0.0};
    double max_voltage_{20.0};
    double max_temperature_;
    double max_current_;
    double default_timeout_;
    double default_tolerance_;

};

}   // namespace zemo_controllers