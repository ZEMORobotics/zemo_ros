#pragma once

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "zemo_msgs/action/servo_command.hpp"

#include <vector>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>

namespace zemo_controllers{

using ServoAction = zemo_msgs::action::ServoCommand;
using GoalHandle = rclcpp_action::ServerGoalHandle<ServoAction>;

using command_interface_t = std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>;
using state_interface_t = std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;

enum class ControlMode : uint8_t{
    POSITION=0,
    VELOCITY=1
};

struct ServoTarget{
    std::string name;

    double position;
    double velocity;
    double acceleration;
};

struct ActiveCommand{
    bool active{false};

    uint64_t start_time_ns;
    uint64_t timeout_ns{2000000000ULL}; // ns

    bool sync_write{false};
    std::vector<ServoTarget> targets;   // Servos that are executing action goal
    std::shared_ptr<GoalHandle> goal_handle;
};


struct Joint{
    // hardware write interfaces
    command_interface_t position_command_handle;
    command_interface_t velocity_command_handle;
    command_interface_t acceleration_command_handle;
    command_interface_t joint_sequence_command_handle;

    // hardware read interfaces
    state_interface_t position_handle;
    state_interface_t speed_handle;
    state_interface_t status_handle;
    state_interface_t current_handle;

    std::string name{"djura"};
    ControlMode mode{ControlMode::POSITION};
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

private:
    static constexpr const char* LOGGER = "zemo_controllers_servo";
    // .yaml parameters
    double max_current_;
    double tolerance_;
    double velocity_tolerance_;
    std::string port_;

    std::vector<std::string> position_joints_;
    std::vector<std::string> velocity_joints_;

    // Parsed data from .yaml
    std::vector<Joint> joints_;

    // Action server variables
    std::deque<ActiveCommand> pending_commands_;
    std::vector<ActiveCommand> active_commands_;

    std::mutex command_mutex_;
    uint64_t next_sequence_{1};

    // Action server shenanigans
    rclcpp_action::Server<ServoAction>::SharedPtr action_server_;
    // rclcpp::TimerBase::SharedPtr action_timer_;

    command_interface_t write_type_command_handle_;
    command_interface_t sequence_command_handle_; 

// Functions
private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ServoAction::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void process_action_results();

    void send_command(const rclcpp::Time &time);
    void check_active_servos(const rclcpp::Time &time);


    bool bind_command(command_interface_t& handle, const std::string& name, const std::string& type){
        auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&](auto& interface){
            return interface.get_prefix_name() == name && interface.get_interface_name() == type;
        });

        if(it == command_interfaces_.end()){
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Missing command interface: %s/%s", name.c_str(), type.c_str());
            return false;
        }

        handle = std::ref(*it);
        return true;
    };

    bool bind_state(state_interface_t& handle, const std::string& name, const std::string& type){
        auto it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](auto& interface){
            return interface.get_prefix_name() == name && interface.get_interface_name() == type;
        });

        if(it == state_interfaces_.end()){
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Missing state interface: %s/%s", name.c_str(), type.c_str());
            return false;
        }

        handle = std::ref(*it);
        return true;
    };

};

}   // namespace zemo_controllers