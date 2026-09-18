#include "zemo_hardware/can_system/diff_drive.hpp"

namespace zemo_hardware{

// Lifecycle inits
bool DiffDrive::init(const hardware_interface::HardwareInfo& params){
    DiffDriveConfig config = parse_diff_drive_params(params);

    kp_ = config.kp; ki_ = config.ki; kd_ = config.kd;
    kp_angular_ = config.kp_angular; ki_angular_ = config.ki_angular; kd_angular_ = config.kd_angular;
    ticks_per_rev_ = config.ticks_per_rev;
    left_wheel = config.left_wheel;
    right_wheel = config.right_wheel;

    if(kp_ < 0.0 || ki_ < 0.0 || kd_ < 0.0 || kp_angular_ < 0.0 || kd_angular_ < 0.0 || ki_angular_ < 0.0){
        RCLCPP_FATAL(rclcpp::get_logger("zemo_hardware"), "Failed to initialize diff drive. PID parameters must be non negative value!\n");
        return false;
    }
    if(left_wheel.empty() || right_wheel.empty()){
        RCLCPP_FATAL(rclcpp::get_logger("zemo_hardware"), "Wrong or non existing wheel joint names in urdf!\n");
        return false;
    }
    if (ticks_per_rev_ <= 0) {
        RCLCPP_FATAL(rclcpp::get_logger("zemo_hardware"), "ticks_per_rev_ param must be above 0!\n");
        return false;
    }
    
    constexpr float TWO_PI = 2*M_PI;
    tick_to_rad_ = TWO_PI / (float)ticks_per_rev_;

    RCLCPP_INFO(rclcpp::get_logger("zemo_hardware"), "Diff Drive initialized!\n");
    return true;
}

bool DiffDrive::activate(const std::vector<hardware_interface::StateInterface::SharedPtr>& states, const std::vector<hardware_interface::CommandInterface::SharedPtr>& commands){
    for(const auto& state : states){
        const auto& name = state->get_name();

        if(name == left_wheel + "/position") left_wheel_position_state_ = state;
        else if(name == left_wheel + "/velocity") left_wheel_velocity_state_ = state;
        else if(name == right_wheel + "/position") right_wheel_position_state_ = state;
        else if(name == right_wheel + "/velocity") right_wheel_velocity_state_ = state;
    }

    for(const auto& command : commands){
        const auto& name = command->get_name();

        if(name == left_wheel + "/velocity") left_wheel_velocity_cmd_ = command;
        else if(name == right_wheel + "/velocity") right_wheel_velocity_cmd_ = command;
    }

    if(!left_wheel_position_state_ || !left_wheel_velocity_state_ ||
        !right_wheel_position_state_ || !right_wheel_velocity_state_ ||
        !left_wheel_velocity_cmd_ || !right_wheel_velocity_cmd_)
    {
        RCLCPP_FATAL(rclcpp::get_logger("zemo_hardware"), "Failed to activate DiffDrive: missing ros2_control interfaces!");
        return false;
    }

    bool success = true;

    success &= left_wheel_position_state_->set_value<float>(0.0, true);
    success &= left_wheel_velocity_state_->set_value<float>(0.0, true);
    success &= right_wheel_position_state_->set_value<float>(0.0, true);
    success &= right_wheel_velocity_state_->set_value<float>(0.0, true);

    encoder_initialized_ = false;

    wheel_position_[LEFT_WHEEL] = 0.0;
    wheel_position_[RIGHT_WHEEL] = 0.0;
    wheel_velocity_[LEFT_WHEEL] = 0.0;
    wheel_velocity_[RIGHT_WHEEL] = 0.0;

    return success;
}

bool DiffDrive::deactivate(){
    return true;
}

std::array<std::pair<canid_t, float>, 6> DiffDrive::get_pid_commands() const{
    return {{
        {can_protocol::PID_KP,     kp_},
        {can_protocol::PID_KI,     ki_},
        {can_protocol::PID_KD,     kd_},
        {can_protocol::PID_KP_ROT, kp_angular_},
        {can_protocol::PID_KI_ROT, ki_angular_},
        {can_protocol::PID_KD_ROT, kd_angular_}
    }};
}

std::vector<can_frame> DiffDrive::generate_activate_msgs(){
    std::vector<can_frame> frames;

    for (const auto& [id, value] : get_pid_commands()) {
        can_frame frame{};
        frame.can_id = id;
        frame.can_dlc = 4;

        can_protocol::pack_float(frame.data, value);
        frames.push_back(frame);
    }

    auto add_command = [&frames](canid_t id){
        can_frame frame{};
        frame.can_id = id;
        frame.can_dlc = 4;

        can_protocol::pack_int32(frame.data, 1);
        frames.push_back(frame);
    };

    add_command(can_protocol::RESET_ENCODERS);
    add_command(can_protocol::MOTORS_ON);
    add_command(can_protocol::FEEDBACK_ON);
    add_command(can_protocol::REFERENCE_ON);

    return frames;
}

std::vector<can_frame> DiffDrive::generate_deactivate_msgs(){
    std::vector<can_frame> frames;
    
    auto add_command = [&frames](canid_t id){
        can_frame frame{};
        frame.can_id = id;
        frame.can_dlc = 4;

        can_protocol::pack_int32(frame.data, 1);
        frames.push_back(frame);
    };

    add_command(can_protocol::REFERENCE_OFF);
    add_command(can_protocol::FEEDBACK_OFF);
    add_command(can_protocol::MOTORS_OFF);
    add_command(can_protocol::RESET_ENCODERS);

    return frames;
}


// Communication
void DiffDrive::can_read_vel(uint8_t *buff){
    std::lock_guard<std::mutex> lock(read_mutex_);
    memcpy(velocity_buffer_, buff, 8);
}

void DiffDrive::can_read_pos(uint8_t *buff){
    std::lock_guard<std::mutex> lock(read_mutex_);
    memcpy(position_buffer_, buff, 8);
}

std::optional<can_frame> DiffDrive::control_write(){
    can_frame msg{};

    auto left_velocity = left_wheel_velocity_cmd_->get_optional<float>();
    auto right_velocity = right_wheel_velocity_cmd_->get_optional<float>();

    if(!left_velocity || !right_velocity) return std::nullopt;

    can_protocol::pack_float(msg.data, *left_velocity);
    can_protocol::pack_float(msg.data + 4, *right_velocity);

    msg.can_id = can_protocol::WHEEL_COMMAND_VEL;
    msg.can_dlc = 8;

    return msg;
}

bool DiffDrive::control_read(){

    {
        std::lock_guard<std::mutex> lock(read_mutex_);
        for(int i = 0; i < 2; i++){
            current_encoder_ticks_[i] = can_protocol::unpack_encoder(position_buffer_, i);
            wheel_velocity_[i] = can_protocol::unpack_velocity(velocity_buffer_, i) * encoder_sign_[i];
        }
    }

    bool success = true;

    if(!encoder_initialized_){
        for(int i = 0; i < 2; i++){
            previous_encoder_ticks_[i] = current_encoder_ticks_[i];
        }
        encoder_initialized_ = true;

        
        success &= left_wheel_position_state_->set_value<float>(wheel_position_[LEFT_WHEEL], false);
        success &= left_wheel_velocity_state_->set_value<float>(wheel_velocity_[LEFT_WHEEL], false);
        success &= right_wheel_position_state_->set_value<float>(wheel_position_[RIGHT_WHEEL], false);
        success &= right_wheel_velocity_state_->set_value<float>(wheel_velocity_[RIGHT_WHEEL], false);

        return success;
    }

    for(int i = 0; i < 2; i++){
        
        int64_t delta_ticks = encoder_overflow_handle(current_encoder_ticks_[i], previous_encoder_ticks_[i]);

        delta_ticks *= encoder_sign_[i];
        float delta_position = static_cast<float>(delta_ticks) * tick_to_rad_;

        wheel_position_[i] += delta_position;
        previous_encoder_ticks_[i] = current_encoder_ticks_[i];
    }

    success &= left_wheel_position_state_->set_value<float>(wheel_position_[LEFT_WHEEL], false);
    success &= left_wheel_velocity_state_->set_value<float>(wheel_velocity_[LEFT_WHEEL], false);
    success &= right_wheel_position_state_->set_value<float>(wheel_position_[RIGHT_WHEEL], false);
    success &= right_wheel_velocity_state_->set_value<float>(wheel_velocity_[RIGHT_WHEEL], false);

    return success;
}

int64_t DiffDrive::encoder_overflow_handle(int32_t current, int32_t previous){
    int64_t delta = static_cast<int64_t>(current) - static_cast<int64_t>(previous);

    constexpr int64_t range = (int64_t{1} << 32);

    if(delta > INT32_MAX) delta -= range;
    if(delta < INT32_MIN) delta += range;

    return delta;
}

DiffDriveConfig DiffDrive::parse_diff_drive_params(const hardware_interface::HardwareInfo& info){
    DiffDriveConfig config;

    auto& map = info.hardware_parameters;

    config.kp = stof(map.at("diff_drive.kp"));
    config.ki = stof(map.at("diff_drive.ki"));
    config.kd = stof(map.at("diff_drive.kd"));
    config.kp_angular = stof(map.at("diff_drive.kp_angular"));
    config.ki_angular = stof(map.at("diff_drive.ki_angular"));
    config.kd_angular = stof(map.at("diff_drive.kd_angular"));

    config.ticks_per_rev = stoi(map.at("diff_drive.enc_tick_per_rev"));

    for (const auto& joint : info.joints){
        if (joint.name == "wheel_left_joint") config.left_wheel = joint.name;

        else if (joint.name == "wheel_right_joint") config.right_wheel = joint.name;
    }

    return config;
}

} // namespace zemo_hardware