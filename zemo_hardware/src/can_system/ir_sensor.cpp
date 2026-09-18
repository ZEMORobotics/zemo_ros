#include "zemo_hardware/can_system/ir_sensor.hpp"

namespace zemo_hardware{

void IRSensor::can_read_front(uint8_t *buff){
    std::lock_guard<std::mutex> lock(read_mutex_);
    front_state_buffer_ = buff[0];
}

void IRSensor::can_read_back(uint8_t *buff){
    std::lock_guard<std::mutex> lock(read_mutex_);
    back_state_buffer_ = buff[0];
}

void IRSensor::control_read(){
    {
        std::lock_guard<std::mutex> lock(read_mutex_);
        back_state_ = back_state_buffer_;
        front_state_ = front_state_buffer_;
    }

    if(front_state_ != prev_front_state_){
        // publisher new state to IR sensor topic
        prev_front_state_ = front_state_;
    }

    if(back_state_ != prev_back_state_){
        // publisher new state to IR sensor topic
        prev_back_state_ = back_state_;
    }
}

} // namespace zemo_hardware