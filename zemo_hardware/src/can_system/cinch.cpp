#include "zemo_hardware/can_system/cinch.hpp"

namespace zemo_hardware{

bool Cinch::init(const rclcpp::Node::SharedPtr& node){
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    publisher_ = node->create_publisher<cinch_msg>(CINCH_TOPIC, qos);

    return publisher_ != nullptr;
}

void Cinch::can_read_state(uint8_t *buff){
    std::lock_guard<std::mutex> lock(read_mutex_);
    cinch_state_buffer_ = buff[0];
}

void Cinch::control_read(){
    {
        std::lock_guard<std::mutex> lock(read_mutex_);
        cinch_state_ = cinch_state_buffer_;
    }

    // When cinch state is 1. START
    if(cinch_state_ != prev_cinch_state_){
        cinch_msg msg;
        msg.data = cinch_state_;
        publisher_->publish(msg);

        prev_cinch_state_ = cinch_state_;
    }
}

} // namespace zemo_hardware