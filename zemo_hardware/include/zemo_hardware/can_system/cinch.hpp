#pragma once

#include <cstdint>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace zemo_hardware{

#define CINCH_TOPIC "cinch/state"
using cinch_msg = std_msgs::msg::UInt8;

class Cinch{

public:
    Cinch() = default;

    // Disallow copy
    Cinch(const Cinch&) = delete;
    Cinch& operator=(const Cinch&) = delete;

    bool init(const rclcpp::Node::SharedPtr& node);

    std::mutex read_mutex_;
    void can_read_state(uint8_t *buff);
    void control_read();

private:
    uint8_t cinch_state_buffer_{0};
    uint8_t prev_cinch_state_{0};
    uint8_t cinch_state_{0};

    rclcpp::Publisher<cinch_msg>::SharedPtr publisher_;
};

} // namespace zemo_hardware