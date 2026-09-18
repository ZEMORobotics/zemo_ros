#pragma once

#include <mutex>

namespace zemo_hardware{

class IRSensor{

public:
    IRSensor() = default;

    // Disallow copy
    IRSensor(const IRSensor&) = delete;
    IRSensor& operator=(const IRSensor&) = delete;

    void can_read_front(uint8_t *buff);
    void can_read_back(uint8_t *buff);
    void control_read();
    
private:
    std::mutex read_mutex_;
    uint8_t front_state_buffer_{0};
    uint8_t back_state_buffer_{0};

    uint8_t front_state_{0};
    uint8_t back_state_{0};
    uint8_t prev_front_state_{0};
    uint8_t prev_back_state_{0};

};

} // namespace zemo_hardware