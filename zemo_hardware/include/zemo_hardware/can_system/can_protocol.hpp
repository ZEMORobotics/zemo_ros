#pragma once

#include <array>
#include <cstdint>
#include <cstring>

extern "C" {
    #include <linux/can.h>
}

namespace can_protocol{

static constexpr canid_t CAN_MASK = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);

enum CanID{
    WHEEL_COMMAND_VEL = 100,
    WHEEL_FEEDBACK_VEL = 101,
    WHEEL_POSITION = 102,
    PID_KP = 103, 
    PID_KI = 104, 
    PID_KD = 105,
    PID_KP_ROT = 106, 
    PID_KI_ROT = 107, 
    PID_KD_ROT = 108,
    MOTORS_ON = 109, 
    MOTORS_OFF = 110,
    FEEDBACK_ON = 111, 
    FEEDBACK_OFF = 112,
    REFERENCE_ON = 113, 
    REFERENCE_OFF = 114,
    RESET_ENCODERS = 115, 
    RESET_PID = 116,
    FRONT_IR = 150, 
    BACK_IR = 151,
    CINCH = 152
};

static constexpr std::array RX_IDS{
    WHEEL_FEEDBACK_VEL,
    WHEEL_POSITION,
    FRONT_IR, 
    BACK_IR,
    CINCH
};

inline int32_t unpack_encoder(uint8_t *buff, int wheel){
    int offset = wheel * 4;

    uint32_t value =
        (static_cast<uint32_t>(buff[offset])     << 24) |
        (static_cast<uint32_t>(buff[offset + 1]) << 16) |
        (static_cast<uint32_t>(buff[offset + 2]) << 8)  |
         static_cast<uint32_t>(buff[offset + 3]);

    return static_cast<int32_t>(value);
}

inline float unpack_velocity(uint8_t *buff, int wheel){
    int offset = wheel * 4;

    uint32_t value =
        (static_cast<uint32_t>(buff[offset])     << 24) |
        (static_cast<uint32_t>(buff[offset + 1]) << 16) |
        (static_cast<uint32_t>(buff[offset + 2]) << 8)  |
         static_cast<uint32_t>(buff[offset + 3]);

    float result;
    std::memcpy(&result, &value, sizeof(float));
    return result;
}

inline void pack_u32(uint8_t* buff, uint32_t value){
    buff[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
    buff[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
    buff[2] = static_cast<uint8_t>((value >> 8)  & 0xFF);
    buff[3] = static_cast<uint8_t>( value        & 0xFF);
}

inline void pack_int32(uint8_t* buff, int32_t value){
    pack_u32(buff, static_cast<uint32_t>(value));
}

inline void pack_float(uint8_t* buff, float value){
    uint32_t raw;
    std::memcpy(&raw, &value, sizeof(float));

    pack_u32(buff, raw);
}

}   // namespace can_protocol