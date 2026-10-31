#pragma once

#include "zemo_hardware/servo_system/waveshare_driver/SMS_STS.h"
#include "zemo_hardware/servo_system/waveshare_driver/HLSCL.h"
#include "zemo_hardware/servo_system/waveshare_driver/SCS0009.h"
#include "zemo_hardware/servo_system/waveshare_driver/SCSCL.h"

#include <variant> // fancy c++ union
#include <algorithm>
#include <unordered_map>
#include <cmath>
#include <string>
#include <optional>
#include <variant>
#include <cstdint>
#include <vector>

namespace zemo_hardware{

enum class ProtocolType{SMS_STS, HLSCL, SCS0009, SCSCL};


// This allows for typos when writing protocol type in urdf
static std::unordered_map<std::string, ProtocolType> allowed_protocols{
    {"sms_sts", ProtocolType::SMS_STS}, 
    {"sts_sms", ProtocolType::SMS_STS},
    {"alibabaisedampatuljaka", ProtocolType::SMS_STS}, 
    {"sms", ProtocolType::SMS_STS},
    {"sts", ProtocolType::SMS_STS}, 
    {"st3215", ProtocolType::SMS_STS}
};

// Fancy c++ union. This means that before runtime we don't know the protocol, therefore don't know
// the type being used by the servo driver. So we apply a pattern with union that will allow us
// to assign the type of driver_ in runtime.
// Strongly recommend to read docs on std::variant.
using ServoDriver = std::variant<SMS_STS, HLSCL, SCS0009, SCSCL>;
enum ControlMode : uint8_t {POSITION=0, VELOCITY=1};


inline std::optional<ProtocolType> parse_protocol(std::string name){
    // This part of the code finds the protocol from the allowed list.

    for(char& c : name) c = tolower(c);   // c++ couldn't find a better way to make all characters lower case
    auto it = allowed_protocols.find(name);

    if(it == allowed_protocols.end()){
        return std::nullopt;
    }

    return it->second;
}

inline bool make_driver(ServoDriver& driver, ProtocolType protocol){
    // Fancy union typization
    switch(protocol){
        case ProtocolType::SMS_STS: driver.emplace<SMS_STS>(); return true;
        case ProtocolType::HLSCL: driver.emplace<HLSCL>(); return true;
        case ProtocolType::SCS0009: driver.emplace<SCS0009>(); return true;
        case ProtocolType::SCSCL: driver.emplace<SCSCL>(); return true;
    }

    return false;
}

struct Command{
    int16_t position{0};
    int16_t velocity{0};
    uint16_t max_speed{0};
    uint8_t acceleration{0};
};

struct Feedback{
    double position{0.0};
    double velocity{0.0};
    double moving{0.0};
    double current{0.0};
};

struct Status{
    double voltage{0.0};
    int temperature{0};
    int current{0};
};


// Default functions for not implemented drivers
// TODO: check if you actually need these, as driver check is performed in hardware interface
template<typename Driver>
inline bool init_motor(Driver&, uint8_t, ControlMode){
    return false;
}

template<typename Driver>
inline bool torque(Driver&, uint8_t, bool){
    return false;
}

template<typename Driver>
inline bool encode(Driver&, ControlMode, double, double, double, Command&){
    return false;
}

template<typename Driver>
inline bool driver_send_single(Driver&, uint8_t, ControlMode, const Command&){
    return false;
}

template<typename Driver>
inline bool send_parallel(Driver&, ControlMode, std::vector<uint8_t>&, std::vector<int16_t>&, std::vector<uint16_t>&, std::vector<int16_t>&, std::vector<uint8_t>&){
    return false;
}

template<typename Driver>
inline bool read_feedback(Driver&, uint8_t, Feedback&){
    return false;
}

template<typename Driver>
inline bool read_status(Driver&, uint8_t, Status&){
    return false;
}

namespace sms_sts{
    inline constexpr double STEPS_PER_RAD = 4096.0 / (2.0 * M_PI);
    inline constexpr double ACC_UNIT = 100.0;

    [[gnu::always_inline]] inline int16_t position(double rad){
        return static_cast<int16_t>(std::clamp(std::lround(rad * STEPS_PER_RAD), -4095L, 4095L));
    }
    [[gnu::always_inline]] inline int16_t velocity(double rad_s){
        return static_cast<int16_t>(std::clamp(std::lround(rad_s * STEPS_PER_RAD), -3400L, 3400L));
    }
    [[gnu::always_inline]] inline uint8_t acceleration(double rad_s2){
        return static_cast<uint8_t>(std::clamp(std::lround(std::abs(rad_s2) * STEPS_PER_RAD / ACC_UNIT), 0L, 254L));
    }

    [[gnu::always_inline]] inline double to_rad(int steps){
        return static_cast<double>(steps) / STEPS_PER_RAD;
    }
}   // namespace sms_sts

inline bool init_motor(SMS_STS& driver, uint8_t& id, ControlMode mode){
    return driver.InitMotor(id, mode, 0) != 0;
}

inline bool torque(SMS_STS& driver, uint8_t& id, uint8_t enable){
    return driver.EnableTorque(id, enable);
}

inline bool encode(SMS_STS&, ControlMode mode, double position, double velocity, double acceleration, Command& command){
    command.velocity = sms_sts::velocity(velocity);
    command.acceleration = sms_sts::acceleration(acceleration);

    if(mode == ControlMode::POSITION){
        command.position = sms_sts::position(position);
        command.max_speed = static_cast<uint16_t>(std::abs(command.velocity));
    }
    return true;
}

inline bool driver_send_single(SMS_STS& driver, uint8_t& id, ControlMode mode, const Command& command){
    if(mode == ControlMode::POSITION){
        return driver.WritePosEx(id, command.position, command.max_speed, command.acceleration);
    }
    return driver.WriteSpe(id, command.velocity, command.acceleration);
}

inline bool send_parallel(SMS_STS& driver, ControlMode mode,
    std::vector<uint8_t>& ids,
    std::vector<int16_t>& positions,
    std::vector<uint16_t>& max_speeds,
    std::vector<int16_t>& velocities,
    std::vector<uint8_t>& accelerations)
{

    if(mode == ControlMode::POSITION){
        driver.SyncWritePosEx(ids.data(), static_cast<uint8_t>(ids.size()), positions.data(), max_speeds.data(), accelerations.data());
    }else{
        driver.SyncWriteSpe(ids.data(), static_cast<uint8_t>(ids.size()), velocities.data(), accelerations.data());
    }

    return true;
}

inline bool read_feedback(SMS_STS& driver, uint8_t& id, Feedback& feedback){
    if(driver.FeedBack(id) != 1)
        return false;

    feedback.position = sms_sts::to_rad(driver.ReadPos(-1));
    feedback.velocity = sms_sts::to_rad(driver.ReadSpeed(-1));
    feedback.moving = static_cast<double>(driver.ReadMove(-1));
    feedback.current = static_cast<double>(driver.ReadCurrent(-1));

    return true;
}

inline bool read_status(SMS_STS& driver, uint8_t& id, Status& status){
    if(driver.FeedBack(id) != 1)
        return false;

    status.voltage = driver.ReadVoltage(-1) / 10.0;
    status.temperature = driver.ReadTemper(-1);
    status.current = driver.ReadCurrent(-1);

    return true;
}

}   // namespace zemo_hardware