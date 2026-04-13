#pragma once

#include <stdint.h>

namespace mrobot::motor {

enum class MotorCapability : uint32_t {
    None = 0,
    Current = 1u << 0,
    Velocity = 1u << 1,
    Position = 1u << 2,
    MIT = 1u << 3,
};

inline constexpr MotorCapability operator|(MotorCapability lhs, MotorCapability rhs) {
    return static_cast<MotorCapability>(static_cast<uint32_t>(lhs) | static_cast<uint32_t>(rhs));
}

inline constexpr MotorCapability operator&(MotorCapability lhs, MotorCapability rhs) {
    return static_cast<MotorCapability>(static_cast<uint32_t>(lhs) & static_cast<uint32_t>(rhs));
}

inline constexpr bool HasCapability(MotorCapability caps, MotorCapability cap) {
    return static_cast<uint32_t>(caps & cap) != 0u;
}

} // namespace mrobot::motor
