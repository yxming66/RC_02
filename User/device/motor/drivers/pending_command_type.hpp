#pragma once

#include <stdint.h>

namespace mrobot::motor {

enum class PendingCommandType : uint8_t {
    None = 0,
    Torque,
    Velocity,
    Position,
    Mit,
};

} // namespace mrobot::motor