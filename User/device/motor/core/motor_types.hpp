//



#pragma once

#include <stdint.h>

namespace mrobot::motor {

enum class Vendor : uint8_t {
    RM = 0,
    DM = 1,
    LZ = 2,
};

enum class Model : uint16_t {
    Unknown = 0,

    RM_2006,
    RM_3508,
    RM_6020,

    DM_J4310,

    LZ_RSO0,
    LZ_RSO1,
    LZ_RSO2,
    LZ_RSO3,
    LZ_RSO4,
    LZ_RSO5,
    LZ_RSO6,
};

} // namespace mrobot::motor
