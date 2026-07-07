#pragma once

/* Runtime debug/profiler switches and update periods.
 * Adjust these defaults here when you need more watch-window detail during tuning.
 */

#ifndef TASK_RUNTIME_PROFILER_ENABLE
#define TASK_RUNTIME_PROFILER_ENABLE (0)
#endif

#ifndef MOTOR_RM_TX_DEBUG_ENABLE
#define MOTOR_RM_TX_DEBUG_ENABLE (0)
#endif

#ifndef MOTOR_PROTOCOL_DEBUG_ENABLE
#define MOTOR_PROTOCOL_DEBUG_ENABLE (0)
#endif

#ifndef MRLINK_PC_DEBUG_FULL_UPDATE_PERIOD_MS
#define MRLINK_PC_DEBUG_FULL_UPDATE_PERIOD_MS (50u)
#endif

#ifndef RC_DEBUG_UPDATE_PERIOD_MS
#define RC_DEBUG_UPDATE_PERIOD_MS (50u)
#endif

#ifndef AUTO_ORE_DEBUG_UPDATE_PERIOD_MS
#define AUTO_ORE_DEBUG_UPDATE_PERIOD_MS (50u)
#endif

#ifndef ORE_STORE_DEBUG_UPDATE_PERIOD_MS
#define ORE_STORE_DEBUG_UPDATE_PERIOD_MS (50u)
#endif

#ifndef SICK_DEBUG_UPDATE_PERIOD_MS
#define SICK_DEBUG_UPDATE_PERIOD_MS (50u)
#endif

#ifndef POLE_PID_DEBUG_UPDATE_PERIOD_MS
#define POLE_PID_DEBUG_UPDATE_PERIOD_MS (50u)
#endif

#ifndef CHASSIS_RUNTIME_DEBUG_ENABLE
#define CHASSIS_RUNTIME_DEBUG_ENABLE (0)
#endif