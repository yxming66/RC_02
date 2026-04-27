#pragma once

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

extern volatile float g_arm_j1_test_torque_nm;
extern volatile float g_arm_j2_test_torque_nm;
extern volatile float g_arm_j3_test_torque_nm;

#ifdef __cplusplus
}

namespace mrobot::arm {

bool Init(float control_freq_hz);
void Update();
void PollCommand(osMessageQueueId_t cmd_queue);
void Control(float dt_s);

}  // namespace mrobot::arm
#endif
