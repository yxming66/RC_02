#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	RC_CMD_CENTER_RF_BEHAVIOR_NONE = 0u,
	RC_CMD_CENTER_RF_BEHAVIOR_RELAX = 1u,
	RC_CMD_CENTER_RF_BEHAVIOR_LOCK = 2u,
} RcCmdCenterRfBehavior_t;

void RcCmdCenterApp_Init(void);
void RcCmdCenterApp_Update(uint32_t now_ms);
void RcCmdCenterApp_SetRfBehavior(RcCmdCenterRfBehavior_t behavior);
void RcCmdCenterApp_RequestReset(void);

#ifdef __cplusplus
}
#endif
