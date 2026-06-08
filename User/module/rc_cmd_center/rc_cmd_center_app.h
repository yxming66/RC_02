#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void RcCmdCenterApp_Init(void);
void RcCmdCenterApp_Update(uint32_t now_ms);

#ifdef __cplusplus
}
#endif
