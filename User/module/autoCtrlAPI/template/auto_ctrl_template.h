#pragma once

/**
 * @file auto_ctrl_template.h
 * @brief AutoCtrl 模板执行上下文定义。
 *
 * 该文件仅定义模板运行时状态，不包含执行逻辑。
 * 模板调度器会在每个周期读写该上下文，用于实现分步执行。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint8_t step_index;            /* 当前模板执行到的 step 编号。 */
  uint32_t step_enter_time_ms;   /* 当前 step 首次进入时刻（ms）。 */
  uint32_t template_start_time_ms; /* 当前模板开始时刻（ms）。 */
  bool step_entered;             /* 当前 step 是否已经执行过 enter 初始化。 */
} auto_ctrl_template_ctx_t;

#ifdef __cplusplus
}
#endif