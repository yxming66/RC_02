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
  /* 电平触发锁存（用于上台阶） */
  bool pe13_photo1_triggered_latched; /* PE13/photo1 latched as triggered. */
  bool pe9_photo2_triggered_latched;  /* PE9/photo2 latched as triggered. */
  bool pa2_photo3_triggered_latched;  /* PA2/photo3 latched as triggered. */
  bool pa0_photo4_triggered_latched;  /* PA0/photo4 latched as triggered. */
  uint32_t pe13_photo1_triggered_since_ms;
  uint32_t pe9_photo2_triggered_since_ms;
  uint32_t pa2_photo3_triggered_since_ms;
  uint32_t pa0_photo4_triggered_since_ms;
  /* 边沿检测（用于下台阶：触发→不触发） */
  bool pe13_photo1_prev_triggered; /* PE13/photo1 previous state. */
  bool pe9_photo2_prev_triggered;  /* PE9/photo2 previous state. */
  bool pa2_photo3_prev_triggered;  /* PA2/photo3 previous state. */
  bool pa0_photo4_prev_triggered;  /* PA0/photo4 previous state. */
  bool pe13_photo1_falling_edge_latched; /* PE13/photo1 falling edge. */
  bool pe9_photo2_falling_edge_latched;  /* PE9/photo2 falling edge. */
  bool pa2_photo3_falling_edge_latched;  /* PA2/photo3 falling edge. */
  bool pa0_photo4_falling_edge_latched;  /* PA0/photo4 falling edge. */
  bool pole_target_seen_not_ready; /* Current step has observed pole target not ready. */
} auto_ctrl_template_ctx_t;

#ifdef __cplusplus
}
#endif
