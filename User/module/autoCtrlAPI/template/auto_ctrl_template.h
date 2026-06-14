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
  uint32_t photo_stop_enter_time_ms; /* descend photo stop hold start, ms */
  uint32_t descend_start_move_time_ms; /* first descend sprint start, ms */
  bool photo_stop_entered;       /* descend photo stop hold has started */
  bool descend_start_move_entered; /* first descend sprint timer has started */
  uint8_t step_index;            /* 当前模板执行到的 step 编号。 */
  uint32_t step_enter_time_ms;   /* 当前 step 首次进入时刻（ms）。 */
  uint32_t template_start_time_ms; /* 当前模板开始时刻（ms）。 */
  bool step_entered;             /* 当前 step 是否已经执行过 enter 初始化。 */
  /* Stable photo trigger/release detection state. */
  bool pe13_photo1_triggered_latched; /* PE13/photo1 latched as triggered. */
  bool pe9_photo2_triggered_latched;  /* PE9/photo2 latched as triggered. */
  bool pa2_photo3_triggered_latched;  /* PA2/photo3 latched as triggered. */
  bool pa0_photo4_triggered_latched;  /* PA0/photo4 latched as triggered. */
  uint32_t pe13_photo1_triggered_since_ms;
  uint32_t pe9_photo2_triggered_since_ms;
  uint32_t pa2_photo3_triggered_since_ms;
  uint32_t pa0_photo4_triggered_since_ms;
  bool pe13_photo1_stable_trigger_seen;
  bool pe9_photo2_stable_trigger_seen;
  bool pa2_photo3_stable_trigger_seen;
  bool pa0_photo4_stable_trigger_seen;
  bool pe13_photo1_stable_release_latched;
  bool pe9_photo2_stable_release_latched;
  bool pa2_photo3_stable_release_latched;
  bool pa0_photo4_stable_release_latched;
  uint32_t pe13_photo1_released_since_ms;
  uint32_t pe9_photo2_released_since_ms;
  uint32_t pa2_photo3_released_since_ms;
  uint32_t pa0_photo4_released_since_ms;
  bool pole_target_seen_not_ready; /* Current step has observed pole target not ready. */
} auto_ctrl_template_ctx_t;

#ifdef __cplusplus
}
#endif
