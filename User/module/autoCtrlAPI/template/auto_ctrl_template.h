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
  uint32_t descend_start_move_time_ms; /* 下台阶首次冲刺起始时间，单位 ms。 */
  bool photo_stop_entered;       /* descend photo stop hold has started */
  bool descend_start_move_entered; /* 下台阶首次冲刺计时已启动。 */
  uint8_t step_index;            /* 当前模板执行到的 step 编号。 */
  uint32_t step_enter_time_ms;   /* 当前 step 首次进入时刻（ms）。 */
  uint32_t template_start_time_ms; /* 当前模板开始时刻（ms）。 */
  uint32_t final_photo_sprint_start_ms; /* 末尾光电触发后额外冲刺起始时刻（ms）。 */
  bool step_entered;             /* 当前 step 是否已经执行过 enter 初始化。 */
  bool final_photo_sprint_started; /* 末尾光电后的额外冲刺计时已启动。 */
  /* 光电稳定触发/释放检测状态。 */
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
  bool pa0_photo4_stable_low_seen;
  uint32_t pe13_photo1_released_since_ms;
  uint32_t pe9_photo2_released_since_ms;
  uint32_t pa2_photo3_released_since_ms;
  uint32_t pa0_photo4_released_since_ms;
  bool pole_target_seen_not_ready; /* 当前 step 已检测到支撑杆目标未到位。 */
  bool distance_latch_valid;        /* 轮编码器距离门控已锁存起点位置。 */
  float distance_start_wheel_rad[4]; /* 当前距离门控 step 入口的四轮位置。 */
  float wheel_delta_rad;            /* 当前距离门控 step 的四轮平均转角，单位 rad。 */
  bool descend_move_override_enabled;
  uint32_t descend_mid_move_ms;
  uint32_t descend_rear_retract_move_ms;
  float descend_rear_retract_move_wheel_delta_rad;
} auto_ctrl_template_ctx_t;

#ifdef __cplusplus
}
#endif
