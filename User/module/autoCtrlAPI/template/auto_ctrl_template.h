#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint8_t step_index;
  uint32_t step_enter_time_ms;
  uint32_t template_start_time_ms;
  bool step_entered;
} auto_ctrl_template_ctx_t;

#ifdef __cplusplus
}
#endif