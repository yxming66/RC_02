#include "module/shared_valve.h"

#include <stddef.h>

#ifndef ROD_NEW_GRIPPER_OPEN_LEVEL
#define ROD_NEW_GRIPPER_OPEN_LEVEL (0u)
#endif

#define SHARED_VALVE_OPEN_REQUEST false
#define SHARED_VALVE_CLOSED_REQUEST true

static bool SharedValve_RequestToOutput(bool request) {
  const bool open_level = (ROD_NEW_GRIPPER_OPEN_LEVEL != 0u);
  if (request == SHARED_VALVE_OPEN_REQUEST) {
    return open_level;
  }
  return !open_level;
}

static SharedValve_Debug_t shared_valve_debug = {
    .source_request = {false, false},
    .output_state = false,
    .conflict_count = 0u,
    .write_count = 0u,
    .gpio = BSP_GPIO_NONE,
};

static bool SharedValve_ResolveOutput(void) {
  return shared_valve_debug.source_request[SHARED_VALVE_SOURCE_ORE_STORE] ||
         shared_valve_debug.source_request[SHARED_VALVE_SOURCE_ROD] ||
         shared_valve_debug.source_request[SHARED_VALVE_SOURCE_ARM];
}

void SharedValve_Init(BSP_GPIO_t gpio) {
  shared_valve_debug.source_request[SHARED_VALVE_SOURCE_ORE_STORE] = false;
  shared_valve_debug.source_request[SHARED_VALVE_SOURCE_ROD] = false;
  shared_valve_debug.source_request[SHARED_VALVE_SOURCE_ARM] = false;
  shared_valve_debug.output_state = SharedValve_RequestToOutput(
      SHARED_VALVE_OPEN_REQUEST);
  shared_valve_debug.conflict_count = 0u;
  shared_valve_debug.write_count = 0u;
  shared_valve_debug.gpio = gpio;
  (void)BSP_GPIO_WritePin(gpio, shared_valve_debug.output_state);
}

void SharedValve_SetRequest(SharedValve_Source_t source, bool request) {
  if (source >= SHARED_VALVE_SOURCE_NUM) {
    return;
  }

  shared_valve_debug.source_request[source] = request;
}

void SharedValve_Output(void) {
  const bool ore_request =
      shared_valve_debug.source_request[SHARED_VALVE_SOURCE_ORE_STORE];
  const bool rod_request =
      shared_valve_debug.source_request[SHARED_VALVE_SOURCE_ROD];
  const bool output = SharedValve_RequestToOutput(SharedValve_ResolveOutput());

  if (ore_request != rod_request) {
    shared_valve_debug.conflict_count++;
  }

  shared_valve_debug.output_state = output;
  shared_valve_debug.write_count++;
  (void)BSP_GPIO_WritePin(shared_valve_debug.gpio, output);
}

void SharedValve_SetOreStoreRequest(bool closed) {
  SharedValve_SetRequest(SHARED_VALVE_SOURCE_ORE_STORE,
                         closed ? SHARED_VALVE_CLOSED_REQUEST
                                : SHARED_VALVE_OPEN_REQUEST);
}

void SharedValve_SetRodRequest(bool grab) {
  SharedValve_SetRequest(SHARED_VALVE_SOURCE_ROD,
                         grab ? SHARED_VALVE_CLOSED_REQUEST
                              : SHARED_VALVE_OPEN_REQUEST);
}

void SharedValve_SetArmRequest(bool suction_on) {
  SharedValve_SetRequest(SHARED_VALVE_SOURCE_ARM,
                         suction_on ? SHARED_VALVE_CLOSED_REQUEST
                                    : SHARED_VALVE_OPEN_REQUEST);
}

const SharedValve_Debug_t *SharedValve_GetDebug(void) {
  return &shared_valve_debug;
}
