/*
 * Pole auto step-climb state machine.
 */

#include "module/pole_auto.h"

#include <string.h>

static const Pole_AutoProfile_t k_pole_auto_profile_200mm = {
    .lift_target_front = 12.0f,
    .lift_target_rear = 12.0f,
    .retract_front_target = 0.0f,
    .retract_rear_target = 0.0f,
    .lift_speed_front = 8.0f,
    .lift_speed_rear = 8.0f,
    .retract_speed_front = 10.0f,
    .retract_speed_rear = 10.0f,
    .front_trigger_threshold_m = 0.20f,
    .rear_trigger_threshold_m = 0.20f,
    .trigger_hold_time_s = 0.05f,
    .stage_timeout_s = 3.0f,
    .arrive_threshold_rad = 0.08f,
};

static const Pole_AutoProfile_t k_pole_auto_profile_400mm = {
    .lift_target_front = 24.0f,
    .lift_target_rear = 24.0f,
    .retract_front_target = 0.0f,
    .retract_rear_target = 0.0f,
    .lift_speed_front = 10.0f,
    .lift_speed_rear = 10.0f,
    .retract_speed_front = 12.0f,
    .retract_speed_rear = 12.0f,
    .front_trigger_threshold_m = 0.40f,
    .rear_trigger_threshold_m = 0.40f,
    .trigger_hold_time_s = 0.05f,
    .stage_timeout_s = 4.0f,
    .arrive_threshold_rad = 0.10f,
};

static void Pole_AutoSetStage(Pole_Auto_t *a, Pole_AutoStage_t stage, uint32_t now) {
  a->stage = stage;
  a->stage_start_tick = now;
  a->trigger_start_tick = 0u;
}

static bool Pole_AutoStageTimeout(const Pole_Auto_t *a, const Pole_AutoProfile_t *profile,
                                  uint32_t now) {
  float elapsed = (float)(now - a->stage_start_tick) * 0.001f;
  return elapsed > profile->stage_timeout_s;
}

static bool Pole_AutoBothSensorsTriggered(const Pole_AutoSensor_t *sensor, uint8_t idx0,
                                          uint8_t idx1, float threshold_m) {
  return sensor->valid[idx0] && sensor->valid[idx1] &&
         (sensor->distance_m[idx0] <= threshold_m) &&
         (sensor->distance_m[idx1] <= threshold_m);
}

static bool Pole_AutoTriggerHeld(Pole_Auto_t *a, bool condition, const Pole_AutoProfile_t *profile,
                                 uint32_t now) {
  if (!condition) {
    a->trigger_start_tick = 0u;
    return false;
  }

  if (a->trigger_start_tick == 0u) {
    a->trigger_start_tick = now;
    return false;
  }

  return ((float)(now - a->trigger_start_tick) * 0.001f) >= profile->trigger_hold_time_s;
}

static void Pole_AutoFillCmd(Pole_CMD_t *pole_cmd, float front_target, float rear_target,
                             float front_speed, float rear_speed) {
  memset(pole_cmd, 0, sizeof(Pole_CMD_t));
  pole_cmd->mode = POLE_MODE_ACTIVE;
  pole_cmd->auto_target_enable[0] = true;
  pole_cmd->auto_target_enable[1] = true;
  pole_cmd->auto_target_lift[0] = front_target;
  pole_cmd->auto_target_lift[1] = rear_target;
  pole_cmd->auto_lift_speed[0] = front_speed;
  pole_cmd->auto_lift_speed[1] = rear_speed;
}

void Pole_AutoInit(Pole_Auto_t *a) {
  if (a == NULL) return;
  memset(a, 0, sizeof(Pole_Auto_t));
  a->stage = POLE_AUTO_STAGE_IDLE;
}

void Pole_AutoReset(Pole_Auto_t *a) {
  Pole_AutoInit(a);
}

const Pole_AutoProfile_t *Pole_AutoGetProfile(Pole_AutoStepType_t step_type) {
  switch (step_type) {
    case POLE_AUTO_STEP_200MM:
      return &k_pole_auto_profile_200mm;
    case POLE_AUTO_STEP_400MM:
      return &k_pole_auto_profile_400mm;
    case POLE_AUTO_STEP_NONE:
    default:
      return NULL;
  }
}

int8_t Pole_AutoUpdate(Pole_Auto_t *a, const Pole_t *pole, const Pole_AutoCmd_t *cmd,
                       const Pole_AutoSensor_t *sensor, Pole_CMD_t *pole_cmd, uint32_t now) {
  if (a == NULL || cmd == NULL || sensor == NULL || pole_cmd == NULL) return POLE_AUTO_ERR_NULL;

  memset(pole_cmd, 0, sizeof(Pole_CMD_t));

  if (!cmd->enable || cmd->step_type == POLE_AUTO_STEP_NONE) {
    Pole_AutoReset(a);
    pole_cmd->mode = POLE_MODE_RELAX;
    return POLE_AUTO_OK;
  }

  if (cmd->restart || !a->running || a->step_type != cmd->step_type) {
    Pole_AutoReset(a);
    a->running = true;
    a->step_type = cmd->step_type;
    Pole_AutoSetStage(a, POLE_AUTO_STAGE_LIFT_ALL, now);
  }

  const Pole_AutoProfile_t *profile = Pole_AutoGetProfile(a->step_type);
  if (profile == NULL) {
    a->error = true;
    Pole_AutoSetStage(a, POLE_AUTO_STAGE_ERROR, now);
    pole_cmd->mode = POLE_MODE_RELAX;
    return POLE_AUTO_ERR;
  }

  switch (a->stage) {
    case POLE_AUTO_STAGE_IDLE:
      pole_cmd->mode = POLE_MODE_RELAX;
      break;

    case POLE_AUTO_STAGE_LIFT_ALL:
      Pole_AutoFillCmd(pole_cmd, profile->lift_target_front, profile->lift_target_rear,
                       profile->lift_speed_front, profile->lift_speed_rear);
      if (pole != NULL && Pole_IsAllAtTarget(pole, profile->arrive_threshold_rad)) {
        Pole_AutoSetStage(a, POLE_AUTO_STAGE_WAIT_FRONT_TRIGGER, now);
      } else if (Pole_AutoStageTimeout(a, profile, now)) {
        a->error = true;
        Pole_AutoSetStage(a, POLE_AUTO_STAGE_ERROR, now);
      }
      break;

    case POLE_AUTO_STAGE_WAIT_FRONT_TRIGGER:
      Pole_AutoFillCmd(pole_cmd, profile->lift_target_front, profile->lift_target_rear,
                       profile->lift_speed_front, profile->lift_speed_rear);
      if (Pole_AutoTriggerHeld(a,
                               Pole_AutoBothSensorsTriggered(sensor, 0u, 1u,
                                                            profile->front_trigger_threshold_m),
                               profile, now)) {
        Pole_AutoSetStage(a, POLE_AUTO_STAGE_RETRACT_FRONT, now);
      } else if (Pole_AutoStageTimeout(a, profile, now)) {
        a->error = true;
        Pole_AutoSetStage(a, POLE_AUTO_STAGE_ERROR, now);
      }
      break;

    case POLE_AUTO_STAGE_RETRACT_FRONT:
      Pole_AutoFillCmd(pole_cmd, profile->retract_front_target, profile->lift_target_rear,
                       profile->retract_speed_front, profile->lift_speed_rear);
      if (pole != NULL && Pole_IsGroupAtTarget(pole, 0u, profile->arrive_threshold_rad)) {
        Pole_AutoSetStage(a, POLE_AUTO_STAGE_WAIT_REAR_TRIGGER, now);
      } else if (Pole_AutoStageTimeout(a, profile, now)) {
        a->error = true;
        Pole_AutoSetStage(a, POLE_AUTO_STAGE_ERROR, now);
      }
      break;

    case POLE_AUTO_STAGE_WAIT_REAR_TRIGGER:
      Pole_AutoFillCmd(pole_cmd, profile->retract_front_target, profile->lift_target_rear,
                       profile->retract_speed_front, profile->lift_speed_rear);
      if (Pole_AutoTriggerHeld(a,
                               Pole_AutoBothSensorsTriggered(sensor, 2u, 3u,
                                                            profile->rear_trigger_threshold_m),
                               profile, now)) {
        Pole_AutoSetStage(a, POLE_AUTO_STAGE_RETRACT_REAR, now);
      } else if (Pole_AutoStageTimeout(a, profile, now)) {
        a->error = true;
        Pole_AutoSetStage(a, POLE_AUTO_STAGE_ERROR, now);
      }
      break;

    case POLE_AUTO_STAGE_RETRACT_REAR:
      Pole_AutoFillCmd(pole_cmd, profile->retract_front_target, profile->retract_rear_target,
                       profile->retract_speed_front, profile->retract_speed_rear);
      if (pole != NULL && Pole_IsGroupAtTarget(pole, 1u, profile->arrive_threshold_rad)) {
        a->done = true;
        a->running = false;
        Pole_AutoSetStage(a, POLE_AUTO_STAGE_FINISHED, now);
      } else if (Pole_AutoStageTimeout(a, profile, now)) {
        a->error = true;
        Pole_AutoSetStage(a, POLE_AUTO_STAGE_ERROR, now);
      }
      break;

    case POLE_AUTO_STAGE_FINISHED:
      Pole_AutoFillCmd(pole_cmd, profile->retract_front_target, profile->retract_rear_target,
                       profile->retract_speed_front, profile->retract_speed_rear);
      break;

    case POLE_AUTO_STAGE_ERROR:
    default:
      a->error = true;
      pole_cmd->mode = POLE_MODE_RELAX;
      break;
  }

  return a->error ? POLE_AUTO_ERR : POLE_AUTO_OK;
}