/*
 * PC_COMM Task - 上位机通信任务
 */
#include <stddef.h>
#include <string.h>
#include <math.h>

#include "task/user_task.h"
#include "device/dr16.h"
#include "device/ore_info/ore_info.h"
#include "device/sick.h"
#include "module/mrlink_pc_comm/mrlink_pc_comm.h"
#include "module/config.h"
#include "module/autoCtrlAPI/api/auto_ctrl_api.h"
#include "module/arm_simple.h"
#include "module/camera_yaw.h"
#include "module/rod_new.h"
#include "main.h"

#define PC_COMM_TX_PERIOD_MS (20u)  /* 50Hz */
#define PC_COMM_LOOP_PERIOD_MS (10u)
#define PC_COMM_TX_DMA_TIMEOUT_MS (100u)
#define PC_COMM_CAMERA_YAW_CMD_TIMEOUT_MS (1000u)
#define PC_COMM_IR_ORE_TX_PERIOD_MS (200u)

extern volatile PC_CommandSource_t g_pc_command_source;
extern DR16_t dr16;

static uint8_t s_tx_buf[512];
static volatile bool s_tx_dma_busy = false;
static volatile uint32_t s_tx_dma_start_tick = 0;
static bool s_pc_comm_channel_inited = false;
static bool s_pc_comm_callbacks_registered = false;
static bool s_pc_comm_inited = false;
static uint32_t s_pc_comm_last_init_attempt_tick = 0u;
static uint32_t s_pc_comm_last_tx_tick = 0u;
static uint32_t s_ir_ore_last_tx_tick = 0u;
static uint32_t s_ir_ore_last_frame_rx_count = 0u;
static bool s_ir_ore_feedback_started = false;
static bool s_sick_front_ore_last_in_region = false;
static uint32_t s_sick_front_ore_stable_since_ms = 0u;
static const uint8_t s_feedback_cmds[] = {
    PC_FEEDBACK_HEARTBEAT,
    PC_FEEDBACK_AUTO_ACTION,
    PC_FEEDBACK_CHASSIS,
    PC_FEEDBACK_POLE,
    PC_FEEDBACK_ARM_SIMPLE,
    PC_FEEDBACK_ROD_NEW,
    PC_FEEDBACK_ORE_STORE,
    PC_FEEDBACK_CAMERA_YAW,
    PC_FEEDBACK_STEP,
    PC_FEEDBACK_SICK_CORRECT,
    PC_FEEDBACK_SICK_FRONT_ORE,
    PC_FEEDBACK_SICK_RAW,
    PC_FEEDBACK_IR_DOCK,
    PC_FEEDBACK_STATUS,
    /* PC_FEEDBACK_IR_ORE 不在自动循环里：由 PcComm_TryUpdateIrOreFeedback
     * 在首次收到/内容变化后持续低频追加，避免 50Hz 重复发送。 */
};

static void PcComm_DebugRecordInitFail(int8_t reason) {
    g_pc_comm_debug.init_fail_count++;
    g_pc_comm_debug.last_init_error = reason;
}

static void PcComm_DebugRecordTx(const uint8_t *data, uint16_t len,
                                 uint8_t frame_count, int8_t tx_result) {
    g_pc_comm_debug.tx_count++;
    g_pc_comm_debug.tx_len = len;
    g_pc_comm_debug.tx_frame_count = frame_count;
    g_pc_comm_debug.tx_result = tx_result;
    g_pc_comm_debug.tx_dma_busy = s_tx_dma_busy ? 1u : 0u;
}

static void PcComm_TxDoneCallback(void) {
    s_tx_dma_busy = false;
    g_pc_comm_debug.tx_dma_busy = 0u; 
}

static void PcComm_TxErrorCallback(void) {
    s_tx_dma_busy = false;
    g_pc_comm_debug.tx_dma_busy = 0u;
    g_pc_comm_debug.tx_dma_error_count++;
}

static AutoOre_DebugRequest_t PcComm_MapAutoAction(uint8_t action) {
    switch ((PC_AutoAction_t)action) {
        case PC_AUTO_ACTION_STORE:
            return AUTO_ORE_DEBUG_REQUEST_STORE;
        case PC_AUTO_ACTION_RELEASE:
            return AUTO_ORE_DEBUG_REQUEST_RELEASE;
        case PC_AUTO_ACTION_RELEASE_LIFT_DETECT:
            return AUTO_ORE_DEBUG_REQUEST_RELEASE_LIFT_DETECT;
        case PC_AUTO_ACTION_RELEASE_STEP1:
            return AUTO_ORE_DEBUG_REQUEST_RELEASE_STEP1;
        case PC_AUTO_ACTION_RELEASE_STEP2:
            return AUTO_ORE_DEBUG_REQUEST_RELEASE_STEP2;
        case PC_AUTO_ACTION_RELEASE_LIFT_DETECT_STEP1:
            return AUTO_ORE_DEBUG_REQUEST_RELEASE_LIFT_DETECT_STEP1;
        case PC_AUTO_ACTION_RELEASE_LIFT_DETECT_STEP2:
            return AUTO_ORE_DEBUG_REQUEST_RELEASE_LIFT_DETECT_STEP2;
        case PC_AUTO_ACTION_CHAMBER:
            return AUTO_ORE_DEBUG_REQUEST_CHAMBER;
        case PC_AUTO_ACTION_PICK_POS_400:
            return AUTO_ORE_DEBUG_REQUEST_PICK_POS_400;
        case PC_AUTO_ACTION_PICK_POS_200:
            return AUTO_ORE_DEBUG_REQUEST_PICK_POS_200;
        case PC_AUTO_ACTION_PICK_NEG_200:
            return AUTO_ORE_DEBUG_REQUEST_PICK_NEG_200;
        case PC_AUTO_ACTION_RECOVER_STORE:
            return AUTO_ORE_DEBUG_REQUEST_RECOVER_STORE;
        case PC_AUTO_ACTION_ROD_SPEARHEAD:
            return AUTO_ORE_DEBUG_REQUEST_ROD_SPEARHEAD;
        case PC_AUTO_ACTION_ROD_SPEARHEAD_STEP1:
            return AUTO_ORE_DEBUG_REQUEST_ROD_SPEARHEAD_STEP1;
        case PC_AUTO_ACTION_ROD_SPEARHEAD_STEP2:
            return AUTO_ORE_DEBUG_REQUEST_ROD_SPEARHEAD_STEP2;
        case PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS1:
            return AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ROD_SPEARHEAD_POS1;
        case PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS2:
            return AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ROD_SPEARHEAD_POS2;
        case PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS3:
            return AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ROD_SPEARHEAD_POS3;
        case PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS4:
            return AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ROD_SPEARHEAD_POS4;
        case PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS5:
            return AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ROD_SPEARHEAD_POS5;
        case PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS6:
            return AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ROD_SPEARHEAD_POS6;
        case PC_AUTO_ACTION_SICK_CORRECT_ORE_RELEASE:
            return AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ORE_RELEASE;
        case PC_AUTO_ACTION_ROD_DOCK_WAIT:
            return AUTO_ORE_DEBUG_REQUEST_ROD_DOCK_WAIT;
        case PC_AUTO_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
            return AUTO_ORE_DEBUG_REQUEST_STEP_PICK_STORE_ASCEND_200_HEAD;
        case PC_AUTO_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
            return AUTO_ORE_DEBUG_REQUEST_STEP_PICK_STORE_DESCEND_200_HEAD;
        case PC_AUTO_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
            return AUTO_ORE_DEBUG_REQUEST_STEP_PICK_STORE_ASCEND_400_HEAD;
        case PC_AUTO_ACTION_PICK_STORE_POS_400:
            return AUTO_ORE_DEBUG_REQUEST_PICK_STORE_POS_400;
        case PC_AUTO_ACTION_PICK_STORE_POS_200:
            return AUTO_ORE_DEBUG_REQUEST_PICK_STORE_POS_200;
        case PC_AUTO_ACTION_PICK_STORE_NEG_200:
            return AUTO_ORE_DEBUG_REQUEST_PICK_STORE_NEG_200;
        case PC_AUTO_ACTION_STEP_DROP_STORE_ASCEND_200_HEAD:
            return AUTO_ORE_DEBUG_REQUEST_STEP_DROP_STORE_ASCEND_200_HEAD;
        case PC_AUTO_ACTION_STEP_DROP_STORE_DESCEND_200_HEAD:
            return AUTO_ORE_DEBUG_REQUEST_STEP_DROP_STORE_DESCEND_200_HEAD;
        case PC_AUTO_ACTION_STEP_DROP_STORE_ASCEND_400_HEAD:
            return AUTO_ORE_DEBUG_REQUEST_STEP_DROP_STORE_ASCEND_400_HEAD;
        case PC_AUTO_ACTION_STEP_ASCEND_200_HEAD:
            return AUTO_ORE_DEBUG_REQUEST_STEP_ASCEND_200_HEAD;
        case PC_AUTO_ACTION_STEP_DESCEND_200_HEAD:
            return AUTO_ORE_DEBUG_REQUEST_STEP_DESCEND_200_HEAD;
        case PC_AUTO_ACTION_STEP_ASCEND_400_HEAD:
            return AUTO_ORE_DEBUG_REQUEST_STEP_ASCEND_400_HEAD;
        case PC_AUTO_ACTION_STEP_DESCEND_400_HEAD:
            return AUTO_ORE_DEBUG_REQUEST_STEP_DESCEND_400_HEAD;
        case PC_AUTO_ACTION_ABORT:
            return AUTO_ORE_DEBUG_REQUEST_ABORT;
        case PC_AUTO_ACTION_NONE:
        default:
            return AUTO_ORE_DEBUG_REQUEST_NONE;
    }
}

static bool PcComm_ProcessAutoActionCommand(void) {
    const PC_AutoActionCMD_t *cmd = MrlinkPc_GetAutoActionCMD();
    if (cmd == NULL) {
        return false;
    }

    const AutoOre_DebugRequest_t request = PcComm_MapAutoAction(cmd->action);
    if (request == AUTO_ORE_DEBUG_REQUEST_NONE) {
        MrlinkPc_ClearAutoActionCommand();
        return false;
    }

    const bool release_step2_continue =
        (request == AUTO_ORE_DEBUG_REQUEST_RELEASE_STEP2 ||
         request == AUTO_ORE_DEBUG_REQUEST_RELEASE_LIFT_DETECT_STEP2) &&
        auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl) &&
        (auto_ore_ctrl.action == AUTO_ORE_ACTION_RELEASE_STEP1 ||
         auto_ore_ctrl.action == AUTO_ORE_ACTION_RELEASE_LIFT_DETECT_STEP1) &&
        AutoOre_IsUpperFinished(&auto_ore_ctrl);

    if (g_auto_ore_debug.request != AUTO_ORE_DEBUG_REQUEST_NONE &&
        request != AUTO_ORE_DEBUG_REQUEST_ABORT &&
        !release_step2_continue) {
        return true;
    }

    if (request != AUTO_ORE_DEBUG_REQUEST_ABORT) {
        AutoCtrl_SetYawSource(&auto_ctrl, AUTO_CTRL_YAW_SOURCE_PC);
        AutoCtrl_SetYawZeroOffset(&auto_ctrl, 0.0f);
    }
    g_auto_ore_debug.request = request;
    MrlinkPc_ClearAutoActionCommand();
    return true;
}

static bool PcComm_AppendFeedbackFrame(uint8_t cmd, uint16_t *tx_len) {
    if (tx_len == NULL || *tx_len >= sizeof(s_tx_buf)) {
        return false;
    }

    uint16_t frame_len = 0u;
    frame_len = MrlinkPc_BuildFeedbackFrame(
        cmd, &s_tx_buf[*tx_len], (uint16_t)(sizeof(s_tx_buf) - *tx_len));
    if (frame_len == 0u) {
        return false;
    }

    *tx_len = (uint16_t)(*tx_len + frame_len);
    return true;
}

static bool PcComm_IsRodSpearheadSickCorrectAction(
    AutoSickCorrect_Action_t action) {
    return action >= AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS1 &&
           action <= AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS6;
}

static uint8_t PcComm_MapSickCorrectAction(AutoSickCorrect_Action_t action) {
    switch (action) {
        case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS1:
            return (uint8_t)PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS1;
        case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS2:
            return (uint8_t)PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS2;
        case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS3:
            return (uint8_t)PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS3;
        case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS4:
            return (uint8_t)PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS4;
        case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS5:
            return (uint8_t)PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS5;
        case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS6:
            return (uint8_t)PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS6;
        case AUTO_SICK_CORRECT_ACTION_NONE:
        case AUTO_SICK_CORRECT_ACTION_ORE_RELEASE:
        default:
            return (uint8_t)PC_AUTO_ACTION_NONE;
    }
}

static void PcComm_UpdateSickCorrectFeedback(void) {
    PC_SickCorrectFeedback_t feedback = {0};
    feedback.position_index = 0xFFu;

    if (!auto_sick_correct_inited) {
        (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_SICK_CORRECT, &feedback);
        return;
    }

    const AutoSickCorrect_Action_t action =
        AutoSickCorrect_GetAction(&auto_sick_correct_ctrl);
    if (PcComm_IsRodSpearheadSickCorrectAction(action)) {
        feedback.action = PcComm_MapSickCorrectAction(action);
        feedback.position_index =
            (uint8_t)(action - AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS1);
    }

    const uint8_t position_index =
        (feedback.position_index < AUTO_SICK_CORRECT_ROD_SPEARHEAD_POSITION_COUNT)
            ? feedback.position_index
            : 0u;
    const AutoSickCorrect_PointParams_t *param =
        &auto_sick_correct_ctrl.param.rod_spearhead_position[position_index];

    feedback.x_target_adc = param->x_target_adc;
    feedback.y_target_adc = param->y_target_adc;
    feedback.x_sample_adc = auto_sick_correct_ctrl.x_sample_adc;
    feedback.y_sample_adc = auto_sick_correct_ctrl.y_sample_adc;

    Sick_Output_t sick_output = {0};
    if (Task_SickGetLatestOutput(&sick_output)) {
        if (param->front_index < SICK_OUTPUT_CHANNEL_COUNT &&
            sick_output.valid[param->front_index]) {
            feedback.x_sample_adc = (float)sick_output.adc_raw[param->front_index];
            feedback.valid_mask |= PC_SICK_CORRECT_VALID_X;
        }
        if (param->rod_rear_index < SICK_OUTPUT_CHANNEL_COUNT &&
            sick_output.valid[param->rod_rear_index]) {
            feedback.y_sample_adc = (float)sick_output.adc_raw[param->rod_rear_index];
            feedback.valid_mask |= PC_SICK_CORRECT_VALID_Y;
        }
    }

    if (feedback.x_target_adc > 0.0f || feedback.x_sample_adc > 0.0f) {
        feedback.valid_mask |= PC_SICK_CORRECT_VALID_X;
    }
    if (feedback.y_target_adc > 0.0f || feedback.y_sample_adc > 0.0f) {
        feedback.valid_mask |= PC_SICK_CORRECT_VALID_Y;
    }

    (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_SICK_CORRECT, &feedback);
}

static void PcComm_UpdateSickFrontOreFeedback(void) {
    Sick_FrontOreDetect_t detect = {0};
    PC_SickFrontOreFeedback_t feedback = {0};
    const Config_RobotParam_t *cfg = Config_GetRobotParam();
    const AutoCtrl_CommonParam_t *common =
        (cfg != NULL) ? &cfg->auto_ctrl_param.common : NULL;
    const uint16_t adc_min =
        (common != NULL) ? common->sick_front_ore_adc_min : 0u;
    const uint16_t adc_max =
        (common != NULL) ? common->sick_front_ore_adc_max : 0u;
    const uint32_t stable_ms =
        (common != NULL) ? common->sick_front_ore_stable_ms : 0u;
    const uint32_t now_ms = BSP_TIME_Get_ms();

    if (SICK_GetFrontOreDetect(&detect)) {
        const bool adc_window_valid = adc_min <= adc_max;
        const bool in_region = detect.sample_valid && adc_window_valid &&
                               detect.adc_raw >= adc_min &&
                               detect.adc_raw <= adc_max;
        if (in_region != s_sick_front_ore_last_in_region) {
            s_sick_front_ore_last_in_region = in_region;
            s_sick_front_ore_stable_since_ms = in_region ? now_ms : 0u;
        } else if (in_region && s_sick_front_ore_stable_since_ms == 0u) {
            s_sick_front_ore_stable_since_ms = now_ms;
        }

        feedback.detected =
            (in_region &&
             (stable_ms == 0u ||
              (uint32_t)(now_ms - s_sick_front_ore_stable_since_ms) >=
                  stable_ms))
                ? 1u
                : 0u;
    }

    (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_SICK_FRONT_ORE, &feedback);
}

static void PcComm_UpdateSickRawFeedback(void) {
    Sick_Output_t output = {0};
    PC_SickRawFeedback_t feedback = {0};

    if (Task_SickGetLatestOutput(&output)) {
        feedback.update_tick = output.update_tick;
        feedback.miss_count = output.miss_count;
        for (uint8_t i = 0u; i < SICK_OUTPUT_CHANNEL_COUNT && i < 4u; i++) {
            feedback.distance_mm[i] = output.distance_mm[i];
            feedback.adc_raw[i] = output.adc_raw[i];
            if (output.valid[i]) {
                feedback.valid_mask |= (uint8_t)(1u << i);
            }
        }
    }

    (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_SICK_RAW, &feedback);
}

static bool PcComm_AppendStartMatchFrame(uint16_t *tx_len) {
    if (tx_len == NULL || *tx_len >= sizeof(s_tx_buf) ||
        !MrlinkPc_HasStartMatchRequest()) {
        return false;
    }

    const uint16_t frame_len = MrlinkPc_BuildStartMatchFrame(
        &s_tx_buf[*tx_len], (uint16_t)(sizeof(s_tx_buf) - *tx_len));
    if (frame_len == 0u) {
        return false;
    }

    *tx_len = (uint16_t)(*tx_len + frame_len);
    return true;
}

static bool PcComm_AppendRetryFrame(uint16_t *tx_len) {
    if (tx_len == NULL || *tx_len >= sizeof(s_tx_buf) ||
        !MrlinkPc_HasRetryRequest()) {
        return false;
    }

    const uint16_t frame_len = MrlinkPc_BuildRetryFrame(
        &s_tx_buf[*tx_len], (uint16_t)(sizeof(s_tx_buf) - *tx_len));
    if (frame_len == 0u) {
        return false;
    }

    *tx_len = (uint16_t)(*tx_len + frame_len);
    return true;
}

static void PcComm_UpdateModuleFeedback(void) {
    const ArmSimple_Feedback_t *arm_fb = Task_ArmSimpleGetFeedback();
    if (arm_fb != NULL) {
        PC_ArmSimpleFeedback_t pc_arm = {0};
        pc_arm.mode = (uint8_t)arm_fb->mode;
        pc_arm.point_mode = (uint8_t)arm_fb->point_mode;
        pc_arm.suction = (uint8_t)arm_fb->suction;
        pc_arm.joint1_angle_rad = arm_fb->joint1_angle_rad;
        pc_arm.joint1_velocity_rad_s = arm_fb->joint1_velocity_rad_s;
        pc_arm.joint2_angle_rad = arm_fb->joint2_angle_rad;
        (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_ARM_SIMPLE, &pc_arm);
    }

    const RodNew_Feedback_t *rod_fb = Task_RodNewGetFeedback();
    if (rod_fb != NULL) {
        PC_RodNewFeedback_t pc_rod = {0};
        pc_rod.mode = (uint8_t)rod_fb->mode;
        pc_rod.pose = (uint8_t)rod_fb->pose;
        pc_rod.grip = (uint8_t)rod_fb->grip;
        pc_rod.at_target = rod_fb->at_target ? 1u : 0u;
        pc_rod.target_angle_rad = rod_fb->target_angle_rad;
        pc_rod.tracked_angle_rad = rod_fb->tracked_angle_rad;
        pc_rod.tracked_velocity_rad_s = rod_fb->tracked_velocity_rad_s;
        pc_rod.feedback_angle_rad = rod_fb->feedback_angle_rad;
        (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_ROD_NEW, &pc_rod);
    }

    const OreStore_Feedback_t *ore_fb = Task_OreStoreGetFeedback();
    if (ore_fb != NULL) {
        const OreStore_CMD_t *ore_cmd = Task_OreStoreGetCommand();
        PC_OreStoreFeedback_t pc_ore = {0};
        pc_ore.mode = (ore_cmd != NULL) ? ore_cmd->mode : 0u;
        pc_ore.all_homed = ore_fb->all_homed ? 1u : 0u;
        pc_ore.transform_low_has_ore =
            g_auto_ore_debug.transform_low_has_ore ? 1u : 0u;
        pc_ore.transform_high_has_ore =
            g_auto_ore_debug.transform_high_has_ore ? 1u : 0u;
        pc_ore.arm_has_ore = g_auto_ore_debug.arm_has_ore ? 1u : 0u;
        pc_ore.release_grid_has_ore =
            g_auto_ore_debug.release_grid_has_ore ? 1u : 0u;
        for (uint8_t axis = 0u; axis < ORE_STORE_AXIS_NUM; ++axis) {
            if (ore_fb->online[axis]) {
                pc_ore.online_mask |= (uint8_t)(1u << axis);
            }
            if (ore_fb->homed[axis]) {
                pc_ore.homed_mask |= (uint8_t)(1u << axis);
            }
        }
        pc_ore.platform_position_rad = ore_fb->position_rad[ORE_STORE_AXIS_PLATFORM];
        (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_ORE_STORE, &pc_ore);
    }

    const CameraYaw_GroupFeedback_t *camera_fb = Task_CameraYawGetGroupFeedback();
    if (camera_fb != NULL) {
        PC_CameraYawFeedback_t pc_camera = {0};
        const CameraYaw_Feedback_t *fb = &camera_fb->yaw[CAMERA_YAW_RIGHT];
        pc_camera.mode = (uint8_t)fb->mode;
        pc_camera.motor_online = fb->motor_online ? 1u : 0u;
        pc_camera.feedback_valid = fb->feedback_valid ? 1u : 0u;
        pc_camera.at_target = fb->at_target ? 1u : 0u;
        pc_camera.target_yaw_rad = fb->target_yaw_rad;
        pc_camera.feedback_yaw_rad = fb->feedback_yaw_rad;
        pc_camera.error_yaw_rad = fb->error_yaw_rad;
        pc_camera.motor_angle_rad = fb->motor_angle_rad;
        pc_camera.motor_velocity_rad_s = fb->motor_velocity_rad_s;
        pc_camera.output = fb->output;
        pc_camera.feedback_age_ms = fb->feedback_age_ms;
        (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_CAMERA_YAW, &pc_camera);
    }
}

static bool PcComm_CameraYawCommandFresh(const MrlinkPc_State_t *state,
                                         uint32_t now_ms) {
    return state != NULL && state->camera_yaw_cmd_tick != 0u;
    // return state != NULL && state->camera_yaw_cmd_tick != 0u &&
    //        (now_ms - state->camera_yaw_cmd_tick) <=
    //            PC_COMM_CAMERA_YAW_CMD_TIMEOUT_MS;
}

static bool PcComm_ShouldRelaxCameraYawByRcSwitch(void) {
    return !dr16.header.online ||
           (dr16.data.sw_l == DR16_SW_UP &&
            (dr16.data.sw_r == DR16_SW_MID ||
             dr16.data.sw_r == DR16_SW_DOWN));
}

static bool PcComm_ShouldUsePcCameraYawCommand(void) {
    return dr16.header.online && dr16.data.sw_l == DR16_SW_UP &&
           dr16.data.sw_r == DR16_SW_UP && MrlinkPc_IsPCControlMode();
}

static float PcComm_GetCameraYawFeedbackRad(uint8_t yaw, float fallback) {
    const CameraYaw_GroupFeedback_t *camera_fb = Task_CameraYawGetGroupFeedback();
    if (camera_fb == NULL || yaw >= CAMERA_YAW_NUM) {
        return fallback;
    }

    const float feedback_yaw_rad = camera_fb->yaw[yaw].feedback_yaw_rad;
    return isfinite(feedback_yaw_rad) ? feedback_yaw_rad : fallback;
}

static void PcComm_UpdateCameraYawCommand(uint32_t now_ms) {
    CameraYaw_GroupCMD_t cmd = {0};
    for (uint8_t yaw = 0u; yaw < CAMERA_YAW_NUM; ++yaw) {
        cmd.yaw[yaw].mode = CAMERA_YAW_MODE_ACTIVE;
        cmd.yaw[yaw].target_yaw_rad = 0.0f;
        cmd.yaw[yaw].feedback_tick_ms = now_ms;
        cmd.yaw[yaw].feedback_valid = true;
    }

    if (PcComm_ShouldRelaxCameraYawByRcSwitch()) {
        for (uint8_t yaw = 0u; yaw < CAMERA_YAW_NUM; ++yaw) {
            cmd.yaw[yaw].mode = CAMERA_YAW_MODE_RELAX;
            cmd.yaw[yaw].target_yaw_rad = 0.0f;
            cmd.yaw[yaw].feedback_tick_ms = now_ms;
            cmd.yaw[yaw].feedback_valid = false;
        }
        (void)Task_CameraYawPostGroupCommand(&cmd);
        return;
    }

    const PC_CameraYawCMD_t *pc_cmd = MrlinkPc_GetCameraYawCMD();
    const MrlinkPc_State_t *state = MrlinkPc_GetState();
    if (PcComm_ShouldUsePcCameraYawCommand() && pc_cmd != NULL &&
        PcComm_CameraYawCommandFresh(state, now_ms)) {
        cmd.yaw[CAMERA_YAW_RIGHT].mode = CAMERA_YAW_MODE_ACTIVE;
        cmd.yaw[CAMERA_YAW_RIGHT].target_yaw_rad =
            isfinite(pc_cmd->target_yaw_rad)
                ? pc_cmd->target_yaw_rad
                : PcComm_GetCameraYawFeedbackRad(CAMERA_YAW_RIGHT, 0.0f);
        cmd.yaw[CAMERA_YAW_RIGHT].feedback_tick_ms = now_ms;
        cmd.yaw[CAMERA_YAW_RIGHT].feedback_valid = true;
        (void)Task_CameraYawPostGroupCommand(&cmd);
        return;
    }

    (void)Task_CameraYawPostGroupCommand(&cmd);
}

static bool PcComm_TryUpdateIrOreFeedback(uint32_t now_ms) {
    OreInfo_Info_t info = {0};
    if (!OreInfo_GetInfo(&info, now_ms)) {
        return false;
    }

    const bool new_frame = !s_ir_ore_feedback_started ||
                           info.frame_rx_count != s_ir_ore_last_frame_rx_count;
    const bool periodic = s_ir_ore_feedback_started &&
                          (now_ms - s_ir_ore_last_tx_tick) >=
                              PC_COMM_IR_ORE_TX_PERIOD_MS;
    if (!new_frame && !periodic) {
        return false;
    }

    PC_IrOreFeedback_t feedback = {0};
    feedback.valid = info.valid ? 1u : 0u;
    feedback.fresh = info.fresh ? 1u : 0u;
    feedback.status = 0u;
    feedback.count = info.count;
    for (uint8_t i = 0u; i < PC_IR_ORE_POSITION_COUNT; ++i) {
        feedback.ore_type[i] = info.ore_type[i];
    }
    feedback.age_ms = info.age_ms;
    feedback.rx_count = info.rx_count;
    (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_IR_ORE, &feedback);

    PC_IrOreBridgeFeedback_t bridge = {0};
    bridge.valid = info.valid ? 1u : 0u;
    bridge.fresh = info.fresh ? 1u : 0u;
    bridge.status = 0u;
    bridge.count = info.count;
    bridge.msg_id = info.msg_id;
    bridge.side = info.side;
    bridge.ack_pending = info.ack_pending;
    bridge.parse_status = info.parse_status;
    for (uint8_t i = 0u; i < PC_IR_ORE_POSITION_COUNT; ++i) {
        bridge.ore_type[i] = info.ore_type[i];
    }
    for (uint8_t i = 0u; i < PC_IR_ORE_RAW_FRAME_SIZE; ++i) {
        bridge.raw_frame[i] = info.raw_frame[i];
    }
    bridge.age_ms = info.age_ms;
    bridge.rx_count = info.rx_count;
    bridge.frame_rx_count = info.frame_rx_count;
    bridge.ack_tx_count = info.ack_tx_count;
    (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_IR_ORE_BRIDGE, &bridge);

    s_ir_ore_feedback_started = true;
    s_ir_ore_last_frame_rx_count = info.frame_rx_count;
    s_ir_ore_last_tx_tick = now_ms;
    return true;
}

static void PcComm_UpdateStatusFeedback(void) {
    const MrlinkPc_State_t *state = MrlinkPc_GetState();
    PC_StatusFeedback_t status = {0};
    status.online = MrlinkPc_CommIsOnline() ? 1u : 0u;
    status.recv_count = (state != NULL) ? state->recv_count : 0u;
    status.cpu_temp = task_runtime.status.cpu_temp;
    status.command_source = g_pc_command_source;
    (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_STATUS, &status);
}

static void PcComm_UpdateIrDockFeedback(uint32_t now_ms) {
    PC_IrDockFeedback_t feedback = {0};
    feedback.valid = (g_ir_dock_debug.protocol_frame_rx_count > 0u) ? 1u : 0u;
    feedback.fresh = IrDock_IsOnline(now_ms) ? 1u : 0u;
    feedback.dock_complete = IrDock_IsDockCompleteFresh(now_ms) ? 1u : 0u;
    feedback.r2_leave_zone1_allowed =
        IrDock_IsR2LeaveZone1Allowed() ? 1u : 0u;
    feedback.release_lift_step2_ready =
        IrDock_IsReleaseLiftStep2Ready() ? 1u : 0u;
    feedback.cleared_ore_id = IrDock_GetLastClearedOreId();
    feedback.zone3_r2_state = IrDock_GetLastZone3R2State();
    feedback.last_dock_complete_cmd = g_ir_dock_debug.last_dock_complete_cmd;
    feedback.last_r2_leave_zone1_cmd =
        g_ir_dock_debug.last_r2_leave_zone1_cmd;
    feedback.last_release_lift_step2_cmd =
        g_ir_dock_debug.last_release_lift_step2_cmd;
    feedback.age_ms = g_ir_dock_debug.last_rx_age_ms;
    feedback.rx_count = g_ir_dock_debug.protocol_frame_rx_count;
    feedback.crc_error_count = g_ir_dock_debug.crc_error_count;
    feedback.error_count = g_ir_dock_debug.error_count;
    (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_IR_DOCK, &feedback);
}

static void PcComm_ProcessIrOreAckCommand(uint32_t now_ms) {
    (void)now_ms;
    if (MrlinkPc_GetIrOreAckCMD() != NULL) {
        g_pc_comm_debug.ir_ore_ack_submit_result = 0xFFu;
        g_pc_comm_debug.ir_ore_ack_submit_error_count++;
        MrlinkPc_ClearIrOreAckCommand();
    }
}

static bool PcComm_TransmitFeedback(void) {
    uint16_t tx_len = 0;
    uint8_t frame_count = 0;
    uint32_t now = BSP_TIME_Get_ms();

    if (s_tx_dma_busy) {
        if ((now - s_tx_dma_start_tick) < PC_COMM_TX_DMA_TIMEOUT_MS) {
            g_pc_comm_debug.tx_dma_busy = 1u;
            g_pc_comm_debug.tx_busy_skip_count++;
            return false;
        }
        s_tx_dma_busy = false;
        g_pc_comm_debug.tx_dma_busy = 0u;
        g_pc_comm_debug.tx_dma_error_count++;
    }

    PcComm_UpdateStatusFeedback();
    PcComm_UpdateModuleFeedback();
    PcComm_UpdateSickCorrectFeedback();
    PcComm_UpdateSickFrontOreFeedback();
    PcComm_UpdateSickRawFeedback();
    PcComm_UpdateIrDockFeedback(now);
    const bool ir_ore_pending = PcComm_TryUpdateIrOreFeedback(now);
    const bool start_match_pending = PcComm_AppendStartMatchFrame(&tx_len);
    if (start_match_pending) {
        frame_count++;
    }
    const bool retry_pending = PcComm_AppendRetryFrame(&tx_len);
    if (retry_pending) {
        frame_count++;
    }

    for (uint8_t i = 0; i < (uint8_t)(sizeof(s_feedback_cmds) / sizeof(s_feedback_cmds[0])); ++i) {
        if (PcComm_AppendFeedbackFrame(s_feedback_cmds[i], &tx_len)) {
            frame_count++;
        }
    }

    /* IR_ORE 首次收到/内容变化后持续低频追加帧，避免 50Hz 重复刷屏。 */
    if (ir_ore_pending) {
        if (PcComm_AppendFeedbackFrame(PC_FEEDBACK_IR_ORE, &tx_len)) {
            frame_count++;
        }
        if (PcComm_AppendFeedbackFrame(PC_FEEDBACK_IR_ORE_BRIDGE, &tx_len)) {
            frame_count++;
        }
    }

    if (tx_len > 0u) {
        s_tx_dma_busy = true;
        s_tx_dma_start_tick = now;
        int8_t tx_result = MrlinkPc_SendFrame(s_tx_buf, tx_len);
        if (tx_result != MRLINK_CHANNEL_OK) {
            s_tx_dma_busy = false;
            g_pc_comm_debug.tx_dma_error_count++;
        } else {
            if (start_match_pending) {
                MrlinkPc_ClearStartMatchRequest();
            }
            if (retry_pending) {
                MrlinkPc_ClearRetryRequest();
            }
        }
        PcComm_DebugRecordTx(s_tx_buf, tx_len, frame_count, tx_result);
        return tx_result == MRLINK_CHANNEL_OK;
    }

    PcComm_DebugRecordTx(s_tx_buf, 0, frame_count, MRLINK_CHANNEL_ERR);
    return false;
}

bool Task_PcCommInitOnce(void) {
    if (s_pc_comm_inited) {
        return true;
    }

    const uint32_t now = BSP_TIME_Get_ms();
    if (s_pc_comm_last_init_attempt_tick != 0u &&
        (now - s_pc_comm_last_init_attempt_tick) < 20u) {
        return false;
    }
    s_pc_comm_last_init_attempt_tick = now;

    if (!s_pc_comm_channel_inited) {
        if (!MrlinkPc_CommInit()) {
            PcComm_DebugRecordInitFail(g_pc_comm_debug.last_init_error);
            return false;
        }
        s_pc_comm_channel_inited = true;
    }

    if (!s_pc_comm_callbacks_registered) {
        if (MrlinkPc_RegisterTxCallbacks(PcComm_TxDoneCallback,
                                         PcComm_TxErrorCallback) !=
            MRLINK_CHANNEL_OK) {
            PcComm_DebugRecordInitFail(-30);
            return false;
        }
        s_pc_comm_callbacks_registered = true;
    }

    s_pc_comm_last_tx_tick = now;
    s_pc_comm_inited = true;
    return true;
}

void Task_PcCommStep(void) {
    if (!s_pc_comm_inited) {
        return;
    }

    const uint32_t profile_start_us =
        Task_ProfilerLoopBegin(TASK_PROFILE_PC_COMM,
                               PC_COMM_LOOP_PERIOD_MS * 1000U);

    uint32_t now = BSP_TIME_Get_ms();

    MrlinkPc_CommProcess(now);
    PcComm_ProcessIrOreAckCommand(now);
    PcComm_UpdateCameraYawCommand(now);

    if ((now - s_pc_comm_last_tx_tick) >= PC_COMM_TX_PERIOD_MS) {
        (void)PcComm_TransmitFeedback();
        s_pc_comm_last_tx_tick = now;
    }

    if (MrlinkPc_IsPCControlMode()) {
        const bool auto_action_pending = PcComm_ProcessAutoActionCommand();
        const PC_AutoStepParams_t *step_params =
            (g_pc_command_source == PC_COMMAND_SOURCE_PC && auto_ctrl_inited &&
             !auto_action_pending)
                ? MrlinkPc_GetAutoStepParams()
                : NULL;
        if (step_params != NULL && !AutoCtrl_IsBusy(&auto_ctrl) &&
            !(auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl)) &&
            !Task_AutoRodSpearheadIsBusy() &&
            !Task_AutoSickCorrectIsBusy()) {
            AutoCtrl_SetYawSource(&auto_ctrl, AUTO_CTRL_YAW_SOURCE_PC);
            AutoCtrl_SetYawZeroOffset(&auto_ctrl, 0.0f);
            if (AutoCtrl_StartTemplate(
                    &auto_ctrl,
                    step_params->template_id,
                    step_params->travel_dir,
                    step_params->target_yaw_rad,
                    step_params->yaw_tolerance_rad,
                    AUTO_CTRL_SENSOR_MODE_NONE,
                    now)) {
                MrlinkPc_ClearStepCommand();
            }
        }
    }

    task_runtime.heartbeat.pc_comm++;
    Task_ProfilerLoopEnd(TASK_PROFILE_PC_COMM, profile_start_us);
}

void Task_pc_comm(void *argument) {
    (void)argument;

    uint32_t delay_tick =
        (osKernelGetTickFreq() * PC_COMM_LOOP_PERIOD_MS) / 1000U;
    if (delay_tick == 0U) {
        delay_tick = 1U;
    }

    while (!Task_PcCommInitOnce()) {
        osDelay(20u);
    }

    uint32_t tick = osKernelGetTickCount();
    while (1) {
        tick += delay_tick;
        Task_PcCommStep();
        Task_DelayUntil(TASK_PROFILE_PC_COMM, &tick, delay_tick);
    }
}

void Task_pc_comm_sick(void *argument) {
    (void)argument;

    uint32_t base_delay_tick =
        osKernelGetTickFreq() / (uint32_t)TASK_FUSED_LOOP_FREQ;
    if (base_delay_tick == 0U) {
        base_delay_tick = 1U;
    }

    const uint32_t now_us = (uint32_t)BSP_TIME_Get_us();
    Task_SubtaskTimer_t pc_comm_timer;
    Task_SubtaskTimer_t sick_timer;
    Task_SubtaskTimerInit(&pc_comm_timer,
                          1000.0f / (float)PC_COMM_LOOP_PERIOD_MS, now_us);
    Task_SubtaskTimerInit(&sick_timer, SICK_FREQ, now_us);

    uint32_t tick = osKernelGetTickCount();
    uint32_t sick_init_tick = tick + SICK_INIT_DELAY;
    bool sick_init_due = (SICK_INIT_DELAY == 0U);

    while (1) {
        const uint32_t profile_start_us =
            Task_ProfilerLoopBegin(TASK_PROFILE_PC_COMM_SICK,
                                   TASK_PERIOD_US(TASK_FUSED_LOOP_FREQ));
        tick += base_delay_tick;
        const uint32_t loop_now_us = (uint32_t)BSP_TIME_Get_us();

        if (Task_SubtaskTimerDue(&pc_comm_timer, loop_now_us)) {
            if (Task_PcCommInitOnce()) {
                Task_PcCommStep();
            }
        }

        if (!sick_init_due &&
            (int32_t)(osKernelGetTickCount() - sick_init_tick) >= 0) {
            sick_init_due = true;
        }
        if (sick_init_due && Task_SubtaskTimerDue(&sick_timer, loop_now_us)) {
            if (Task_SickInitOnce()) {
                Task_SickStep();
            }
        }

        task_runtime.heartbeat.pc_comm_sick++;
        Task_ProfilerLoopEnd(TASK_PROFILE_PC_COMM_SICK, profile_start_us);
        Task_DelayUntil(TASK_PROFILE_PC_COMM_SICK, &tick, base_delay_tick);
    }
}
