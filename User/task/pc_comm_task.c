/*
 * PC_COMM Task - 上位机通信任务
 */
#include <stddef.h>
#include <string.h>
#include <math.h>

#include "task/user_task.h"
#include "device/dr16.h"
#include "module/mrlink_pc_comm/mrlink_pc_comm.h"
#include "module/autoCtrlAPI/api/auto_ctrl_api.h"
#include "module/arm_simple.h"
#include "module/camera_yaw.h"
#include "module/rod_new.h"
#include "main.h"

#define PC_COMM_TX_PERIOD_MS (20u)  /* 50Hz */
#define PC_COMM_LOOP_PERIOD_MS (10u)
#define PC_COMM_TX_DMA_TIMEOUT_MS (100u)
#define PC_COMM_CAMERA_YAW_CMD_TIMEOUT_MS (1000u)
#define PC_COMM_CAMERA_YAW_CHANNEL CAMERA_YAW_RIGHT
/* IR_ORE 12 位置矿种类只在对端发来时变一次，正常状态几乎不变。
 * 仅在数据变化或距上次发送超过心跳周期时才发，避免 50Hz 重复刷屏。 */
#define PC_COMM_IR_ORE_HEARTBEAT_MS (500u)

extern volatile PC_CommandSource_t g_pc_command_source;
extern DR16_t dr16;

static uint8_t s_tx_buf[384];
static volatile bool s_tx_dma_busy = false;
static volatile uint32_t s_tx_dma_start_tick = 0;
static bool s_pc_comm_channel_inited = false;
static bool s_pc_comm_callbacks_registered = false;
static bool s_pc_comm_inited = false;
static uint32_t s_pc_comm_last_init_attempt_tick = 0u;
static uint32_t s_pc_comm_last_tx_tick = 0u;
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
    PC_FEEDBACK_STATUS,
    /* PC_FEEDBACK_IR_ORE 不在自动循环里：由 PcComm_TryUpdateIrOreFeedback
     * 判定变更或心跳到期后单独追加，避免 50Hz 重复发送。 */
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
        case PC_AUTO_ACTION_RELEASE_STEP1:
            return AUTO_ORE_DEBUG_REQUEST_RELEASE_STEP1;
        case PC_AUTO_ACTION_RELEASE_STEP2:
            return AUTO_ORE_DEBUG_REQUEST_RELEASE_STEP2;
        case PC_AUTO_ACTION_RELEASE_LIFT_DETECT:
            return AUTO_ORE_DEBUG_REQUEST_RELEASE_LIFT_DETECT;
        case PC_AUTO_ACTION_CHAMBER:
            return AUTO_ORE_DEBUG_REQUEST_CHAMBER;
        case PC_AUTO_ACTION_PICK_POS_400:
            return AUTO_ORE_DEBUG_REQUEST_PICK_POS_400;
        case PC_AUTO_ACTION_PICK_POS_200:
            return AUTO_ORE_DEBUG_REQUEST_PICK_POS_200;
        case PC_AUTO_ACTION_PICK_NEG_200:
            return AUTO_ORE_DEBUG_REQUEST_PICK_NEG_200;
        case PC_AUTO_ACTION_ROD_SPEARHEAD:
            return AUTO_ORE_DEBUG_REQUEST_ROD_SPEARHEAD;
        case PC_AUTO_ACTION_ROD_SPEARHEAD_STEP1:
            return AUTO_ORE_DEBUG_REQUEST_ROD_SPEARHEAD_STEP1;
        case PC_AUTO_ACTION_ROD_SPEARHEAD_STEP2:
            return AUTO_ORE_DEBUG_REQUEST_ROD_SPEARHEAD_STEP2;
        case PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD:
            return AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ROD_SPEARHEAD;
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

    if (g_auto_ore_debug.request != AUTO_ORE_DEBUG_REQUEST_NONE &&
        request != AUTO_ORE_DEBUG_REQUEST_ABORT) {
        MrlinkPc_ClearAutoActionCommand();
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
        const PC_OreStoreCMD_t *ore_cmd = MrlinkPc_GetOreStoreCMD();
        PC_OreStoreFeedback_t pc_ore = {0};
        pc_ore.mode = (ore_cmd != NULL) ? ore_cmd->mode : 0u;
        pc_ore.all_homed = ore_fb->all_homed ? 1u : 0u;
        pc_ore.transform_low_has_ore =
            g_auto_ore_debug.transform_low_has_ore ? 1u : 0u;
        pc_ore.transform_high_has_ore =
            g_auto_ore_debug.transform_high_has_ore ? 1u : 0u;
        pc_ore.arm_has_ore = g_auto_ore_debug.arm_has_ore ? 1u : 0u;
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
        const CameraYaw_Feedback_t *fb =
            &camera_fb->yaw[PC_COMM_CAMERA_YAW_CHANNEL];
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
        cmd.yaw[PC_COMM_CAMERA_YAW_CHANNEL].mode = CAMERA_YAW_MODE_ACTIVE;
        cmd.yaw[PC_COMM_CAMERA_YAW_CHANNEL].target_yaw_rad =
            isfinite(pc_cmd->target_yaw_rad)
                ? pc_cmd->target_yaw_rad
                : PcComm_GetCameraYawFeedbackRad(PC_COMM_CAMERA_YAW_CHANNEL,
                                                 0.0f);
        cmd.yaw[PC_COMM_CAMERA_YAW_CHANNEL].feedback_tick_ms = now_ms;
        cmd.yaw[PC_COMM_CAMERA_YAW_CHANNEL].feedback_valid = true;
        (void)Task_CameraYawPostGroupCommand(&cmd);
        return;
    }

    (void)Task_CameraYawPostGroupCommand(&cmd);
}

static PC_IrOreFeedback_t s_last_published_ir_ore = {0};
static PC_IrOreBridgeFeedback_t s_last_published_ir_ore_bridge = {0};
static bool s_last_published_ir_ore_inited = false;
static uint32_t s_last_ir_ore_publish_ms = 0u;

static bool PcComm_TryUpdateIrOreFeedback(uint32_t now_ms) {
    IrDock_OreInfo_t ir_info = {0};
    PC_IrOreFeedback_t pc_ir_ore = {0};
    PC_IrOreBridgeFeedback_t pc_ir_ore_bridge = {0};

    (void)IrDock_GetOreInfo(&ir_info, now_ms);
    pc_ir_ore.valid = ir_info.valid ? 1u : 0u;
    pc_ir_ore.fresh = ir_info.fresh ? 1u : 0u;
    pc_ir_ore.status = ir_info.status;
    pc_ir_ore.count = PC_IR_ORE_POSITION_COUNT;
    for (uint8_t i = 0u; i < PC_IR_ORE_POSITION_COUNT; ++i) {
        pc_ir_ore.ore_type[i] = ir_info.ore_type[i];
    }
    pc_ir_ore.age_ms = ir_info.age_ms;
    pc_ir_ore.rx_count = ir_info.rx_count;

    pc_ir_ore_bridge.valid = pc_ir_ore.valid;
    pc_ir_ore_bridge.fresh = pc_ir_ore.fresh;
    pc_ir_ore_bridge.status = ir_info.status;
    pc_ir_ore_bridge.count = PC_IR_ORE_POSITION_COUNT;
    pc_ir_ore_bridge.msg_id = ir_info.msg_id;
    pc_ir_ore_bridge.side = ir_info.side;
    pc_ir_ore_bridge.ack_pending = ir_info.ack_pending;
    pc_ir_ore_bridge.parse_status = ir_info.parse_status;
    for (uint8_t i = 0u; i < PC_IR_ORE_POSITION_COUNT; ++i) {
        pc_ir_ore_bridge.ore_type[i] = ir_info.ore_type[i];
    }
    for (uint8_t i = 0u; i < PC_IR_ORE_RAW_FRAME_SIZE; ++i) {
        pc_ir_ore_bridge.raw_frame[i] = ir_info.raw_frame[i];
    }
    pc_ir_ore_bridge.age_ms = ir_info.age_ms;
    pc_ir_ore_bridge.rx_count = ir_info.rx_count;
    pc_ir_ore_bridge.frame_rx_count = ir_info.frame_rx_count;
    pc_ir_ore_bridge.ack_tx_count = ir_info.ack_tx_count;

    const bool first_send = !s_last_published_ir_ore_inited;
    const bool changed = first_send ||
        memcmp(&pc_ir_ore, &s_last_published_ir_ore, sizeof(pc_ir_ore)) != 0 ||
        memcmp(&pc_ir_ore_bridge, &s_last_published_ir_ore_bridge,
               sizeof(pc_ir_ore_bridge)) != 0;
    const bool heartbeat_due =
        (now_ms - s_last_ir_ore_publish_ms) >= PC_COMM_IR_ORE_HEARTBEAT_MS;

    if (!changed && !heartbeat_due) {
        return false;
    }

    s_last_published_ir_ore = pc_ir_ore;
    s_last_published_ir_ore_bridge = pc_ir_ore_bridge;
    s_last_published_ir_ore_inited = true;
    s_last_ir_ore_publish_ms = now_ms;
    (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_IR_ORE, &pc_ir_ore);
    (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_IR_ORE_BRIDGE,
                                   &pc_ir_ore_bridge);
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

static void PcComm_ProcessIrOreAckCommand(uint32_t now_ms) {
    const PC_IrOreAckCMD_t *cmd = MrlinkPc_GetIrOreAckCMD();
    if (cmd == NULL) {
        return;
    }

    IrDock_AckSubmitResult_t result = IrDock_SendAckFrame(cmd->frame, now_ms);
    g_pc_comm_debug.ir_ore_ack_submit_result = (uint8_t)result;
    if (result == IR_DOCK_ACK_SUBMIT_OK) {
        g_pc_comm_debug.ir_ore_ack_submit_count++;
        MrlinkPc_ClearIrOreAckCommand();
    } else if (result != IR_DOCK_ACK_SUBMIT_BUSY) {
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
    const bool ir_ore_pending = PcComm_TryUpdateIrOreFeedback(now);
    const bool start_match_pending = PcComm_AppendStartMatchFrame(&tx_len);
    if (start_match_pending) {
        frame_count++;
    }

    for (uint8_t i = 0; i < (uint8_t)(sizeof(s_feedback_cmds) / sizeof(s_feedback_cmds[0])); ++i) {
        if (PcComm_AppendFeedbackFrame(s_feedback_cmds[i], &tx_len)) {
            frame_count++;
        }
    }

    /* IR_ORE 仅在数据变更或心跳到期时才追加帧，避免 50Hz 重复刷屏。 */
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
        } else if (start_match_pending) {
            MrlinkPc_ClearStartMatchRequest();
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

    if (g_pc_command_source == PC_COMMAND_SOURCE_PC &&
        MrlinkPc_IsPCControlMode()) {
        const bool auto_action_pending = PcComm_ProcessAutoActionCommand();
        const PC_AutoStepParams_t *step_params =
            (auto_ctrl_inited && !auto_action_pending)
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

    MrlinkPc_DebugUpdate();
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

static void Task_pc_comm_legacy(void *argument) {
    (void)argument;

    while (!MrlinkPc_CommInit()) {
        PcComm_DebugRecordInitFail(g_pc_comm_debug.last_init_error);
        osDelay(20u);
    }

    while (MrlinkPc_RegisterTxCallbacks(PcComm_TxDoneCallback,
                             PcComm_TxErrorCallback) !=
            MRLINK_CHANNEL_OK) {
        PcComm_DebugRecordInitFail(-30);
        osDelay(20u);
    }

    uint32_t last_tx_tick = BSP_TIME_Get_ms();

    while (1) {
        const uint32_t profile_start_us =
            Task_ProfilerLoopBegin(TASK_PROFILE_PC_COMM,
                                   PC_COMM_LOOP_PERIOD_MS * 1000U);

        uint32_t now = BSP_TIME_Get_ms();

        MrlinkPc_CommProcess(now);
        PcComm_ProcessIrOreAckCommand(now);
        PcComm_UpdateCameraYawCommand(now);

        if ((now - last_tx_tick) >= PC_COMM_TX_PERIOD_MS) {
            (void)PcComm_TransmitFeedback();
            last_tx_tick = now;
        }

        if (g_pc_command_source == PC_COMMAND_SOURCE_PC &&
            MrlinkPc_IsPCControlMode()) {
            const bool auto_action_pending = PcComm_ProcessAutoActionCommand();
            const PC_AutoStepParams_t *step_params =
                (auto_ctrl_inited && !auto_action_pending)
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

        MrlinkPc_DebugUpdate();
        task_runtime.heartbeat.pc_comm++;
        Task_ProfilerLoopEnd(TASK_PROFILE_PC_COMM, profile_start_us);
        osDelay(PC_COMM_LOOP_PERIOD_MS);
    }
}
