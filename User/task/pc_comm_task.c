/*
 * PC_COMM Task - 上位机通信任务
 */
#include <stddef.h>

#include "task/user_task.h"
#include "module/mrlink_pc_comm/mrlink_pc_comm.h"
#include "module/autoCtrlAPI/api/auto_ctrl_api.h"
#include "module/arm_simple.h"
#include "module/rod_new.h"
#include "bsp/uart.h"
#include "main.h"

#define PC_COMM_TX_PERIOD_MS (20u)  /* 50Hz */
#define PC_COMM_LOOP_PERIOD_MS (1u)
#define PC_COMM_TX_DMA_TIMEOUT_MS (100u)

extern volatile PC_CommandSource_t g_pc_command_source;

static uint8_t s_tx_buf[256];
static volatile bool s_tx_dma_busy = false;
static volatile uint32_t s_tx_dma_start_tick = 0;
static const uint8_t s_feedback_cmds[] = {
    PC_FEEDBACK_HEARTBEAT,
    PC_FEEDBACK_CHASSIS,
    PC_FEEDBACK_POLE,
    PC_FEEDBACK_ARM_SIMPLE,
    PC_FEEDBACK_ROD_NEW,
    PC_FEEDBACK_ORE_STORE,
    PC_FEEDBACK_STEP,
    PC_FEEDBACK_STATUS,
};

static uint16_t PcComm_DebugCopyRaw(volatile uint8_t *dst, uint16_t dst_size,
                                    const uint8_t *src, uint16_t src_len) {
    uint16_t copy_len = src_len;
    if (copy_len > dst_size) {
        copy_len = dst_size;
    }
    for (uint16_t i = 0; i < copy_len; ++i) {
        dst[i] = src[i];
    }
    for (uint16_t i = copy_len; i < dst_size; ++i) {
        dst[i] = 0;
    }
    return copy_len;
}

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
    g_pc_comm_debug.tx_raw_len = PcComm_DebugCopyRaw(
        g_pc_comm_debug.tx_raw, PC_COMM_DEBUG_TX_RAW_SIZE, data, len);
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

static bool PcComm_AppendFeedbackFrame(uint8_t cmd, uint16_t *tx_len) {
    if (tx_len == NULL || *tx_len >= sizeof(s_tx_buf)) {
        return false;
    }

    uint16_t frame_len = MrlinkPc_BuildFeedbackFrame(
        cmd, &s_tx_buf[*tx_len], (uint16_t)(sizeof(s_tx_buf) - *tx_len));
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
        pc_arm.joint1_temperature_warning =
            arm_fb->joint1_temperature_warning ? 1u : 0u;
        pc_arm.joint1_temperature_over_limit =
            arm_fb->joint1_temperature_over_limit ? 1u : 0u;
        pc_arm.joint1_temperature_limit_latched =
            arm_fb->joint1_temperature_limit_latched ? 1u : 0u;
        pc_arm.joint1_angle_rad = arm_fb->joint1_angle_rad;
        pc_arm.joint1_velocity_rad_s = arm_fb->joint1_velocity_rad_s;
        pc_arm.joint1_temperature_c = arm_fb->joint1_temperature_c;
        pc_arm.joint2_angle_rad = arm_fb->joint2_angle_rad;
        pc_arm.target_joint1_rad = arm_fb->target_joint1_rad;
        pc_arm.target_joint2_rad = arm_fb->target_joint2_rad;
        MrlinkPc_SetArmSimpleFeedback(&pc_arm);
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
        MrlinkPc_SetRodNewFeedback(&pc_rod);
    }

    const OreStore_Feedback_t *ore_fb = Task_OreStoreGetFeedback();
    if (ore_fb != NULL) {
        const PC_OreStoreCMD_t *ore_cmd = MrlinkPc_GetOreStoreCMD();
        PC_OreStoreFeedback_t pc_ore = {0};
        pc_ore.mode = (ore_cmd != NULL) ? ore_cmd->mode : 0u;
        pc_ore.all_homed = ore_fb->all_homed ? 1u : 0u;
        for (uint8_t axis = 0u; axis < ORE_STORE_AXIS_NUM; ++axis) {
            if (ore_fb->online[axis]) {
                pc_ore.online_mask |= (uint8_t)(1u << axis);
            }
            if (ore_fb->homed[axis]) {
                pc_ore.homed_mask |= (uint8_t)(1u << axis);
            }
        }
        pc_ore.platform_position_rad = ore_fb->position_rad[ORE_STORE_AXIS_PLATFORM];
        pc_ore.gate_position_rad[0] = 0.0f;
        pc_ore.gate_position_rad[1] = 0.0f;
        pc_ore.track_position_rad[0] = 0.0f;
        pc_ore.track_position_rad[1] = 0.0f;
        MrlinkPc_SetOreStoreFeedback(&pc_ore);
    }
}

static void PcComm_UpdateStatusFeedback(void) {
    const MrlinkPc_State_t *state = MrlinkPc_GetState();
    PC_StatusFeedback_t status = {0};
    status.online = MrlinkPc_CommIsOnline() ? 1u : 0u;
    status.recv_count = (state != NULL) ? state->recv_count : 0u;
    status.cpu_temp = task_runtime.status.cpu_temp;
    status.command_source = g_pc_command_source;
    MrlinkPc_SetStatusFeedback(&status);
}

static bool PcComm_TransmitFeedback(void) {
    uint16_t tx_len = 0;
    uint8_t frame_count = 0;
    uint32_t now = osKernelGetTickCount();

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

    for (uint8_t i = 0; i < (uint8_t)(sizeof(s_feedback_cmds) / sizeof(s_feedback_cmds[0])); ++i) {
        if (PcComm_AppendFeedbackFrame(s_feedback_cmds[i], &tx_len)) {
            frame_count++;
        }
    }

    if (tx_len > 0u) {
        s_tx_dma_busy = true;
        s_tx_dma_start_tick = now;
        int8_t tx_result = BSP_UART_Transmit(BSP_UART_PC, s_tx_buf, tx_len, true);
        if (tx_result != BSP_OK) {
            s_tx_dma_busy = false;
            g_pc_comm_debug.tx_dma_error_count++;
        }
        PcComm_DebugRecordTx(s_tx_buf, tx_len, frame_count, tx_result);
        return tx_result == BSP_OK;
    }

    PcComm_DebugRecordTx(s_tx_buf, 0, frame_count, BSP_ERR);
    return false;
}

void Task_pc_comm(void *argument) {
    (void)argument;

    while (!MrlinkPc_CommInit()) {
        PcComm_DebugRecordInitFail(g_pc_comm_debug.last_init_error);
        osDelay(20u);
    }

    while (BSP_UART_RegisterCallback(BSP_UART_PC, BSP_UART_TX_CPLT_CB,
                                     PcComm_TxDoneCallback) != BSP_OK ||
           BSP_UART_RegisterCallback(BSP_UART_PC, BSP_UART_ERROR_CB,
                                     PcComm_TxErrorCallback) != BSP_OK ||
           BSP_UART_RegisterCallback(BSP_UART_PC, BSP_UART_ABORT_TX_CPLT_CB,
                                     PcComm_TxErrorCallback) != BSP_OK) {
        PcComm_DebugRecordInitFail(-30);
        osDelay(20u);
    }

    uint32_t last_tx_tick = osKernelGetTickCount();

    while (1) {
        task_runtime.stack_water_mark.pc_comm = uxTaskGetStackHighWaterMark(NULL);

        uint32_t now = osKernelGetTickCount();

        MrlinkPc_CommProcess(now);

        if ((now - last_tx_tick) >= PC_COMM_TX_PERIOD_MS) {
            (void)PcComm_TransmitFeedback();
            last_tx_tick = now;
        }

        if (auto_ctrl_inited &&
            g_pc_command_source == PC_COMMAND_SOURCE_PC &&
            MrlinkPc_IsPCControlMode()) {
            const PC_AutoStepParams_t *step_params = MrlinkPc_GetAutoStepParams();
            if (step_params != NULL && !AutoCtrl_IsBusy(&auto_ctrl)) {
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
        osDelay(PC_COMM_LOOP_PERIOD_MS);
    }
}
