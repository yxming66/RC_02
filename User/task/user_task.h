#pragma once

/* Includes ----------------------------------------------------------------- */
#include <cmsis_os2.h>
#include "FreeRTOS.h"
#include "task.h"

/* USER INCLUDE BEGIN */
#include <stdbool.h>
#include <stdint.h>
#include "bsp/time.h"
#include "module/autoCtrlAPI/api/auto_ctrl_api.h"
#include "module/autoCtrlAPI/ore_store/auto_ore_store.h"
#include "module/autoCtrlAPI/rod/auto_rod_spearhead.h"
#include "module/autoCtrlAPI/sick/auto_sick_correct.h"
#include "device/ir_dock/ir_dock.h"
#include "module/arm_simple.h"
#include "module/rod_new.h"
#include "module/ore_store.h"
#include "module/camera_yaw.h"
#include "device/buzzer.h"
#include "device/sick.h"

/* USER INCLUDE END */

#ifdef __cplusplus
extern "C" {
#endif
/* Exported constants ------------------------------------------------------- */
/* 任务运行频率 */
#define BLINK_FREQ (100.0)
#define ATTI_ESTI_FREQ (100.0)
#define CHASSIS_MAIN_FREQ (200.0)
#define POLE_MAIN_FREQ (1000.0)
#define RC_MAIN_FREQ (500.0)
#define SICK_FREQ (100.0)
#define AUTO_CTRL_FREQ (100.0)
#define CAMERA_YAW_FREQ (200.0)
#define ARM_SIMPLE_FREQ (200.0)
#define ROD_FREQ (200.0)
#define PC_COMM_FREQ (100.0)
#define ORE_STORE_FREQ (200.0)
#define IR_DOCK_FREQ (4.0)
#define TASK_FUSED_LOOP_FREQ (200.0)
/* 任务初始化延时ms */
#define TASK_INIT_DELAY (100u)
#define BLINK_INIT_DELAY (0)
#define ATTI_ESTI_INIT_DELAY (0)
#define CHASSIS_MAIN_INIT_DELAY (0) 
#define POLE_MAIN_INIT_DELAY (1u)
#define RC_MAIN_INIT_DELAY (0)
#define SICK_INIT_DELAY (0)
#define AUTO_CTRL_INIT_DELAY (0)
#define CAMERA_YAW_INIT_DELAY (0)
#define ARM_SIMPLE_INIT_DELAY (100u)
#define ROD_INIT_DELAY (0)
#define PC_COMM_INIT_DELAY (500u)
#define ORE_STORE_INIT_DELAY (100u)
#define IR_DOCK_INIT_DELAY (0)
/* Exported defines --------------------------------------------------------- */
#define TASK_PERIOD_US(freq) ((uint32_t)(1000000.0f / (float)(freq)))
/* Exported macro ----------------------------------------------------------- */
/* Exported types ----------------------------------------------------------- */
typedef enum {
    TASK_PROFILE_BLINK = 0,
    TASK_PROFILE_ATTI_ESTI,
    TASK_PROFILE_CHASSIS_ORE,
    TASK_PROFILE_CHASSIS_MAIN,
    TASK_PROFILE_ORE_STORE,
    TASK_PROFILE_UPPER_MECH,
    TASK_PROFILE_CAMERA_YAW,
    TASK_PROFILE_ARM_SIMPLE,
    TASK_PROFILE_ROD,
    TASK_PROFILE_POLE_MAIN,
    TASK_PROFILE_RC_MAIN,
    TASK_PROFILE_PC_COMM_SICK,
    TASK_PROFILE_PC_COMM,
    TASK_PROFILE_SICK,
    TASK_PROFILE_AUTO_CTRL,
    TASK_PROFILE_IR_DOCK,
    TASK_PROFILE_COUNT,
} Task_ProfileId_t;

typedef struct {
    volatile uint32_t target_period_us;
    volatile uint32_t last_start_us;
    volatile uint32_t period_us;
    volatile uint32_t min_period_us;
    volatile uint32_t max_period_us;
    volatile uint32_t exec_us;
    volatile uint32_t max_exec_us;
    volatile uint32_t overrun_count;
    volatile uint32_t long_period_count;
    volatile uint32_t deadline_miss_count;
    volatile uint32_t resync_count;
    volatile uint32_t late_tick;
    volatile uint32_t max_late_tick;
    volatile uint32_t loop_count;
} Task_ProfileStats_t;

typedef struct {
    uint32_t period_us;
    uint32_t next_due_us;
    bool initialized;
} Task_SubtaskTimer_t;

typedef enum {
    BUZZER_ALARM_NONE = 0,
    BUZZER_ALARM_TEMP_WARNING,
    BUZZER_ALARM_TEMP_OVER_LIMIT,
} Buzzer_AlarmLevel_t;

typedef struct {
    volatile Buzzer_AlarmLevel_t level;
    volatile uint32_t request_tick;
    volatile uint32_t min_duration_ms;
} Buzzer_AlarmRequest_t;

typedef enum {
    AUTO_ORE_DEBUG_REQUEST_NONE = 0,
    AUTO_ORE_DEBUG_REQUEST_STORE = 1,
    AUTO_ORE_DEBUG_REQUEST_RELEASE = 2,
    AUTO_ORE_DEBUG_REQUEST_CHAMBER = 3,
    AUTO_ORE_DEBUG_REQUEST_ABORT = 4,
    AUTO_ORE_DEBUG_REQUEST_PICK_POS_400 = 5,
    AUTO_ORE_DEBUG_REQUEST_PICK_POS_200 = 6,
    AUTO_ORE_DEBUG_REQUEST_PICK_NEG_200 = 7,
    AUTO_ORE_DEBUG_REQUEST_ROD_SPEARHEAD = 8,
    AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ROD_SPEARHEAD = 9,
    AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ORE_RELEASE = 10,
    AUTO_ORE_DEBUG_REQUEST_ROD_DOCK_WAIT = 11,
    AUTO_ORE_DEBUG_REQUEST_STEP_PICK_STORE_ASCEND_200_HEAD = 12,
    AUTO_ORE_DEBUG_REQUEST_STEP_PICK_STORE_DESCEND_200_HEAD = 13,
    AUTO_ORE_DEBUG_REQUEST_STEP_PICK_STORE_ASCEND_400_HEAD = 14,
    AUTO_ORE_DEBUG_REQUEST_ROD_SPEARHEAD_STEP1 = 15,
    AUTO_ORE_DEBUG_REQUEST_STEP_ASCEND_200_HEAD = 16,
    AUTO_ORE_DEBUG_REQUEST_STEP_DESCEND_200_HEAD = 17,
    AUTO_ORE_DEBUG_REQUEST_STEP_ASCEND_400_HEAD = 18,
    AUTO_ORE_DEBUG_REQUEST_STEP_DESCEND_400_HEAD = 19,
    AUTO_ORE_DEBUG_REQUEST_ROD_SPEARHEAD_STEP2 = 20,
} AutoOre_DebugRequest_t;

typedef struct {
    volatile AutoOre_DebugRequest_t request;
    volatile AutoOre_DebugRequest_t last_request;
    volatile bool last_result;
    volatile uint32_t request_count;
    volatile uint32_t accept_count;
    volatile bool force_output_enable;
    volatile uint32_t force_output_count;
    volatile bool busy;
    volatile AutoOre_State_t state;
    volatile AutoOre_Result_t result;
    volatile AutoOre_Fault_t fault;
    volatile AutoOre_Action_t action;
    volatile AutoOre_Position_t active_position;
    volatile AutoOre_Position_t last_active_position;
    volatile uint8_t step_index;
    volatile uint8_t step_phase;
    volatile uint8_t occupancy_mask;
    volatile bool transform_low_has_ore;
    volatile bool transform_high_has_ore;
    volatile bool arm_has_ore;
    volatile bool checkphoto_spear_triggered;
    volatile bool checkphoto_orelow_triggered;
    volatile bool checkphoto_orehigh_triggered;
    volatile bool photo_transfer_valid;
    volatile uint16_t photo_transfer_raw_mask;
    volatile uint32_t photo_transfer_age_ms;
    volatile uint32_t photo_transfer_rx_count;
    volatile uint32_t photo_transfer_timeout_count;
    volatile bool arm_cmd_valid;
    volatile bool ore_store_cmd_valid;
    volatile bool pole_cmd_valid;
    volatile bool chassis_cmd_valid;
    volatile bool pick_lift_confirmed;
    volatile float fused_wheel_delta_rad;
    volatile float fused_target_wheel_delta_rad;
    volatile bool fused_step_done;
    volatile bool fused_store_done;
    volatile bool arm_at_target;
    volatile bool ore_store_all_at_target;
    volatile bool pole_all_at_target;
    volatile float arm_feedback_joint1_rad;
    volatile float arm_feedback_joint2_rad;
    volatile float arm_feedback_target_joint1_rad;
    volatile float arm_feedback_target_joint2_rad;
    volatile float arm_feedback_output_target_joint1_rad;
    volatile float arm_feedback_output_target_joint2_rad;
    volatile float arm_joint1_error_rad;
    volatile float arm_joint2_error_rad;
    volatile float arm_output_joint1_error_rad;
    volatile float arm_output_joint2_error_rad;
    volatile float arm_cmd_joint1_rad;
    volatile float arm_cmd_joint2_rad;
    volatile float arm_cmd_joint1_max_vel_rad_s;
    volatile float arm_cmd_joint2_max_vel_rad_s;
    volatile float ore_store_cmd_platform_rad;
    volatile float ore_store_feedback_platform_rad;
    volatile float ore_store_platform_error_rad;
    volatile float pole_cmd_front_lift_rad;
    volatile float pole_cmd_rear_lift_rad;
    volatile float chassis_cmd_vx_mps;
    volatile bool auto_rod_spearhead_busy;
    volatile AutoRodSpearhead_State_t auto_rod_spearhead_state;
    volatile AutoRodSpearhead_Result_t auto_rod_spearhead_result;
    volatile AutoRodSpearhead_Fault_t auto_rod_spearhead_fault;
    volatile AutoRodSpearhead_Action_t auto_rod_spearhead_action;
    volatile uint8_t auto_rod_spearhead_step_index;
    volatile bool auto_rod_spearhead_rod_at_target;
    volatile bool auto_rod_spearhead_ore_store_at_target;
    volatile bool auto_rod_spearhead_photo_stable_state;
    volatile bool auto_rod_spearhead_rod_cmd_valid;
    volatile RodNew_Pose_t auto_rod_spearhead_rod_cmd_pose;
    volatile RodNew_GripState_t auto_rod_spearhead_rod_cmd_grip;
    volatile float auto_rod_spearhead_rod_cmd_target_angle_rad;
    volatile bool auto_rod_spearhead_ore_store_cmd_valid;
    volatile float auto_rod_spearhead_ore_store_cmd_platform_rad;
    volatile bool auto_sick_correct_busy;
    volatile AutoSickCorrect_State_t auto_sick_correct_state;
    volatile AutoSickCorrect_Result_t auto_sick_correct_result;
    volatile AutoSickCorrect_Fault_t auto_sick_correct_fault;
    volatile AutoSickCorrect_Action_t auto_sick_correct_action;
    volatile uint8_t auto_sick_correct_step_index;
    volatile bool auto_sick_correct_pole_all_at_target;
    volatile float auto_sick_correct_x_sample_adc;
    volatile float auto_sick_correct_y_sample_adc;
    volatile float auto_sick_correct_yaw_sample_diff_adc;
    volatile float auto_sick_correct_x_error_adc;
    volatile float auto_sick_correct_y_error_adc;
    volatile float auto_sick_correct_yaw_error_adc;
    volatile float auto_sick_correct_vx_mps;
    volatile float auto_sick_correct_vy_mps;
    volatile float auto_sick_correct_wz_rad_s;
    volatile float auto_sick_correct_pole_target_lift;
    volatile uint8_t auto_sick_correct_x_sample_index;
    volatile float auto_sick_correct_x_target_adc;
    volatile float auto_sick_correct_y_target_adc;
    volatile float auto_sick_correct_z_target_diff_adc;
    volatile float auto_sick_correct_x_kp_mps_per_adc;
    volatile float auto_sick_correct_y_kp_mps_per_adc;
    volatile float auto_sick_correct_z_kp_rad_s_per_adc;
    volatile bool auto_sick_correct_param_override_enable;
    volatile float auto_sick_correct_override_x_target_adc;
    volatile float auto_sick_correct_override_y_target_adc;
    volatile float auto_sick_correct_override_z_target_diff_adc;
    volatile float auto_sick_correct_override_x_kp_mps_per_adc;
    volatile float auto_sick_correct_override_y_kp_mps_per_adc;
    volatile float auto_sick_correct_override_z_kp_rad_s_per_adc;
    volatile bool ir_dock_complete_fresh;
    volatile uint8_t ir_dock_last_rx_status;
    volatile uint32_t ir_dock_last_rx_age_ms;
    volatile uint32_t ir_dock_rx_count;
    volatile uint32_t ir_dock_error_count;
} AutoOre_DebugControl_t;

typedef struct {
    volatile uint32_t can_bus;
    volatile uint32_t can_id;
    volatile uint16_t adc_raw[SICK_OUTPUT_CHANNEL_COUNT];
    volatile float distance_mm[SICK_OUTPUT_CHANNEL_COUNT];
    volatile float distance_m[SICK_OUTPUT_CHANNEL_COUNT];
    volatile bool valid[SICK_OUTPUT_CHANNEL_COUNT];
    volatile uint16_t miss_count;
    volatile uint32_t update_tick;
    volatile uint32_t read_count;
    volatile uint32_t read_fail_count;
} Sick_Debug_t;

typedef struct {
    volatile bool override_enable;
    volatile bool load_from_config_once;
    volatile bool restore_defaults_once;
    volatile bool initialized;
    volatile uint32_t apply_count;
    volatile uint32_t load_count;
    volatile uint32_t restore_count;
    volatile float pos_k;
    volatile float pos_p;
    volatile float pos_i;
    volatile float pos_d;
    volatile float pos_i_limit;
    volatile float pos_out_limit;
    volatile float pos_d_cutoff_freq;
    volatile float pos_range;
    volatile float vel_k;
    volatile float vel_p;
    volatile float vel_i;
    volatile float vel_d;
    volatile float vel_i_limit;
    volatile float vel_out_limit;
    volatile float vel_d_cutoff_freq;
    volatile float vel_range;
} PolePidDebugControl_t;

/* 任务运行时结构体 */
typedef struct {
    /* 各任务，也可以叫做线程 */
    struct {
        osThreadId_t blink;
        osThreadId_t atti_esti;
        osThreadId_t chassis_ore;
        osThreadId_t upper_mech;
        osThreadId_t pole_main;
        osThreadId_t rc_main;
        osThreadId_t auto_ctrl;
        osThreadId_t pc_comm_sick;
        osThreadId_t ir_dock;

    } thread;

    /* USER MESSAGE BEGIN */
    struct {
        struct {
            osMessageQueueId_t imu;
            osMessageQueueId_t cmd;
            osMessageQueueId_t yaw;
        } chassis;
        struct {
            osMessageQueueId_t cmd;
        } pole;
        struct {
            osMessageQueueId_t cmd;
        } arm_simple;
        struct {
            osMessageQueueId_t cmd;
        } rod;
        struct {
            osMessageQueueId_t cmd;
        } camera_yaw;
        struct {
            osMessageQueueId_t cmd;
        } ore_store;
    } msgq;
    /* USER MESSAGE END */

    /* 机器人状态 */
    struct {
        float battery; /* 电池电量百分比 */
        float vbat; /* 电池电压 */
        float cpu_temp; /* CPU温度 */
        struct {
            Buzzer_AlarmLevel_t level;
            bool axis_temperature_warning[ORE_STORE_AXIS_NUM];
            bool axis_temperature_over_limit[ORE_STORE_AXIS_NUM];
            float max_temperature_c;
        } ore_store_alarm;
        struct {
            Buzzer_AlarmLevel_t level;
            bool motor_temperature_warning[POLE_MOTOR_NUM];
            bool motor_temperature_over_limit[POLE_MOTOR_NUM];
            float max_temperature_c;
        } pole_alarm;
    } status;

    /* USER CONFIG BEGIN */

    /* USER CONFIG END */

    /* Heartbeat counters for debugger/watch-window task liveness checks. */
    struct {
        volatile uint32_t init;
        volatile uint32_t blink;
        volatile uint32_t atti_esti;
        volatile uint32_t chassis_ore;
        volatile uint32_t chassis_main;
        volatile uint32_t upper_mech;
        volatile uint32_t camera_yaw;
        volatile uint32_t pole_main;
        volatile uint32_t rc_main;
        volatile uint32_t pc_comm_sick;
        volatile uint32_t sick;
        volatile uint32_t auto_ctrl;
        volatile uint32_t arm_simple;
        volatile uint32_t rod;
        volatile uint32_t pc_comm;
        volatile uint32_t ore_store;
        volatile uint32_t ir_dock;
    } heartbeat;

    Task_ProfileStats_t profile[TASK_PROFILE_COUNT];

} Task_Runtime_t;

/* 任务运行时结构体 */
extern Task_Runtime_t task_runtime;

extern auto_ctrl_t auto_ctrl;
extern bool auto_ctrl_inited;
extern AutoOre_t auto_ore_ctrl;
extern bool auto_ore_inited;
extern volatile AutoOre_DebugControl_t g_auto_ore_debug;
extern volatile Sick_Debug_t g_sick_debug;
extern volatile PolePidDebugControl_t g_pole_pid_debug;
extern volatile IrDock_Debug_t g_ir_dock_debug;
extern AutoRodSpearhead_t auto_rod_spearhead_ctrl;
extern bool auto_rod_spearhead_inited;
extern AutoSickCorrect_t auto_sick_correct_ctrl;
extern bool auto_sick_correct_inited;
extern bool auto_ctrl_local_yaw_zero_initialized;
extern float auto_ctrl_local_yaw_zero_rad;
extern BUZZER_t buzzer;
extern bool g_buzzer_calib_active;
extern Buzzer_AlarmRequest_t g_buzzer_alarm_request;

/* 初始化任务句柄 */
extern const osThreadAttr_t attr_init;
extern const osThreadAttr_t attr_blink;
extern const osThreadAttr_t attr_atti_esti;
extern const osThreadAttr_t attr_chassis_ore;
extern const osThreadAttr_t attr_upper_mech;
extern const osThreadAttr_t attr_pole_main;
extern const osThreadAttr_t attr_rc_main;
extern const osThreadAttr_t attr_auto_ctrl;
extern const osThreadAttr_t attr_pc_comm_sick;
extern const osThreadAttr_t attr_ir_dock;
/* 任务函数声明 */
void Task_Init(void *argument);
void Task_blink(void *argument);
void Task_atti_esti(void *argument);
void Task_chassis_ore(void *argument);
void Task_chassis_main(void *argument);
void Task_upper_mech(void *argument);
void Task_pole_main(void *argument);
void Task_rc_main(void *argument);
void Task_sick(void *argument);
void Task_auto_ctrl(void *argument);
void Task_arm_simple(void *argument);
void Task_rod(void *argument);
void Task_pc_comm(void *argument);
void Task_pc_comm_sick(void *argument);
void Task_ore_store(void *argument);
void Task_ir_dock(void *argument);
uint32_t Task_ProfilerLoopBegin(Task_ProfileId_t id, uint32_t target_period_us);
void Task_ProfilerLoopEnd(Task_ProfileId_t id, uint32_t loop_start_us);
void Task_DelayUntil(Task_ProfileId_t id, uint32_t *wake_tick,
                     uint32_t delay_tick);
void Task_SubtaskTimerInit(Task_SubtaskTimer_t *timer, float frequency_hz,
                           uint32_t now_us);
bool Task_SubtaskTimerDue(Task_SubtaskTimer_t *timer, uint32_t now_us);
bool Task_ChassisMainInitOnce(void);
void Task_ChassisMainStep(void);
bool Task_OreStoreInitOnce(void);
void Task_OreStoreStep(void);
bool Task_CameraYawInitOnce(void);
void Task_CameraYawStep(void);
bool Task_ArmSimpleInitOnce(void);
void Task_ArmSimpleStep(void);
bool Task_RodNewInitOnce(void);
void Task_RodNewStep(void);
bool Task_PcCommInitOnce(void);
void Task_PcCommStep(void);
bool Task_SickInitOnce(void);
void Task_SickStep(void);

const Chassis_Feedback_t *Task_ChassisGetFeedback(void);
bool Task_PoleMainGroupAtTarget(uint8_t group, float threshold_rad);
bool Task_PoleMainAllAtTarget(float threshold_rad);
bool Task_PoleMainGetSupportLift(float *front_lift_rad, float *rear_lift_rad);
bool Task_PoleMainGetHoldCommand(Pole_CMD_t *cmd);
bool Task_AutoOreStartStore(void);
bool Task_AutoOreStartRelease(void);
bool Task_AutoOreStartChamber(void);
bool Task_AutoOreStartPickPos400(void);
bool Task_AutoOreStartPickPos200(void);
bool Task_AutoOreStartPickNeg200(void);
bool Task_AutoOreStartStepPickStoreAscend200Head(void);
bool Task_AutoOreStartStepPickStoreDescend200Head(void);
bool Task_AutoOreStartStepPickStoreAscend400Head(void);
bool Task_AutoStepStartAscend200Head(void);
bool Task_AutoStepStartDescend200Head(void);
bool Task_AutoStepStartAscend400Head(void);
bool Task_AutoStepStartDescend400Head(void);
void Task_AutoOreAbort(void);
bool Task_AutoRodSpearheadStart(void);
bool Task_AutoRodSpearheadStartStep1(void);
bool Task_AutoRodSpearheadStartStep2(void);
bool Task_AutoRodSpearheadStartDockWait(void);
void Task_AutoRodSpearheadAbort(void);
bool Task_AutoRodSpearheadIsBusy(void);
bool Task_AutoRodSpearheadIsPickupStep1(void);
const RodNew_CMD_t *Task_AutoRodSpearheadGetCommand(void);
const OreStore_CMD_t *Task_AutoRodSpearheadGetOreStoreCommand(void);
bool Task_AutoSickCorrectStartRodSpearhead(void);
bool Task_AutoSickCorrectStartOreRelease(void);
void Task_AutoSickCorrectAbort(void);
bool Task_AutoSickCorrectIsBusy(void);
const Chassis_CMD_t *Task_AutoSickCorrectGetChassisCommand(void);
const Pole_CMD_t *Task_AutoSickCorrectGetPoleCommand(void);
bool Task_IrDockIsDockCompleteFresh(void);
int8_t Task_OreStorePostCommand(const OreStore_CMD_t *cmd);
void Task_OreStoreRequestRehome(void);
int8_t Task_OreStoreAssumeAxisHomedAtCurrent(uint8_t axis,
                                             float position_rad);
bool Task_OreStoreIsAxisHomed(uint8_t axis);
bool Task_OreStoreIsAllHomed(void);
bool Task_OreStorePowerOnHomeInProgress(void);
bool Task_OreStorePowerOnHomeAttemptDone(void);
bool Task_OreStorePowerOnHomeFailed(void);
bool Task_OreStoreIsAxisAtTarget(uint8_t axis, float threshold_rad);
bool Task_OreStoreIsAllAtTarget(float threshold_rad);
bool Task_SickGetLatestOutput(Sick_Output_t *output);
const ArmSimple_Feedback_t *Task_ArmSimpleGetFeedback(void);
const RodNew_Feedback_t *Task_RodNewGetFeedback(void);
const CameraYaw_Feedback_t *Task_CameraYawGetFeedback(void);
const CameraYaw_GroupFeedback_t *Task_CameraYawGetGroupFeedback(void);
int8_t Task_CameraYawPostCommand(const CameraYaw_CMD_t *cmd);
int8_t Task_CameraYawPostGroupCommand(const CameraYaw_GroupCMD_t *cmd);
const OreStore_Feedback_t *Task_OreStoreGetFeedback(void);
const OreStore_Debug_t *Task_OreStoreGetDebug(void);
#ifdef __cplusplus
}
#endif
