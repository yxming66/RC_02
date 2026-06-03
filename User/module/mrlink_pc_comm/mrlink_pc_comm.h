#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "module/autoCtrlAPI/core/auto_ctrl_def.h"
#include "mrlink/mrlink.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PC_MODE_RC = 0,
    PC_MODE_PC = 1,
} PC_ControlMode_t;

typedef enum {
    PC_COMMAND_SOURCE_RC = 0,
    PC_COMMAND_SOURCE_PC = 1,
} PC_CommandSource_t;

typedef enum {
    PC_CMD_HEARTBEAT = 0x01,
    PC_CMD_CHASSIS = 0x10,
    PC_CMD_POLE = 0x11,
    PC_CMD_STEP = 0x12,
    PC_CMD_ARM_SIMPLE = 0x13,
    PC_CMD_ROD_NEW = 0x14,
    PC_CMD_ORE_STORE = 0x15,
    PC_CMD_IMU = 0x20,
} PC_CMD_t;

typedef enum {
    PC_FEEDBACK_HEARTBEAT = 0x81,
    PC_FEEDBACK_CHASSIS = 0x90,
    PC_FEEDBACK_POLE = 0x91,
    PC_FEEDBACK_STEP = 0x92,
    PC_FEEDBACK_ARM_SIMPLE = 0x93,
    PC_FEEDBACK_ROD_NEW = 0x94,
    PC_FEEDBACK_ORE_STORE = 0x95,
    PC_FEEDBACK_STATUS = 0xA0,
} PC_FeedbackCMD_t;

#define MRLINK_PC_MAX_PAYLOAD_SIZE (64u)
#define MRLINK_PC_MAX_FRAME_SIZE (2u + 1u + 1u + MRLINK_PC_MAX_PAYLOAD_SIZE + 2u)
#define PC_COMM_DEBUG_RX_RAW_SIZE (256u)
#define PC_COMM_DEBUG_TX_RAW_SIZE (96u)

typedef struct {
    float vx;
    float vy;
    float wz;
} PC_ChassisCMD_t;

typedef struct {
    uint8_t mode;
    float lift[2];
} PC_PoleCMD_t;

typedef struct {
    float y;
    float z;
    float pitch;
} PC_ArmCMD_t;

typedef struct {
    uint8_t mode;
    uint8_t point_mode;
    uint8_t suction;
    float target_joint1_rad;
    float target_joint2_rad;
} PC_ArmSimpleCMD_t;

typedef struct {
    uint8_t mode;
    uint8_t pose;
    uint8_t grip;
    float target_angle_rad;
} PC_RodNewCMD_t;

typedef struct {
    uint8_t mode;
    uint8_t force_rehome;
    float platform_target_rad;
    float gate_target_rad[2];
    float track_target_rad[2];
} PC_OreStoreCMD_t;

typedef enum {
    PC_STEP_TEMPLATE_NONE = 0,
    PC_STEP_TEMPLATE_ASCEND_200_HEAD = 1,
    PC_STEP_TEMPLATE_ASCEND_400_HEAD = 2,
    PC_STEP_TEMPLATE_DESCEND_200_HEAD = 3,
    PC_STEP_TEMPLATE_DESCEND_400_HEAD = 4,
    PC_STEP_TEMPLATE_ASCEND_200_TAIL = 5,
    PC_STEP_TEMPLATE_ASCEND_400_TAIL = 6,
    PC_STEP_TEMPLATE_DESCEND_200_TAIL = 7,
    PC_STEP_TEMPLATE_DESCEND_400_TAIL = 8,
} PC_StepTemplate_t;

typedef enum {
    PC_STEP_DIR_HEAD_FORWARD = 0,
    PC_STEP_DIR_TAIL_FORWARD = 1,
} PC_StepDir_t;

typedef struct {
    PC_StepTemplate_t template_id;
    PC_StepDir_t travel_dir;
    float target_yaw_rad;
    float yaw_tolerance_rad;
} PC_StepCMD_t;

typedef struct {
    float qw;
    float qx;
    float qy;
    float qz;
    float roll;
    float pitch;
    float yaw;
} PC_ImuCMD_t;

typedef struct {
    float vx;
    float vy;
    float wz;
} PC_ChassisFeedback_t;

typedef struct {
    float lift[2];
    float motor_total_angle[4];
} PC_PoleFeedback_t;

typedef struct {
    float y;
    float z;
    float pitch;
} PC_ArmFeedback_t;

typedef struct {
    uint8_t mode;
    uint8_t point_mode;
    uint8_t suction;
    uint8_t joint1_temperature_warning;
    uint8_t joint1_temperature_over_limit;
    uint8_t joint1_temperature_limit_latched;
    float joint1_angle_rad;
    float joint1_velocity_rad_s;
    float joint1_temperature_c;
    float joint2_angle_rad;
    float target_joint1_rad;
    float target_joint2_rad;
} PC_ArmSimpleFeedback_t;

typedef struct {
    uint8_t mode;
    uint8_t pose;
    uint8_t grip;
    uint8_t at_target;
    float target_angle_rad;
    float tracked_angle_rad;
    float tracked_velocity_rad_s;
    float feedback_angle_rad;
} PC_RodNewFeedback_t;

typedef struct {
    uint8_t mode;
    uint8_t all_homed;
    uint8_t online_mask;
    uint8_t homed_mask;
    float platform_position_rad;
    float gate_position_rad[2];
    float track_position_rad[2];
} PC_OreStoreFeedback_t;

typedef enum {
    PC_STEP_STATE_IDLE = 0,
    PC_STEP_STATE_PREALIGN = 1,
    PC_STEP_STATE_RUNNING = 2,
    PC_STEP_STATE_SUCCESS = 3,
    PC_STEP_STATE_FAIL = 4,
    PC_STEP_STATE_ABORT = 5,
} PC_StepState_t;

typedef enum {
    PC_STEP_RESULT_NONE = 0,
    PC_STEP_RESULT_RUNNING = 1,
    PC_STEP_RESULT_SUCCESS = 2,
    PC_STEP_RESULT_FAIL = 3,
    PC_STEP_RESULT_ABORTED = 4,
} PC_StepResult_t;

typedef enum {
    PC_STEP_FAULT_NONE = 0,
    PC_STEP_FAULT_INVALID_TEMPLATE = 1,
    PC_STEP_FAULT_PREALIGN_TIMEOUT = 2,
    PC_STEP_FAULT_TEMPLATE_TIMEOUT = 3,
    PC_STEP_FAULT_SENSOR_INVALID = 4,
    PC_STEP_FAULT_UNSUPPORTED = 5,
    PC_STEP_FAULT_ABORTED = 6,
} PC_StepFault_t;

typedef struct {
    PC_StepState_t state;
    PC_StepResult_t result;
    PC_StepFault_t fault;
    PC_StepTemplate_t template_id;
    uint8_t step_index;
    float progress;
} PC_StepFeedback_t;

typedef struct {
    uint8_t online;
    uint32_t recv_count;
    float cpu_temp;
    PC_CommandSource_t command_source;
} PC_StatusFeedback_t;

typedef struct {
    PC_ChassisCMD_t chassis;
    PC_PoleCMD_t pole;
    PC_ArmCMD_t arm;
    PC_ArmSimpleCMD_t arm_simple;
    PC_RodNewCMD_t rod_new;
    PC_OreStoreCMD_t ore_store;
    PC_StepCMD_t step;
    PC_ImuCMD_t imu;
} MrlinkPc_CMD_Data_t;

typedef struct {
    PC_ChassisFeedback_t chassis;
    PC_PoleFeedback_t pole;
    PC_ArmFeedback_t arm;
    PC_ArmSimpleFeedback_t arm_simple;
    PC_RodNewFeedback_t rod_new;
    PC_OreStoreFeedback_t ore_store;
    PC_StepFeedback_t step;
    PC_StatusFeedback_t status;
} MrlinkPc_FeedbackData_t;

typedef struct {
    PC_ControlMode_t control_mode;
    bool heartbeat_valid;
    uint32_t last_heartbeat_tick;
    uint32_t last_recv_time;
    uint32_t recv_count;
    uint32_t error_count;
    bool online;
    MrlinkPc_CMD_Data_t cmd;
    MrlinkPc_FeedbackData_t feedback;
} MrlinkPc_State_t;

typedef struct {
    uint8_t usart10_config_ok;
    uint32_t usart10_baudrate;
    uint32_t usart10_word_length;
    uint32_t usart10_stop_bits;
    uint32_t usart10_parity;
    uint32_t usart10_mode;

    uint8_t online;
    uint8_t heartbeat_valid;
    uint8_t control_mode;
    uint8_t command_source;
    uint32_t last_heartbeat_tick;
    uint32_t last_recv_time;
    uint32_t recv_count;
    uint32_t error_count;
    uint32_t rx_heartbeat_count;
    uint32_t rx_chassis_count;
    uint32_t rx_pole_count;
    uint32_t rx_arm_simple_count;
    uint32_t rx_rod_new_count;
    uint32_t rx_ore_store_count;
    uint32_t rx_step_count;
    uint32_t rx_imu_count;
    uint32_t init_fail_count;
    int8_t last_init_error;
    uint32_t rx_restart_fail_count;
    uint32_t rx_header_skip_count;
    uint32_t rx_irq_count;
    uint32_t rx_irq_byte_count;
    uint32_t rx_queue_overflow_count;
    uint32_t rx_dma_start_fail_count;

    uint32_t rx_batch_count;
    uint16_t rx_batch_len;
    uint8_t rx_batch_frame_count;
    uint16_t rx_batch_raw_len;
    uint8_t rx_batch_raw[PC_COMM_DEBUG_RX_RAW_SIZE];

    uint8_t last_rx_cmd;
    uint8_t last_rx_len;
    int8_t last_rx_result;
    uint16_t last_rx_frame_size;
    uint16_t last_rx_crc_received;
    uint16_t last_rx_crc_calculated;
    uint16_t last_rx_raw_len;
    uint8_t last_rx_raw[MRLINK_PC_MAX_FRAME_SIZE];

    PC_ChassisCMD_t rx_chassis;
    PC_PoleCMD_t rx_pole;
    PC_ArmSimpleCMD_t rx_arm_simple;
    PC_RodNewCMD_t rx_rod_new;
    PC_OreStoreCMD_t rx_ore_store;
    PC_StepCMD_t rx_step;
    PC_ImuCMD_t rx_imu;

    uint32_t tx_count;
    uint8_t tx_dma_busy;
    uint32_t tx_busy_skip_count;
    uint32_t tx_dma_error_count;
    uint16_t tx_len;
    uint8_t tx_frame_count;
    int8_t tx_result;
    uint16_t tx_raw_len;
    uint8_t tx_raw[PC_COMM_DEBUG_TX_RAW_SIZE];

    MrLink_Stats_t mrlink_stats;
} PC_CommDebug_t;

typedef struct {
    auto_ctrl_template_e template_id;
    auto_ctrl_travel_dir_e travel_dir;
    float target_yaw_rad;
    float yaw_tolerance_rad;
} PC_AutoStepParams_t;

extern volatile PC_CommDebug_t g_pc_comm_debug;

bool MrlinkPc_CommInit(void);
void MrlinkPc_CommProcess(uint32_t now_ms);
bool MrlinkPc_CommIsOnline(void);
const MrlinkPc_State_t *MrlinkPc_GetState(void);
void MrlinkPc_DebugUpdate(void);

bool MrlinkPc_IsPCControlMode(void);
bool MrlinkPc_IsHeartbeatValid(void);
const PC_ChassisCMD_t *MrlinkPc_GetChassisCMD(void);
const PC_PoleCMD_t *MrlinkPc_GetPoleCMD(void);
const PC_ArmCMD_t *MrlinkPc_GetArmCMD(void);
const PC_ArmSimpleCMD_t *MrlinkPc_GetArmSimpleCMD(void);
const PC_RodNewCMD_t *MrlinkPc_GetRodNewCMD(void);
const PC_OreStoreCMD_t *MrlinkPc_GetOreStoreCMD(void);
const PC_StepCMD_t *MrlinkPc_GetStepCMD(void);
const PC_ImuCMD_t *MrlinkPc_GetImuCMD(void);

void MrlinkPc_SetChassisFeedback(const PC_ChassisFeedback_t *fb);
void MrlinkPc_SetPoleFeedback(const PC_PoleFeedback_t *fb);
void MrlinkPc_SetArmFeedback(const PC_ArmFeedback_t *fb);
void MrlinkPc_SetArmSimpleFeedback(const PC_ArmSimpleFeedback_t *fb);
void MrlinkPc_SetRodNewFeedback(const PC_RodNewFeedback_t *fb);
void MrlinkPc_SetOreStoreFeedback(const PC_OreStoreFeedback_t *fb);
void MrlinkPc_SetStepFeedback(const PC_StepFeedback_t *fb);
void MrlinkPc_SetStatusFeedback(const PC_StatusFeedback_t *fb);

const PC_AutoStepParams_t *MrlinkPc_GetAutoStepParams(void);
void MrlinkPc_ClearStepCommand(void);
uint16_t MrlinkPc_BuildFeedbackFrame(uint8_t cmd, uint8_t *tx_buf,
                                     uint16_t buf_size);

#ifdef __cplusplus
}
#endif