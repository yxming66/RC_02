#pragma once

/* Includes ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>

/* USER INCLUDE BEGIN */
#include "module/autoCtrlAPI/core/auto_ctrl_def.h"
/* USER INCLUDE END */

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ----------------------------------------------------------- */
/* PC控制模式 */
typedef enum {
    PC_MODE_RC = 0,
    PC_MODE_PC = 1,
} PC_ControlMode_t;

/* 通信命令 */
typedef enum {
    PC_CMD_HEARTBEAT = 0x01,
    PC_CMD_CHASSIS = 0x10,
    PC_CMD_POLE = 0x11,
    PC_CMD_STEP = 0x12,       /* 自动上台阶命令 */
    PC_CMD_IMU = 0x20,        /* IMU四元数/欧拉角命令 */
} PC_CMD_t;

/* 反馈命令 */
typedef enum {
    PC_FEEDBACK_HEARTBEAT = 0x81,
    PC_FEEDBACK_CHASSIS = 0x90,
    PC_FEEDBACK_POLE = 0x91,
    PC_FEEDBACK_STEP = 0x92,  /* 自动上台阶反馈 */
    PC_FEEDBACK_STATUS = 0xA0,
} PC_FeedbackCMD_t;

/* 帧头 */
#define PC_PROTOCOL_FRAME_HEADER_0 (0xAA)
#define PC_PROTOCOL_FRAME_HEADER_1 (0x55)
#define PC_PROTOCOL_MAX_PAYLOAD_SIZE (64)
#define PC_PROTOCOL_MAX_FRAME_SIZE (2u + 1u + 1u + PC_PROTOCOL_MAX_PAYLOAD_SIZE + 2u)
#define PC_COMM_DEBUG_RX_RAW_SIZE (256u)
#define PC_COMM_DEBUG_TX_RAW_SIZE (96u)

/* Chassis命令 */
typedef struct {
    float vx;
    float vy;
    float wz;
} PC_ChassisCMD_t;

/* Pole命令 */
typedef struct {
    uint8_t mode;
    float lift[2];  /* rad, target lift relative to calibrated lower limit */
} PC_PoleCMD_t;

/* Arm命令 */
typedef struct {
    float y;
    float z;
    float pitch;
} PC_ArmCMD_t;

/* 自动上台阶命令 - 对应 auto_ctrl_template_e */
typedef enum {
    PC_STEP_TEMPLATE_NONE = 0,              /* 未分配模板 */
    PC_STEP_TEMPLATE_ASCEND_200_HEAD = 1,   /* 头向上 200mm 台阶 */
    PC_STEP_TEMPLATE_ASCEND_400_HEAD = 2,   /* 头向上 400mm 台阶 */
    PC_STEP_TEMPLATE_DESCEND_200_HEAD = 3,  /* 头向下 200mm 台阶 */
    PC_STEP_TEMPLATE_DESCEND_400_HEAD = 4,  /* 头向下 400mm 台阶 */
    PC_STEP_TEMPLATE_ASCEND_200_TAIL = 5,   /* 尾向上 200mm 台阶 */
    PC_STEP_TEMPLATE_ASCEND_400_TAIL = 6,  /* 尾向上 400mm 台阶 */
    PC_STEP_TEMPLATE_DESCEND_200_TAIL = 7,  /* 尾向下 200mm 台阶 */
    PC_STEP_TEMPLATE_DESCEND_400_TAIL = 8, /* 尾向下 400mm 台阶 */
} PC_StepTemplate_t;

/* 行进方向 - 对应 auto_ctrl_travel_dir_e */
typedef enum {
    PC_STEP_DIR_HEAD_FORWARD = 0,  /* 头部向前 */
    PC_STEP_DIR_TAIL_FORWARD = 1,  /* 尾部向前 */
} PC_StepDir_t;

/* 自动上台阶命令 */
typedef struct {
    PC_StepTemplate_t template_id;  /* 模板类型 */
    PC_StepDir_t travel_dir;        /* 行进方向 */
    float target_yaw_rad;           /* 目标偏航角 (rad) */
    float yaw_tolerance_rad;        /* yaw 容差 (rad) */
} PC_StepCMD_t;

/* IMU四元数/欧拉角命令 */
typedef struct {
    float qw;          /* 四元数 w */
    float qx;          /* 四元数 x */
    float qy;          /* 四元数 y */
    float qz;          /* 四元数 z */
    float roll;        /* 横滚角 (rad) */
    float pitch;       /* 俯仰角 (rad) */
    float yaw;         /* 偏航角 (rad) */
} PC_ImuCMD_t;

/* Chassis反馈 */
typedef struct {
    float vx;
    float vy;
    float wz;
} PC_ChassisFeedback_t;

/* Pole反馈 */
typedef struct {
    float lift[2];
} PC_PoleFeedback_t;

/* Arm反馈 */
typedef struct {
    float y;
    float z;
    float pitch;
} PC_ArmFeedback_t;

/* 自动上台阶反馈状态 - 对应 auto_ctrl_run_state_e */
typedef enum {
    PC_STEP_STATE_IDLE = 0,       /* 空闲态 */
    PC_STEP_STATE_PREALIGN = 1,   /* 姿态预对齐阶段 */
    PC_STEP_STATE_RUNNING = 2,    /* 模板执行阶段 */
    PC_STEP_STATE_SUCCESS = 3,    /* 任务成功结束 */
    PC_STEP_STATE_FAIL = 4,       /* 任务失败结束 */
    PC_STEP_STATE_ABORT = 5,      /* 外部中止 */
} PC_StepState_t;

/* 任务结果 - 对应 auto_ctrl_result_e */
typedef enum {
    PC_STEP_RESULT_NONE = 0,     /* 未开始 */
    PC_STEP_RESULT_RUNNING = 1,  /* 执行中 */
    PC_STEP_RESULT_SUCCESS = 2,  /* 成功 */
    PC_STEP_RESULT_FAIL = 3,     /* 失败 */
    PC_STEP_RESULT_ABORTED = 4, /* 被中止 */
} PC_StepResult_t;

/* 故障码 - 对应 auto_ctrl_fault_e */
typedef enum {
    PC_STEP_FAULT_NONE = 0,              /* 无故障 */
    PC_STEP_FAULT_INVALID_TEMPLATE = 1,  /* 模板ID非法 */
    PC_STEP_FAULT_PREALIGN_TIMEOUT = 2,  /* 预对齐超时 */
    PC_STEP_FAULT_TEMPLATE_TIMEOUT = 3,  /* 模板执行超时 */
    PC_STEP_FAULT_SENSOR_INVALID = 4,    /* 传感器异常 */
    PC_STEP_FAULT_UNSUPPORTED = 5,       /* 不支持的模板/step */
    PC_STEP_FAULT_ABORTED = 6,           /* 被上层中止 */
} PC_StepFault_t;

/* 自动上台阶反馈 */
typedef struct {
    PC_StepState_t state;        /* 运行状态 */
    PC_StepResult_t result;      /* 任务结果 */
    PC_StepFault_t fault;        /* 故障码 */
    PC_StepTemplate_t template_id; /* 当前执行模板 */
    uint8_t step_index;          /* 当前 step 编号 */
    float progress;              /* 进度 0.0~1.0 */
} PC_StepFeedback_t;

/* Status反馈 */
typedef struct {
    uint8_t online;
    uint32_t recv_count;
    float cpu_temp;
} PC_StatusFeedback_t;

/* 命令数据 */
typedef struct {
    PC_ChassisCMD_t chassis;
    PC_PoleCMD_t pole;
    PC_ArmCMD_t arm;
    PC_StepCMD_t step;
    PC_ImuCMD_t imu;
} PC_CMD_Data_t;

/* 反馈数据 */
typedef struct {
    PC_ChassisFeedback_t chassis;
    PC_PoleFeedback_t pole;
    PC_ArmFeedback_t arm;
    PC_StepFeedback_t step;
    PC_StatusFeedback_t status;
} PC_FeedbackData_t;

/* PC协议结构体 */
typedef struct {
    PC_ControlMode_t control_mode;
    bool heartbeat_valid;
    uint32_t last_heartbeat_tick;
    PC_CMD_Data_t cmd;
    PC_FeedbackData_t feedback;
} PC_Protocol_t;

/* PC通信结构体 */
typedef struct {
    PC_Protocol_t protocol;
    uint32_t last_recv_time;
    uint32_t recv_count;
    uint32_t error_count;
    bool online;
} PC_Comm_t;

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
    uint32_t last_heartbeat_tick;
    uint32_t last_recv_time;
    uint32_t recv_count;
    uint32_t error_count;
    uint32_t rx_heartbeat_count;
    uint32_t rx_chassis_count;
    uint32_t rx_pole_count;
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
    uint8_t last_rx_raw[PC_PROTOCOL_MAX_FRAME_SIZE];

    PC_ChassisCMD_t rx_chassis;
    PC_PoleCMD_t rx_pole;
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
} PC_CommDebug_t;

extern volatile PC_CommDebug_t g_pc_comm_debug;

/* Exported function prototypes --------------------------------------------- */
void PC_Protocol_Init(PC_Protocol_t *proto);
void PC_Protocol_Update(PC_Protocol_t *proto);
int8_t PC_Protocol_ParseFrame(PC_Protocol_t *proto, const uint8_t *data, uint16_t size);
uint16_t PC_Protocol_BuildFrame(PC_Protocol_t *proto, uint8_t cmd, uint8_t *tx_buf, uint16_t buf_size);
bool PC_Protocol_IsPCControlMode(const PC_Protocol_t *proto);
bool PC_Protocol_IsHeartbeatValid(const PC_Protocol_t *proto);
const PC_ChassisCMD_t* PC_Protocol_GetChassisCMD(const PC_Protocol_t *proto);
const PC_PoleCMD_t* PC_Protocol_GetPoleCMD(const PC_Protocol_t *proto);
const PC_ArmCMD_t* PC_Protocol_GetArmCMD(const PC_Protocol_t *proto);
const PC_StepCMD_t* PC_Protocol_GetStepCMD(const PC_Protocol_t *proto);
const PC_ImuCMD_t* PC_Protocol_GetImuCMD(const PC_Protocol_t *proto);
void PC_Protocol_SetChassisFeedback(PC_Protocol_t *proto, const PC_ChassisFeedback_t *fb);
void PC_Protocol_SetPoleFeedback(PC_Protocol_t *proto, const PC_PoleFeedback_t *fb);
void PC_Protocol_SetArmFeedback(PC_Protocol_t *proto, const PC_ArmFeedback_t *fb);
void PC_Protocol_SetStepFeedback(PC_Protocol_t *proto, const PC_StepFeedback_t *fb);
void PC_Protocol_SetStatusFeedback(PC_Protocol_t *proto, const PC_StatusFeedback_t *fb);

/* PC Step命令参数映射结果 - 使用auto_ctrl枚举类型 */
typedef struct {
    auto_ctrl_template_e template_id;
    auto_ctrl_travel_dir_e travel_dir;
    float target_yaw_rad;
    float yaw_tolerance_rad;
} PC_AutoStepParams_t;

const PC_AutoStepParams_t* PC_Protocol_GetAutoStepParams(const PC_Protocol_t *proto);

/* PC通信管理函数 */
bool PC_Comm_Init(PC_Comm_t *comm);
bool PC_Comm_IsOnline(const PC_Comm_t *comm);
void PC_Comm_Process(PC_Comm_t *comm, uint32_t now_ms);
PC_Comm_t* PC_Comm_GetInstance(void);
void PC_Comm_DebugUpdate(const PC_Comm_t *comm);
void PC_Comm_DebugRecordRxBatch(const uint8_t *data, uint16_t len);
void PC_Comm_DebugRecordRxFrame(const PC_Comm_t *comm, const uint8_t *frame,
                                uint16_t frame_size, int8_t parse_result);
void PC_Comm_DebugRecordRxError(const PC_Comm_t *comm, const uint8_t *data,
                                uint16_t available_size,
                                int8_t parse_result);
void PC_Comm_DebugRecordTx(const uint8_t *data, uint16_t len,
                           uint8_t frame_count, int8_t tx_result);

/* USER FUNCTION BEGIN */
/* USER FUNCTION END */

#ifdef __cplusplus
}
#endif
