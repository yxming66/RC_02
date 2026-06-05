#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "device/mrlink/mrlink_channel.h"
#include "module/autoCtrlAPI/core/auto_ctrl_def.h"
#include "mrlink/mrlink.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PC_MODE_RC = 0,    /* 遥控器控制模式 */
    PC_MODE_PC = 1,    /* PC 控制模式 */
} PC_ControlMode_t;

typedef enum {
    PC_COMMAND_SOURCE_RC = 0,    /* 当前命令来源为遥控器 */
    PC_COMMAND_SOURCE_PC = 1,    /* 当前命令来源为 PC */
} PC_CommandSource_t;

typedef enum {
    PC_CMD_HEARTBEAT = 0x01,     /* PC 心跳，切入/保持 PC 在线控制 */
    PC_CMD_CHASSIS = 0x10,       /* 底盘速度控制命令 */
    PC_CMD_POLE = 0x11,          /* 撑杆目标高度控制命令 */
    PC_CMD_STEP = 0x12,          /* 自动上下台阶流程启动命令 */
    PC_CMD_ARM_SIMPLE = 0x13,    /* 简易机械臂关节/吸盘控制命令 */
    PC_CMD_ROD_NEW = 0x14,       /* 取矛头机构舵机/夹爪控制命令 */
    PC_CMD_ORE_STORE = 0x15,     /* 矿仓平台控制命令 */
    PC_CMD_AUTO_ACTION = 0x16,   /* 一键取矿/存矿/上膛/放矿/取矛头命令 */
    PC_CMD_IMU = 0x20,           /* PC 下发姿态数据命令 */
} PC_CMD_t;

typedef enum {
    PC_FEEDBACK_HEARTBEAT = 0x81,    /* STM32 心跳反馈 */
    PC_FEEDBACK_CHASSIS = 0x90,      /* 底盘速度反馈 */
    PC_FEEDBACK_POLE = 0x91,         /* 撑杆位置/电机角度反馈 */
    PC_FEEDBACK_STEP = 0x92,         /* 自动上下台阶流程状态反馈 */
    PC_FEEDBACK_ARM_SIMPLE = 0x93,   /* 简易机械臂状态反馈 */
    PC_FEEDBACK_ROD_NEW = 0x94,      /* 取矛头机构状态反馈 */
    PC_FEEDBACK_ORE_STORE = 0x95,    /* 矿仓平台状态反馈 */
    PC_FEEDBACK_STATUS = 0xA0,       /* 通信在线、接收计数、CPU 温度等状态反馈 */
    PC_FEEDBACK_AUTO_ACTION = 0x96,
    PC_FEEDBACK_IR_ORE = 0x97,
} PC_FeedbackCMD_t;

#define MRLINK_PC_MAX_PAYLOAD_SIZE (64u)
#define MRLINK_PC_MAX_FRAME_SIZE (2u + 1u + 1u + MRLINK_PC_MAX_PAYLOAD_SIZE + 2u)
#define PC_COMM_DEBUG_RX_RAW_SIZE (256u)
#define PC_COMM_DEBUG_TX_RAW_SIZE (96u)

#ifndef MRLINK_PC_RX_DMA_SLOT_COUNT
#define MRLINK_PC_RX_DMA_SLOT_COUNT (4u)
#endif

#ifndef MRLINK_PC_RX_DMA_BUF_SIZE
#define MRLINK_PC_RX_DMA_BUF_SIZE (256u)
#endif

#ifndef MRLINK_PC_RX_STREAM_BUF_SIZE
#define MRLINK_PC_RX_STREAM_BUF_SIZE (512u)
#endif

typedef struct {
    float vx;    /* 底盘 x 方向目标速度，单位 m/s */
    float vy;    /* 底盘 y 方向目标速度，单位 m/s */
    float wz;    /* 底盘目标角速度，单位 rad/s */
} PC_ChassisCMD_t;

typedef struct {
    uint8_t mode;      /* 撑杆控制模式，0=放松，其它=主动控制 */
    float lift[2];     /* 撑杆目标高度/角度，[0]=前组，[1]=后组，单位 rad */
} PC_PoleCMD_t;

typedef struct {
    float y;        /* 机械臂笛卡尔 y 目标/速度，当前保留 */
    float z;        /* 机械臂笛卡尔 z 目标/速度，当前保留 */
    float pitch;    /* 机械臂末端 pitch 目标/速度，当前保留，单位 rad */
} PC_ArmCMD_t;

typedef struct {
    uint8_t mode;                /* 简易机械臂模式，见 ArmSimple_Mode_t */
    uint8_t point_mode;          /* 简易机械臂点位模式，见 ArmSimple_PointMode_t */
    uint8_t suction;             /* 吸盘控制，0=关闭，1=开启 */
    float target_joint1_rad;     /* 关节 1 目标角度，单位 rad */
    float target_joint2_rad;     /* 关节 2 目标角度，单位 rad */
} PC_ArmSimpleCMD_t;

typedef struct {
    uint8_t mode;              /* 取矛头机构模式，见 RodNew_Mode_t */
    uint8_t pose;              /* 取矛头机构姿态，见 RodNew_Pose_t */
    uint8_t grip;              /* 夹爪控制，0=松开，1=夹紧 */
    float target_angle_rad;    /* 手动模式下舵机目标角度，单位 rad */
} PC_RodNewCMD_t;

typedef struct {
    uint8_t mode;                  /* 矿仓模式，见 OreStore_Mode_t */
    uint8_t force_rehome;          /* 强制重新回零，0=不触发，1=触发一次 */
    float platform_target_rad;     /* 平台轴目标位置，单位 rad */
} PC_OreStoreCMD_t;

typedef enum {
    PC_AUTO_ACTION_NONE = 0,             /* 无一键动作 */
    PC_AUTO_ACTION_STORE = 1,            /* 一键存矿 */
    PC_AUTO_ACTION_RELEASE = 2,          /* 一键放矿 */
    PC_AUTO_ACTION_CHAMBER = 3,          /* 一键上膛 */
    PC_AUTO_ACTION_PICK_POS_400 = 4,     /* 一键取正 400mm 矿 */
    PC_AUTO_ACTION_PICK_POS_200 = 5,     /* 一键取正 200mm 矿 */
    PC_AUTO_ACTION_PICK_NEG_200 = 6,     /* 一键取反 200mm 矿 */
    PC_AUTO_ACTION_ROD_SPEARHEAD = 7,    /* 一键取矛头 */
    PC_AUTO_ACTION_ABORT = 8,            /* 中止一键动作 */
} PC_AutoAction_t;

typedef enum {
    PC_AUTO_ACTION_SUBSYSTEM_NONE = 0,
    PC_AUTO_ACTION_SUBSYSTEM_ORE = 1,
    PC_AUTO_ACTION_SUBSYSTEM_ROD_SPEARHEAD = 2,
} PC_AutoActionSubsystem_t;

typedef enum {
    PC_AUTO_ACTION_STATE_IDLE = 0,
    PC_AUTO_ACTION_STATE_RUNNING = 1,
    PC_AUTO_ACTION_STATE_SUCCESS = 2,
    PC_AUTO_ACTION_STATE_FAIL = 3,
    PC_AUTO_ACTION_STATE_ABORT = 4,
} PC_AutoActionState_t;

typedef enum {
    PC_AUTO_ACTION_RESULT_NONE = 0,
    PC_AUTO_ACTION_RESULT_RUNNING = 1,
    PC_AUTO_ACTION_RESULT_SUCCESS = 2,
    PC_AUTO_ACTION_RESULT_FAIL = 3,
    PC_AUTO_ACTION_RESULT_ABORTED = 4,
} PC_AutoActionResult_t;

typedef enum {
    PC_AUTO_ACTION_FAULT_NONE = 0,
    PC_AUTO_ACTION_FAULT_INVALID_OCCUPANCY = 1,
    PC_AUTO_ACTION_FAULT_INVALID_PARAM = 2,
    PC_AUTO_ACTION_FAULT_NOT_HOMED = 3,
    PC_AUTO_ACTION_FAULT_TIMEOUT = 4,
    PC_AUTO_ACTION_FAULT_ABORTED = 5,
    PC_AUTO_ACTION_FAULT_NO_SPEARHEAD = 6,
} PC_AutoActionFault_t;

#define PC_AUTO_ACTION_FLAG_ORE_INITED (1u << 0)
#define PC_AUTO_ACTION_FLAG_ORE_BUSY (1u << 1)
#define PC_AUTO_ACTION_FLAG_ROD_INITED (1u << 2)
#define PC_AUTO_ACTION_FLAG_ROD_BUSY (1u << 3)

typedef struct {
    uint8_t action;    /* 一键动作类型，见 PC_AutoAction_t */
} PC_AutoActionCMD_t;

typedef enum {
    PC_STEP_TEMPLATE_NONE = 0,              /* 无自动台阶流程 */
    PC_STEP_TEMPLATE_ASCEND_200_HEAD = 1,   /* 头向前上 200mm 台阶 */
    PC_STEP_TEMPLATE_ASCEND_400_HEAD = 2,   /* 头向前上 400mm 台阶 */
    PC_STEP_TEMPLATE_DESCEND_200_HEAD = 3,  /* 头向前下 200mm 台阶 */
    PC_STEP_TEMPLATE_DESCEND_400_HEAD = 4,  /* 头向前下 400mm 台阶 */
    PC_STEP_TEMPLATE_ASCEND_200_TAIL = 5,   /* 尾向前上 200mm 台阶 */
    PC_STEP_TEMPLATE_ASCEND_400_TAIL = 6,   /* 尾向前上 400mm 台阶 */
    PC_STEP_TEMPLATE_DESCEND_200_TAIL = 7,  /* 尾向前下 200mm 台阶 */
    PC_STEP_TEMPLATE_DESCEND_400_TAIL = 8,  /* 尾向前下 400mm 台阶 */
} PC_StepTemplate_t;

typedef enum {
    PC_STEP_DIR_HEAD_FORWARD = 0,  /* 头向前行进 */
    PC_STEP_DIR_TAIL_FORWARD = 1,  /* 尾向前行进 */
} PC_StepDir_t;

typedef struct {
    PC_StepTemplate_t template_id;   /* 自动台阶模板，见 PC_StepTemplate_t */
    PC_StepDir_t travel_dir;         /* 行进方向，见 PC_StepDir_t */
    float target_yaw_rad;            /* 目标航向角，单位 rad */
    float yaw_tolerance_rad;         /* 航向允许误差，单位 rad */
} PC_StepCMD_t;

typedef struct {
    float qw;       /* 四元数 w */
    float qx;       /* 四元数 x */
    float qy;       /* 四元数 y */
    float qz;       /* 四元数 z */
    float roll;     /* 横滚角，单位 rad */
    float pitch;    /* 俯仰角，单位 rad */
    float yaw;      /* 航向角，单位 rad */
} PC_ImuCMD_t;

typedef struct {
    float vx;    /* 底盘 x 方向速度反馈，单位 m/s */
    float vy;    /* 底盘 y 方向速度反馈，单位 m/s */
    float wz;    /* 底盘角速度反馈，单位 rad/s */
} PC_ChassisFeedback_t;

typedef struct {
    float lift[2];                /* 撑杆组高度/角度反馈，[0]=前组，[1]=后组，单位 rad */
    float motor_total_angle[4];    /* 4 个撑杆电机累计角度反馈，单位 rad */
} PC_PoleFeedback_t;

typedef struct {
    uint8_t mode;                  /* 简易机械臂当前模式，见 ArmSimple_Mode_t */
    uint8_t point_mode;            /* 简易机械臂点位模式，见 ArmSimple_PointMode_t */
    uint8_t suction;               /* 吸盘状态，0=关闭，1=开启 */
    float joint1_angle_rad;        /* 关节 1 当前角度，单位 rad */
    float joint1_velocity_rad_s;   /* 关节 1 当前速度，单位 rad/s */
    float joint2_angle_rad;        /* 关节 2 当前角度，单位 rad */
} PC_ArmSimpleFeedback_t;

typedef struct {
    uint8_t mode;                    /* 取矛头机构当前模式，见 RodNew_Mode_t */
    uint8_t pose;                    /* 取矛头机构目标姿态，见 RodNew_Pose_t */
    uint8_t grip;                    /* 夹爪状态，0=松开，1=夹紧 */
    uint8_t at_target;               /* 到位标志，0=未到位，1=已到位 */
    float target_angle_rad;          /* 舵机目标角度，单位 rad */
    float tracked_angle_rad;         /* 轨迹规划后的跟踪角度，单位 rad */
    float tracked_velocity_rad_s;    /* 轨迹规划后的跟踪速度，单位 rad/s */
    float feedback_angle_rad;        /* 舵机反馈角度，单位 rad */
} PC_RodNewFeedback_t;

typedef struct {
    uint8_t mode;                    /* 矿仓当前模式，见 OreStore_Mode_t */
    uint8_t all_homed;               /* 全部轴回零标志，0=未全部回零，1=全部已回零 */
    uint8_t online_mask;             /* 轴在线 bitmask，bit0=平台轴在线，1 表示在线 */
    uint8_t homed_mask;              /* 轴回零 bitmask，bit0=平台轴已回零，1 表示已回零 */
    float platform_position_rad;     /* 平台轴当前位置，单位 rad */
} PC_OreStoreFeedback_t;

typedef struct {
    uint8_t action;
    uint8_t busy;
    uint8_t subsystem;
    uint8_t state;
    uint8_t result;
    uint8_t fault;
    uint8_t step_index;
    uint8_t step_phase;
    uint8_t active_position;
    uint8_t occupancy_mask;
    uint8_t ore_action;
    uint8_t ore_state;
    uint8_t ore_result;
    uint8_t ore_fault;
    uint8_t ore_step_index;
    uint8_t ore_step_phase;
    uint8_t ore_active_position;
    uint8_t ore_occupancy_mask;
    uint8_t rod_state;
    uint8_t rod_result;
    uint8_t rod_fault;
    uint8_t rod_step_index;
    uint8_t flags;
    uint8_t reserved;
} PC_AutoActionFeedback_t;

typedef enum {
    PC_ORE_TYPE_UNKNOWN = 0,
    PC_ORE_TYPE_R1 = 1,
    PC_ORE_TYPE_R2 = 2,
    PC_ORE_TYPE_FAKE = 3,
} PC_OreType_t;

#define PC_IR_ORE_POSITION_COUNT (12u)

/* 红外对接反馈帧：把 UART7 收到的 IR 命令（1 字节状态 + 12 字节矿种）
 * 透传给 PC 上位机，由 pc_comm_task 通过 MrlinkPc_PublishFeedback(PC_FEEDBACK_IR_ORE, …) 周期上报。
 * 与 IrDock_OreInfo_t / IrDock_Debug_t 的对应关系见各字段注释。 */
typedef struct {
    uint8_t valid;                                           /* 矿种信息是否曾成功解析过一帧，0=未收到过 12 字节矿种包，1=有效 */
    uint8_t fresh;                                           /* 矿种信息是否新鲜，0=过期/未收到，1=在 IR_DOCK_ORE_INFO_FRESH_MS (1000ms) 内。注意：仅描述矿种包，不代表 status 状态命令 */
    uint8_t status;                                          /* 最近一次 1 字节状态命令，见 IrDock_Status_t (IDLE=0/DOCKING=1/DOCK_COMPLETE=2)，状态命令单独到达也会刷新此字段 */
    uint8_t count;                                           /* 矿位个数，固定等于 PC_IR_ORE_POSITION_COUNT (12) */
    uint8_t ore_type[PC_IR_ORE_POSITION_COUNT];              /* 12 个矿位种类，见 PC_OreType_t (UNKNOWN=0/R1=1/R2=2/FAKE=3)，下标为矿位编号 */
    uint32_t age_ms;                                         /* 距最近一次成功接收 12 字节矿种包的时间，单位 ms；未收到过为 0 */
    uint32_t rx_count;                                       /* 12 字节矿种包累计成功接收次数，CRC/格式错误不会计入；可用于观察对接链路健康度 */
} PC_IrOreFeedback_t;

typedef enum {
    PC_STEP_STATE_IDLE = 0,       /* 空闲 */
    PC_STEP_STATE_PREALIGN = 1,   /* 预对准 */
    PC_STEP_STATE_RUNNING = 2,    /* 流程运行中 */
    PC_STEP_STATE_SUCCESS = 3,    /* 流程成功完成 */
    PC_STEP_STATE_FAIL = 4,       /* 流程失败 */
    PC_STEP_STATE_ABORT = 5,      /* 流程已中止 */
} PC_StepState_t;

typedef enum {
    PC_STEP_RESULT_NONE = 0,       /* 无结果 */
    PC_STEP_RESULT_RUNNING = 1,    /* 正在执行 */
    PC_STEP_RESULT_SUCCESS = 2,    /* 执行成功 */
    PC_STEP_RESULT_FAIL = 3,       /* 执行失败 */
    PC_STEP_RESULT_ABORTED = 4,    /* 被中止 */
} PC_StepResult_t;

typedef enum {
    PC_STEP_FAULT_NONE = 0,                /* 无故障 */
    PC_STEP_FAULT_INVALID_TEMPLATE = 1,    /* 无效自动台阶模板 */
    PC_STEP_FAULT_PREALIGN_TIMEOUT = 2,    /* 预对准超时 */
    PC_STEP_FAULT_TEMPLATE_TIMEOUT = 3,    /* 模板流程超时 */
    PC_STEP_FAULT_SENSOR_INVALID = 4,      /* 传感器数据无效 */
    PC_STEP_FAULT_UNSUPPORTED = 5,         /* 当前模板或参数不支持 */
    PC_STEP_FAULT_ABORTED = 6,             /* 人为中止 */
} PC_StepFault_t;

typedef struct {
    PC_StepState_t state;           /* 当前状态，见 PC_StepState_t */
    PC_StepResult_t result;         /* 当前结果，见 PC_StepResult_t */
    PC_StepFault_t fault;           /* 当前故障，见 PC_StepFault_t */
    PC_StepTemplate_t template_id;  /* 当前执行的台阶模板 */
    uint8_t step_index;             /* 当前模板内部步骤索引 */
    float progress;                 /* 流程进度，0.0~1.0，当前未完整使用时为 0 */
} PC_StepFeedback_t;

typedef struct {
    uint8_t online;                         /* PC 通信在线标志，0=离线，1=在线 */
    uint32_t recv_count;                    /* 已成功接收的 PC 帧计数 */
    float cpu_temp;                         /* STM32 CPU 温度，单位 degC */
    PC_CommandSource_t command_source;      /* 当前命令来源，见 PC_CommandSource_t */
} PC_StatusFeedback_t;

typedef struct {
    PC_ChassisCMD_t chassis;             /* 最近一次底盘命令 */
    PC_PoleCMD_t pole;                   /* 最近一次撑杆命令 */
    PC_ArmCMD_t arm;                     /* 最近一次机械臂笛卡尔命令，当前保留 */
    PC_ArmSimpleCMD_t arm_simple;        /* 最近一次简易机械臂命令 */
    PC_RodNewCMD_t rod_new;              /* 最近一次取矛头机构命令 */
    PC_OreStoreCMD_t ore_store;          /* 最近一次矿仓命令 */
    PC_AutoActionCMD_t auto_action;      /* 待消费的一键动作命令，消费后清零 */
    PC_StepCMD_t step;                   /* 待执行/最近一次自动台阶命令 */
    PC_ImuCMD_t imu;                     /* 最近一次 PC 姿态数据 */
} MrlinkPc_CMD_Data_t;

typedef struct {
    PC_ChassisFeedback_t chassis;          /* 底盘反馈缓存 */
    PC_PoleFeedback_t pole;                /* 撑杆反馈缓存 */
    PC_ArmSimpleFeedback_t arm_simple;     /* 简易机械臂反馈缓存 */
    PC_RodNewFeedback_t rod_new;           /* 取矛头机构反馈缓存 */
    PC_OreStoreFeedback_t ore_store;       /* 矿仓反馈缓存 */
    PC_StepFeedback_t step;                /* 自动台阶反馈缓存 */
    PC_StatusFeedback_t status;            /* 通信/系统状态反馈缓存 */
} MrlinkPc_FeedbackData_t;

typedef struct {
    PC_ControlMode_t control_mode;       /* PC 控制模式，见 PC_ControlMode_t */
    bool heartbeat_valid;                /* 心跳有效标志，false=心跳超时 */
    uint32_t last_heartbeat_tick;        /* 最近一次心跳 tick，单位 ms */
    uint32_t last_recv_time;             /* 最近一次收到任意 PC 帧的 tick，单位 ms */
    uint32_t recv_count;                 /* 成功接收帧计数 */
    uint32_t error_count;                /* 通信解析错误计数 */
    bool online;                         /* PC 在线标志，false=离线，true=在线 */
    MrlinkPc_CMD_Data_t cmd;             /* PC 命令缓存 */
    MrlinkPc_FeedbackData_t feedback;    /* 反馈缓存 */
} MrlinkPc_State_t;

typedef struct {
    uint8_t usart10_config_ok;         /* USART10 配置检查，0=异常，1=符合 PC 通信配置 */
    uint32_t usart10_baudrate;         /* USART10 波特率 */
    uint32_t usart10_word_length;      /* USART10 数据位配置，HAL UART_WORDLENGTH_* */
    uint32_t usart10_stop_bits;        /* USART10 停止位配置，HAL UART_STOPBITS_* */
    uint32_t usart10_parity;           /* USART10 校验位配置，HAL UART_PARITY_* */
    uint32_t usart10_mode;             /* USART10 收发模式配置，HAL UART_MODE_* */

    uint8_t online;                    /* PC 在线标志，0=离线，1=在线 */
    uint8_t heartbeat_valid;           /* PC 心跳有效标志，0=超时/无效，1=有效 */
    uint8_t control_mode;              /* PC 控制模式，见 PC_ControlMode_t */
    uint8_t command_source;            /* 当前命令来源，见 PC_CommandSource_t */
    uint32_t last_heartbeat_tick;      /* 最近一次 PC 心跳 tick，单位 ms */
    uint32_t last_recv_time;           /* 最近一次收到任意 PC 帧的 tick，单位 ms */
    uint32_t recv_count;               /* 成功接收并识别的 PC 帧总数 */
    uint32_t error_count;              /* PC 通信解析错误总数 */
    uint32_t rx_heartbeat_count;       /* 收到心跳命令次数 */
    uint32_t rx_chassis_count;         /* 收到底盘命令次数 */
    uint32_t rx_pole_count;            /* 收到撑杆命令次数 */
    uint32_t rx_arm_simple_count;      /* 收到简易机械臂命令次数 */
    uint32_t rx_rod_new_count;         /* 收到取矛头机构命令次数 */
    uint32_t rx_ore_store_count;       /* 收到矿仓命令次数 */
    uint32_t rx_auto_action_count;     /* 收到一键动作命令次数 */
    uint32_t rx_step_count;            /* 收到自动台阶命令次数 */
    uint32_t rx_imu_count;             /* 收到 PC 姿态数据次数 */
    uint32_t init_fail_count;          /* PC 通信初始化失败次数 */
    int8_t last_init_error;            /* 最近一次初始化错误码 */
    uint32_t rx_restart_fail_count;    /* UART/FDCAN/USB 接收重启失败次数 */
    uint32_t rx_header_skip_count;     /* mrlink 跳过非帧头字节计数 */
    uint32_t rx_irq_count;             /* 底层接收完成中断/事件次数 */
    uint32_t rx_irq_byte_count;        /* 底层接收完成中断/事件累计字节数 */
    uint32_t rx_queue_overflow_count;  /* 底层接收队列溢出次数 */
    uint32_t rx_dma_start_fail_count;  /* UART DMA 接收启动失败次数 */
    uint32_t mrlink_error_count;       /* mrlink 协议错误事件次数 */
    uint8_t mrlink_last_error_code;    /* 最近一次 mrlink 错误类型，见 MrLink_ErrorCode_t */
    uint8_t mrlink_last_error_cmd;     /* 最近一次 mrlink 错误关联命令 */
    uint16_t mrlink_last_error_payload_len;       /* 最近一次错误帧 payload 长度 */
    uint16_t mrlink_last_error_expected_size;     /* 最近一次错误期望长度/上限 */
    uint16_t mrlink_last_error_available;         /* 最近一次错误发生时可用字节数 */
    uint16_t mrlink_last_error_dropped_bytes;     /* 最近一次错误丢弃字节数 */
    uint16_t mrlink_last_error_crc_received;      /* 最近一次错误帧接收到的 CRC */
    uint16_t mrlink_last_error_crc_calculated;    /* 最近一次错误帧计算出的 CRC */

    uint32_t rx_batch_count;           /* 从底层通道弹出的接收批次数 */
    uint16_t rx_batch_len;             /* 最近一批接收数据长度 */
    uint8_t rx_batch_frame_count;      /* 最近一批数据中成功解析出的帧数 */
    uint16_t rx_batch_raw_len;         /* rx_batch_raw 中保存的有效字节数 */
    uint8_t rx_batch_raw[PC_COMM_DEBUG_RX_RAW_SIZE];    /* 最近一批原始接收数据 */

    uint8_t last_rx_cmd;               /* 最近一次接收帧命令字 */
    uint8_t last_rx_len;               /* 最近一次接收帧 payload 长度 */
    int8_t last_rx_result;             /* 最近一次接收处理结果，MRLINK_OK/错误码 */
    uint16_t last_rx_frame_size;       /* 最近一次接收帧完整长度 */
    uint16_t last_rx_crc_received;     /* 最近一次接收帧携带的 CRC */
    uint16_t last_rx_crc_calculated;   /* 最近一次接收帧计算出的 CRC */
    uint16_t last_rx_raw_len;          /* last_rx_raw 中保存的有效字节数 */
    uint8_t last_rx_raw[MRLINK_PC_MAX_FRAME_SIZE];      /* 最近一次原始接收帧 */

    PC_ChassisCMD_t rx_chassis;             /* 调试用：最近一次底盘命令 */
    PC_PoleCMD_t rx_pole;                   /* 调试用：最近一次撑杆命令 */
    PC_ArmSimpleCMD_t rx_arm_simple;        /* 调试用：最近一次简易机械臂命令 */
    PC_RodNewCMD_t rx_rod_new;              /* 调试用：最近一次取矛头机构命令 */
    PC_OreStoreCMD_t rx_ore_store;          /* 调试用：最近一次矿仓命令 */
    PC_AutoActionCMD_t rx_auto_action;      /* 调试用：最近一次一键动作命令 */
    PC_StepCMD_t rx_step;                   /* 调试用：最近一次自动台阶命令 */
    PC_ImuCMD_t rx_imu;                     /* 调试用：最近一次 PC 姿态数据 */

    uint32_t tx_count;                /* 反馈发送尝试次数 */
    uint8_t tx_dma_busy;              /* 发送 DMA 忙标志，0=空闲，1=忙 */
    uint32_t tx_busy_skip_count;      /* 因发送 DMA 忙而跳过发送的次数 */
    uint32_t tx_dma_error_count;      /* 发送 DMA 错误/超时次数 */
    uint16_t tx_len;                  /* 最近一次发送总长度 */
    uint8_t tx_frame_count;           /* 最近一次发送中打包的反馈帧数量 */
    int8_t tx_result;                 /* 最近一次发送结果，MRLINK_CHANNEL_OK/错误码 */
    uint16_t tx_raw_len;              /* tx_raw 中保存的有效字节数 */
    uint8_t tx_raw[PC_COMM_DEBUG_TX_RAW_SIZE];          /* 最近一次原始发送数据 */

    MrLink_Stats_t mrlink_stats;      /* mrlink 协议层统计快照 */
} PC_CommDebug_t;

typedef struct {
    auto_ctrl_template_e template_id;       /* 映射后的自动台阶模板 */
    auto_ctrl_travel_dir_e travel_dir;      /* 映射后的行进方向 */
    float target_yaw_rad;                   /* 自动台阶目标航向角，单位 rad */
    float yaw_tolerance_rad;                /* 自动台阶航向允许误差，单位 rad */
} PC_AutoStepParams_t;

extern volatile PC_CommDebug_t g_pc_comm_debug;

typedef void (*MrlinkPc_TxCallback_t)(void);

bool MrlinkPc_SelectUart(BSP_UART_t uart);
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
const PC_AutoActionCMD_t *MrlinkPc_GetAutoActionCMD(void);
const PC_StepCMD_t *MrlinkPc_GetStepCMD(void);
const PC_ImuCMD_t *MrlinkPc_GetImuCMD(void);

bool MrlinkPc_PublishFeedback(uint8_t topic, const void *feedback);

const PC_AutoStepParams_t *MrlinkPc_GetAutoStepParams(void);
void MrlinkPc_ClearStepCommand(void);
void MrlinkPc_ClearAutoActionCommand(void);
uint16_t MrlinkPc_BuildFeedbackFrame(uint8_t cmd, uint8_t *tx_buf,
                                     uint16_t buf_size);
int8_t MrlinkPc_SendFrame(uint8_t *data, uint16_t len);
int8_t MrlinkPc_RegisterTxCallbacks(MrlinkPc_TxCallback_t tx_done,
                                    MrlinkPc_TxCallback_t tx_error);

#ifdef __cplusplus
}

namespace mr {
namespace link {
class Instance;
}
}

mr::link::Instance &MrlinkPc_Bus(void);
#endif
