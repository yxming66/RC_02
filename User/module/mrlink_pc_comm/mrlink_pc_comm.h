#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "device/mrlink/mrlink_channel.h"
#include "module/autoCtrlAPI/core/auto_ctrl_def.h"
#include "mrlink/mrlink.h"

#define PC_CAMERA_YAW_COUNT (1u)

#ifdef __cplusplus
extern "C" {
#endif

/*
 * PC_COMM is a topic-style protocol carried by mrlink frames:
 *   [0x4D][0x52][payload_len][cmd][payload...][crc16_lo][crc16_hi]
 *
 * Important wire-layout note:
 * Some firmware-side structs below are internal command/cache types and contain
 * enum fields that are 4 bytes on ARM GCC. The real serial payload for those
 * commands/feedbacks is defined in pc_messages.hpp as packed pc_comm::wire::*
 * structs. Host software should follow docs/pccomm_protocol.md or
 * pc_messages.hpp exact offsets, not blindly mirror every C struct here.
 */

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
    PC_CMD_CAMERA_YAW = 0x17,    /* 相机云台 yaw 保持命令 */
    PC_CMD_ABSTRACT_POSITION = 0x18, /* 抽象位置命令 */
    PC_CMD_IMU = 0x20,           /* PC 下发姿态数据命令 */
    PC_CMD_IR_ORE_ACK = 0x21,    /* PC 透传红外对接 ACK 帧，payload 为 6 字节原始 ACK */
    PC_CMD_R2_READY_STATE = 0x22,/* PC 下发 R2 准备/重试状态，驱动灯效模式 */
    PC_CMD_AUTO_ACTION_V2 = 0x23, /* 带 request/job 标识的一键动作事务命令 */
    PC_CMD_AUTO_ACTION_V3 = 0x24, /* 多 Job 资源调度命令 */
} PC_CMD_t;

typedef enum {
    PC_FEEDBACK_HEARTBEAT = 0x81,    /* STM32 心跳反馈 */
    PC_FEEDBACK_START_MATCH = 0x02,  /* STM32->PC 一次性开始比赛命令 */
    PC_FEEDBACK_RETRY = 0x03,        /* STM32->PC 一次性重试请求命令 */
    PC_FEEDBACK_CHASSIS = 0x90,      /* 底盘速度反馈 */
    PC_FEEDBACK_POLE = 0x91,         /* 撑杆位置/电机角度反馈 */
    PC_FEEDBACK_STEP = 0x92,         /* 自动上下台阶流程状态反馈 */
    PC_FEEDBACK_ARM_SIMPLE = 0x93,   /* 简易机械臂状态反馈 */
    PC_FEEDBACK_ROD_NEW = 0x94,      /* 取矛头机构状态反馈 */
    PC_FEEDBACK_ORE_STORE = 0x95,    /* 矿仓平台状态反馈 */
    PC_FEEDBACK_STATUS = 0xA0,       /* 通信在线、接收计数、CPU 温度等状态反馈 */
    PC_FEEDBACK_AUTO_ACTION = 0x96,  /* 一键动作简化结果反馈 */
    PC_FEEDBACK_IR_ORE = 0x97,       /* 红外对接矿种与对接状态反馈 */
    PC_FEEDBACK_CAMERA_YAW = 0x98,   /* 相机云台 yaw 状态反馈 */
    PC_FEEDBACK_IR_ORE_BRIDGE = 0x99,/* 红外对接桥接调试反馈，含 msg_id/side/原始 18 字节帧 */
    PC_FEEDBACK_IR_DOCK = 0x9A,      /* R1/R2 红外对接新协议状态反馈 */
    PC_FEEDBACK_SICK_CORRECT = 0x9B, /* 取矛头 SICK 校正标准值/实时值反馈 */
    PC_FEEDBACK_SICK_FRONT_ORE = 0x9C, /* 前 SICK 区域正方形矿检测反馈。 */
    PC_FEEDBACK_SICK_RAW = 0x9D,      /* 4 路 SICK 原始 ADC/距离/有效位反馈 */
    PC_FEEDBACK_AUTO_ACTION_V2 = 0x9E, /* 一键动作事务、分支和失败状态 */
    PC_FEEDBACK_AUTO_ACTION_V3 = 0x9F, /* 全部 AutoAction Job 快照 */
    PC_FEEDBACK_AUTO_ACTION_V3_REJECT = 0xA1, /* V3 命令拒绝结果 */
} PC_FeedbackCMD_t;


 
#define MRLINK_PC_MAX_PAYLOAD_SIZE (64u)    /* 单帧 payload 最大字节数，上位机结构体不能超过该值 */
#define MRLINK_PC_MAX_FRAME_SIZE (2u + 1u + 1u + MRLINK_PC_MAX_PAYLOAD_SIZE + 2u) /* 完整 mrlink 帧最大长度：帧头+长度+命令+payload+CRC */

#ifndef MRLINK_PC_RX_DMA_SLOT_COUNT
#define MRLINK_PC_RX_DMA_SLOT_COUNT (4u)    /* UART RX DMA 环形接收槽数量 */
#endif

#ifndef MRLINK_PC_RX_DMA_BUF_SIZE
#define MRLINK_PC_RX_DMA_BUF_SIZE (256u)    /* 每个 UART RX DMA 槽的字节数 */
#endif

#ifndef MRLINK_PC_RX_STREAM_BUF_SIZE
#define MRLINK_PC_RX_STREAM_BUF_SIZE (512u) /* mrlink 协议解析流缓冲区字节数 */
#endif

#if !defined(MRLINK_CPP_MAX_LATEST_MESSAGES) || (MRLINK_CPP_MAX_LATEST_MESSAGES < 32u)
#undef MRLINK_CPP_MAX_LATEST_MESSAGES
#define MRLINK_CPP_MAX_LATEST_MESSAGES (32u)
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
    float target_joint1_rad;     /* 关节 1 目标角度，单位 rad */
    float target_joint2_rad;     /* 关节 2 目标角度，单位 rad */
} PC_ArmSimpleCMD_t;

typedef struct {
    uint8_t mode;              /* 取矛头机构模式，见 RodNew_Mode_t */
    uint8_t pose;              /* 取矛头机构姿态，见 RodNew_Pose_t */
    float target_angle_rad;    /* 手动模式下舵机目标角度，单位 rad */
} PC_RodNewCMD_t;

typedef struct {
    uint8_t mode;                  /* 矿仓模式，见 OreStore_Mode_t */
    uint8_t force_rehome;          /* 强制重新回零，0=不触发，1=触发一次 */
    float platform_target_rad;     /* 平台轴目标位置，单位 rad */
} PC_OreStoreCMD_t;

typedef struct {
    uint8_t mode;              /* 云台模式；单云台协议映射到右云台 */
    float target_yaw_rad;      /* 车身系目标 yaw，单位 rad */
} PC_CameraYawCMD_t;

typedef enum {
    PC_ABSTRACT_MODULE_ARM_SIMPLE = (1u << 0),
    PC_ABSTRACT_MODULE_ROD_NEW = (1u << 1),
    PC_ABSTRACT_MODULE_ORE_STORE = (1u << 2),
    PC_ABSTRACT_MODULE_POLE = (1u << 3),
} PC_AbstractModuleMask_t;

typedef enum {
    PC_ABSTRACT_ARM_SIMPLE_RELAX = 0,
    PC_ABSTRACT_ARM_SIMPLE_SLEEP = 1,
    PC_ABSTRACT_ARM_SIMPLE_GRAB = 2,
    PC_ABSTRACT_ARM_SIMPLE_LIFT = 3,
    PC_ABSTRACT_ARM_SIMPLE_RELEASE = 4,
    PC_ABSTRACT_ARM_SIMPLE_BEHAVIOR_STANDBY = 16,
    PC_ABSTRACT_ARM_SIMPLE_BEHAVIOR_STORE_ORE = 17,
    PC_ABSTRACT_ARM_SIMPLE_BEHAVIOR_CHAMBER_ORE = 18,
    PC_ABSTRACT_ARM_SIMPLE_BEHAVIOR_WAIT_STORE_ORE = 19,
    PC_ABSTRACT_ARM_SIMPLE_BEHAVIOR_WAIT_RELEASE_ORE = 20,
    PC_ABSTRACT_ARM_SIMPLE_BEHAVIOR_RELEASE_ORE = 21,
    PC_ABSTRACT_ARM_SIMPLE_BEHAVIOR_PICK_POS_400 = 22,
    PC_ABSTRACT_ARM_SIMPLE_BEHAVIOR_PICK_POS_200 = 23,
    PC_ABSTRACT_ARM_SIMPLE_BEHAVIOR_PICK_NEG_200 = 24,
    PC_ABSTRACT_ARM_SIMPLE_BEHAVIOR_PICK_LIFT_DETECT = 25,
    PC_ABSTRACT_ARM_SIMPLE_BEHAVIOR_RELEASE_ORE_ASSIST = 26,
} PC_AbstractArmSimplePosition_t;

typedef enum {
    PC_ABSTRACT_ROD_NEW_RELAX = 0,
    PC_ABSTRACT_ROD_NEW_STANDBY = 1,
    PC_ABSTRACT_ROD_NEW_GRAB_HIGH = 2,
    PC_ABSTRACT_ROD_NEW_DOCK_WAIT = 3,
} PC_AbstractRodNewPosition_t;

typedef enum {
    PC_ABSTRACT_ORE_STORE_RELAX = 0,
    PC_ABSTRACT_ORE_STORE_HOME = 1,
    PC_ABSTRACT_ORE_STORE_STANDBY = 2,
    PC_ABSTRACT_ORE_STORE_MID_WAIT = 3,
    PC_ABSTRACT_ORE_STORE_LIFT = 4,
    PC_ABSTRACT_ORE_STORE_BUFFER = 5,
    PC_ABSTRACT_ORE_STORE_SPEARHEAD_PICKUP = 6,
} PC_AbstractOreStorePosition_t;

typedef enum {
    PC_ABSTRACT_POLE_RELAX = 0,
    PC_ABSTRACT_POLE_STEP_200_ALL_EXTEND = 1,
    PC_ABSTRACT_POLE_STEP_200_FRONT_RETRACT = 2,
    PC_ABSTRACT_POLE_STEP_200_ALL_RETRACT = 3,
    PC_ABSTRACT_POLE_STEP_200_SMALL = 4,
    PC_ABSTRACT_POLE_STEP_400_ALL_EXTEND = 5,
    PC_ABSTRACT_POLE_STEP_400_FRONT_RETRACT = 6,
    PC_ABSTRACT_POLE_STEP_400_ALL_RETRACT = 7,
    PC_ABSTRACT_POLE_ORE_RELEASE = 8,
} PC_AbstractPolePosition_t;

typedef struct {
    uint8_t enable_mask;               /* PC_AbstractModuleMask_t 位掩码 */
    uint8_t arm_simple_position;       /* PC_AbstractArmSimplePosition_t */
    uint8_t rod_new_position;          /* PC_AbstractRodNewPosition_t */
    uint8_t ore_store_position;        /* PC_AbstractOreStorePosition_t */
    uint8_t pole_position;             /* PC_AbstractPolePosition_t */
} PC_AbstractPositionCMD_t;

typedef enum {
    PC_AUTO_ACTION_NONE = 0,             /* 无一键动作 */
    PC_AUTO_ACTION_ABORT = 1,            /* 中止一键动作 */
    PC_AUTO_ACTION_STORE = 2,            /* 一键存矿 */
    PC_AUTO_ACTION_RELEASE = 3,          /* 一键放矿 */
    PC_AUTO_ACTION_RELEASE_LIFT_DETECT = 4, /* 一键放矿：Pole 到位后检测抬升再放矿 */
    PC_AUTO_ACTION_CHAMBER = 5,          /* 一键上膛 */
    PC_AUTO_ACTION_PICK_POS_400 = 6,     /* 一键取正 400mm 矿 */
    PC_AUTO_ACTION_PICK_POS_200 = 7,     /* 一键取正 200mm 矿 */
    PC_AUTO_ACTION_PICK_NEG_200 = 8,     /* 一键取反 200mm 矿 */
    PC_AUTO_ACTION_PICK_STORE_POS_400 = 9, /* 一键取正 400mm 矿后并行存矿和后退 */
    PC_AUTO_ACTION_PICK_STORE_POS_200 = 10, /* 一键取正 200mm 矿后并行存矿和后退 */
    PC_AUTO_ACTION_PICK_STORE_NEG_200 = 11, /* 一键取反 200mm 矿后并行存矿和后退 */
    PC_AUTO_ACTION_ROD_SPEARHEAD = 12,   /* 一键取矛头 */
    PC_AUTO_ACTION_ROD_SPEARHEAD_STEP1 = 13, /* 取矛头 step1：平台到位并张开待机 */
    PC_AUTO_ACTION_ROD_SPEARHEAD_STEP2 = 14, /* 取矛头 step2：夹取抬高并确认 */
    PC_AUTO_ACTION_ROD_DOCK_WAIT = 15,   /* 取矛v头机构等待对接 */
    PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS1 = 16, /* 取矛头位置1 SICK 校正 */
    PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS2 = 17, /* 取矛头位置2 SICK 校正 */
    PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS3 = 18, /* 取矛头位置3 SICK 校正 */
    PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS4 = 19, /* 取矛头位置4 SICK 校正 */
    PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS5 = 20, /* 取矛头位置5 SICK 校正 */ 
    PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD_POS6 = 21, /* 取矛头位置6 SICK 校正 */
    PC_AUTO_ACTION_SICK_CORRECT_ORE_RELEASE = 22, /* 放矿前 SICK 一键校正 */
    PC_AUTO_ACTION_STEP_ASCEND_200_HEAD = 23, /* 普通头向上 200mm 台阶 */
    PC_AUTO_ACTION_STEP_DESCEND_200_HEAD = 24, /* 普通头向下 200mm 台阶 */
    PC_AUTO_ACTION_STEP_ASCEND_400_HEAD = 25, /* 普通头向上 400mm 台阶 */
    PC_AUTO_ACTION_STEP_DESCEND_400_HEAD = 26, /* 普通头向下 400mm 台阶 */
    PC_AUTO_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD = 27, /* 融合取矿存矿并头向上 200mm 台阶 */
    PC_AUTO_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD = 28, /* 融合取矿存矿并头向下 200mm 台阶 */
    PC_AUTO_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD = 29, /* 融合取矿存矿并头向上 400mm 台阶 */
    PC_AUTO_ACTION_STEP_DROP_STORE_ASCEND_200_HEAD = 30, /* 丢矿版融合取矿存矿并头向上 200mm 台阶 */
    PC_AUTO_ACTION_STEP_DROP_STORE_DESCEND_200_HEAD = 31, /* 丢矿版融合取矿存矿并头向下 200mm 台阶 */
    PC_AUTO_ACTION_STEP_DROP_STORE_ASCEND_400_HEAD = 32, /* 丢矿版融合取矿存矿并头向上 400mm 台阶 */
    PC_AUTO_ACTION_RECOVER_STORE = 33,    /* 回收地面矿并存矿 */
    PC_AUTO_ACTION_RELEASE_STEP1 = 34,    /* 放矿 step1：竖直观察目标格，占矿结果上报后保持矿 */
    PC_AUTO_ACTION_RELEASE_STEP2 = 35,    /* 放矿 step2：确认目标格后继续释放矿 */
    PC_AUTO_ACTION_RELEASE_LIFT_DETECT_STEP1 = 36, /* 三层放矿 step1：竖直观察目标格并等待抬升检测 */
    PC_AUTO_ACTION_RELEASE_LIFT_DETECT_STEP2 = 37, /* 三层放矿 step2：确认目标格/抬升后继续释放矿 */
    PC_AUTO_ACTION_RELEASE_IR_LIFT_DETECT = 38, /* 三层放矿单事务：命令3允许放矿；命令4结束；PC CANCEL中止 */
} PC_AutoAction_t;

typedef enum {
    PC_AUTO_ACTION_RESULT_SUCCESS = 0,    /* 执行成功 */
    PC_AUTO_ACTION_RESULT_FAIL = 1,       /* 执行失败 */
} PC_AutoActionResult_t;

#define PC_AUTO_ACTION_FAILURE_SETUP (1u << 0)          /* 启动条件/参数/配置类失败 */
#define PC_AUTO_ACTION_FAILURE_PICK_ORE (1u << 1)       /* 取矿失败 */
#define PC_AUTO_ACTION_FAILURE_STORE_ORE (1u << 2)      /* 存矿失败 */
#define PC_AUTO_ACTION_FAILURE_RELEASE_ORE (1u << 3)    /* 放矿失败 */
#define PC_AUTO_ACTION_FAILURE_CHAMBER (1u << 4)        /* 上膛失败 */
#define PC_AUTO_ACTION_FAILURE_STEP (1u << 5)           /* 上台阶失败 */
#define PC_AUTO_ACTION_FAILURE_ROD_SPEARHEAD (1u << 6)  /* 取矛头失败 */
#define PC_AUTO_ACTION_FAILURE_ROD_DOCK_WAIT (1u << 7)  /* 矛头对接等待失败 */
#define PC_AUTO_ACTION_FAILURE_SICK_CORRECT (1u << 8)   /* SICK 校正失败 */
#define PC_AUTO_ACTION_FAILURE_ABORTED (1u << 9)        /* 人为中止 */

#define PC_AUTO_ACTION_SEGMENT_PICK (1u << 0)  /* 取矿/arm 交矿侧动作完成 */
#define PC_AUTO_ACTION_SEGMENT_STORE (1u << 1) /* 存矿机构侧动作完成 */
#define PC_AUTO_ACTION_SEGMENT_STEP (1u << 2)  /* 台阶/底盘侧动作完成 */

typedef struct {
    uint8_t action;    /* 一键动作类型，见 PC_AutoAction_t */
} PC_AutoActionCMD_t;

typedef enum {
    PC_AUTO_ACTION_V2_OP_NONE = 0,
    PC_AUTO_ACTION_V2_OP_START = 1,
    PC_AUTO_ACTION_V2_OP_CONTINUE = 2,
    PC_AUTO_ACTION_V2_OP_ABORT = 3,
    PC_AUTO_ACTION_V2_OP_ACK = 4,
} PC_AutoActionV2Operation_t;

typedef enum {
    PC_AUTO_ACTION_JOB_IDLE = 0,
    PC_AUTO_ACTION_JOB_ACCEPTED = 1,
    PC_AUTO_ACTION_JOB_RUNNING = 2,
    PC_AUTO_ACTION_JOB_WAIT_GATE = 3,
    PC_AUTO_ACTION_JOB_ABORTING = 4,
    PC_AUTO_ACTION_JOB_SUCCEEDED = 5,
    PC_AUTO_ACTION_JOB_FAILED = 6,
    PC_AUTO_ACTION_JOB_ABORTED = 7,
    PC_AUTO_ACTION_JOB_REJECTED = 8,
} PC_AutoActionJobState_t;

typedef enum {
    PC_AUTO_ACTION_REJECT_NONE = 0,
    PC_AUTO_ACTION_REJECT_INVALID_OPERATION = 1,
    PC_AUTO_ACTION_REJECT_INVALID_ACTION = 2,
    PC_AUTO_ACTION_REJECT_BUSY = 3,
    PC_AUTO_ACTION_REJECT_JOB_MISMATCH = 4,
    PC_AUTO_ACTION_REJECT_NOT_WAITING_GATE = 5,
    PC_AUTO_ACTION_REJECT_START_FAILED = 6,
    PC_AUTO_ACTION_REJECT_QUEUE_FULL = 7,
} PC_AutoActionRejectReason_t;

typedef enum {
    PC_AUTO_ACTION_GATE_NONE = 0,
    PC_AUTO_ACTION_GATE_RELEASE_STEP2 = 1,
} PC_AutoActionGate_t;

/* 精确线格式为 <HHBBBB：8 字节，小端。 */
typedef struct __attribute__((packed)) {
    uint16_t request_id;
    uint16_t job_id;
    uint8_t operation; /* PC_AutoActionV2Operation_t */
    uint8_t action;    /* START 时为 PC_AutoAction_t */
    uint8_t gate_id;   /* CONTINUE 时为 PC_AutoActionGate_t */
    uint8_t flags;
} PC_AutoActionV2CMD_t;

/* 精确线格式为 <HHBBBBBBHBB：14 字节，小端。 */
typedef struct __attribute__((packed)) {
    uint16_t request_id;
    uint16_t job_id;
    uint8_t action;
    uint8_t state; /* PC_AutoActionJobState_t */
    uint8_t required_mask;
    uint8_t running_mask;
    uint8_t completed_mask;
    uint8_t failed_mask;
    uint16_t failure_mask;
    uint8_t reject_reason; /* PC_AutoActionRejectReason_t */
    uint8_t active_node;
} PC_AutoActionV2Feedback_t;

typedef enum {
    PC_AUTO_ACTION_V3_OP_NONE = 0,
    PC_AUTO_ACTION_V3_OP_SUBMIT = 1,
    PC_AUTO_ACTION_V3_OP_CANCEL = 2,
    PC_AUTO_ACTION_V3_OP_CONTINUE = 3,
    PC_AUTO_ACTION_V3_OP_ACK = 4,
    PC_AUTO_ACTION_V3_OP_QUERY = 5,
} PC_AutoActionV3Operation_t;

typedef enum {
    PC_AUTO_ACTION_V3_JOB_FREE = 0,
    PC_AUTO_ACTION_V3_JOB_QUEUED = 1,
    PC_AUTO_ACTION_V3_JOB_WAIT_RESOURCE = 2,
    PC_AUTO_ACTION_V3_JOB_STARTING = 3,
    PC_AUTO_ACTION_V3_JOB_RUNNING = 4,
    PC_AUTO_ACTION_V3_JOB_WAIT_GATE = 5,
    PC_AUTO_ACTION_V3_JOB_CANCEL_REQUESTED = 6,
    PC_AUTO_ACTION_V3_JOB_SUCCEEDED = 7,
    PC_AUTO_ACTION_V3_JOB_FAILED = 8,
    PC_AUTO_ACTION_V3_JOB_CANCELLED = 9,
    PC_AUTO_ACTION_V3_JOB_REJECTED = 10,
} PC_AutoActionV3JobState_t;

typedef enum {
    PC_AUTO_ACTION_V3_BLOCK_NONE = 0,
    PC_AUTO_ACTION_V3_BLOCK_RESOURCE = 1,
    PC_AUTO_ACTION_V3_BLOCK_EXECUTOR = 2,
    PC_AUTO_ACTION_V3_BLOCK_DEPENDENCY = 3,
    PC_AUTO_ACTION_V3_BLOCK_MOTION_CONSTRAINT = 4,
    PC_AUTO_ACTION_V3_BLOCK_SENSOR = 5,
    PC_AUTO_ACTION_V3_BLOCK_EXTERNAL_GATE = 6,
} PC_AutoActionV3BlockedReason_t;

typedef enum {
    PC_AUTO_ACTION_V3_REJECT_NONE = 0,
    PC_AUTO_ACTION_V3_REJECT_INVALID_REQUEST = 1,
    PC_AUTO_ACTION_V3_REJECT_INVALID_ACTION = 2,
    PC_AUTO_ACTION_V3_REJECT_QUEUE_FULL = 3,
    PC_AUTO_ACTION_V3_REJECT_REQUEST_CONFLICT = 4,
    PC_AUTO_ACTION_V3_REJECT_JOB_NOT_FOUND = 5,
    PC_AUTO_ACTION_V3_REJECT_INVALID_STATE = 6,
    PC_AUTO_ACTION_V3_REJECT_START_FAILED = 7,
} PC_AutoActionV3RejectReason_t;

typedef struct __attribute__((packed)) {
    uint16_t request_id;
    uint16_t job_id;
    uint8_t operation; /* PC_AutoActionV3Operation_t */
    uint8_t action;    /* SUBMIT 时为 PC_AutoAction_t */
    uint8_t gate_id;
    uint8_t flags;
} PC_AutoActionV3CMD_t;

typedef struct __attribute__((packed)) {
    uint16_t request_id;
    uint16_t job_id;
    uint16_t failure_mask;
    uint8_t action;
    uint8_t state;
    uint8_t owned_resource_mask;
    uint8_t waiting_resource_mask;
    uint8_t running_segment_mask;
    uint8_t completed_segment_mask;
    uint8_t failed_segment_mask;
    uint8_t blocked_reason;
    uint8_t reject_reason;
} PC_AutoActionV3JobFeedback_t;

#define PC_AUTO_ACTION_V3_JOB_CAPACITY (4u)

typedef struct __attribute__((packed)) {
    uint16_t generation;
    uint8_t count;
    uint8_t capacity;
    PC_AutoActionV3JobFeedback_t jobs[PC_AUTO_ACTION_V3_JOB_CAPACITY];
} PC_AutoActionV3Feedback_t;

typedef struct __attribute__((packed)) {
    uint16_t request_id;
    uint16_t job_id;
    uint8_t operation;
    uint8_t action;
    uint8_t reject_reason;
    uint8_t reserved;
} PC_AutoActionV3RejectFeedback_t;

typedef enum {
    PC_R2_READY_STATE_NOT_READY = 0,  /* 上位机未准备完毕，对应灯效 0 号模式 */
    PC_R2_READY_STATE_READY = 1,      /* 上位机已准备完毕，对应灯效 1 号模式 */
    PC_R2_READY_STATE_RETRY = 2,      /* 上位机请求重试，对应灯效 2 号模式 */
    PC_R2_READY_STATE_FAIL = 3,       /* 上位机/流程失败，对应灯效 3 号模式 */
} PC_R2ReadyState_t;

typedef struct {
    uint8_t state;    /* 见 PC_R2ReadyState_t */
} PC_R2ReadyStateCMD_t;

typedef enum {
    PC_STEP_TEMPLATE_NONE = 0,              /* 无自动台阶流程 */
    PC_STEP_TEMPLATE_ASCEND_200_HEAD = 1,   /* 头向前上 200mm 台阶 */
    PC_STEP_TEMPLATE_ASCEND_400_HEAD = 2,   /* 头向前上 400mm 台阶 */
    PC_STEP_TEMPLATE_DESCEND_200_HEAD = 3,  /* 头向前下 200mm 台阶 */
    PC_STEP_TEMPLATE_DESCEND_400_HEAD = 4,  /* 头向前下 400mm 台阶 */
} PC_StepTemplate_t;

typedef enum {
    PC_STEP_DIR_HEAD_FORWARD = 0,  /* 头向前行进 */
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
    uint8_t transform_low_has_ore;    /* 变形机构低位占矿状态，0/1 */
    uint8_t transform_high_has_ore;   /* 变形机构高位占矿状态，0/1 */
    uint8_t arm_has_ore;              /* 机械臂持矿状态，0/1 */
    uint8_t release_grid_has_ore;     /* 放矿目标格占矿检测结果，0/1 */
} PC_OreStoreFeedback_t;

typedef struct {
    uint8_t mode;                    /* 右云台当前模式，见 CameraYaw_Mode_t */
    uint8_t motor_online;            /* 云台电机在线标志，0=离线，1=在线 */
    uint8_t feedback_valid;          /* PC yaw 命令是否仍在有效期内，0=超时/无效，1=有效 */
    uint8_t at_target;               /* 当前 yaw 是否到达目标，0=未到位，1=已到位 */
    float target_yaw_rad;            /* 当前控制目标 yaw，单位 rad */
    float feedback_yaw_rad;          /* 反馈 yaw，单位 rad */
    float error_yaw_rad;             /* target_yaw_rad - feedback_yaw_rad，单位 rad */
    float motor_angle_rad;           /* 6020 电机机械角/累计角反馈，单位 rad */
    float motor_velocity_rad_s;      /* 6020 电机速度反馈，单位 rad/s */
    float output;                    /* yaw 闭环输出量，单位取决于电机控制层 */
    uint32_t feedback_age_ms;        /* 距最近一次有效 PC yaw 命令的时间，单位 ms */
} PC_CameraYawFeedback_t;

typedef struct {
    uint8_t action;                 /* 当前/最近一键动作，见 PC_AutoAction_t */
    uint8_t busy;                   /* 任意一键动作是否正在执行，0=空闲，1=忙 */
    uint8_t finished;               /* 是否已有总动作结束结果，0=无，1=有 */
    uint8_t result;                 /* 结束结果，见 PC_AutoActionResult_t */
    uint16_t failure_mask;          /* 失败部位 bitmask，见 PC_AUTO_ACTION_FAILURE_* */
    uint8_t segment_finished_mask;  /* 分段完成 bit0=pick/arm交矿，bit1=store，bit2=step */
    uint8_t reserved;               /* 保留字段，发送端固定为 0 */
} PC_AutoActionFeedback_t;

typedef struct {
    uint8_t action;                  /* 当前 SICK 校正一键动作，见 PC_AutoAction_t；非取矛头校正时为 0 */
    uint8_t position_index;          /* 取矛头位置索引，0~5；无效时为 0xFF */
    uint8_t valid_mask;              /* bit0=x 字段有效，bit1=y 字段有效 */
    uint8_t reserved;                /* 保留字段，发送端固定为 0 */
    float x_target_adc;              /* X 方向 SICK 标准 ADC 值 */
    float x_sample_adc;              /* X 方向 SICK 实时 ADC 值 */
    float y_target_adc;              /* Y 方向 SICK 标准 ADC 值 */
    float y_sample_adc;              /* Y 方向 SICK 实时 ADC 值 */
} PC_SickCorrectFeedback_t;

#define PC_SICK_CORRECT_VALID_X (1u << 0)
#define PC_SICK_CORRECT_VALID_Y (1u << 1)

typedef struct {
    uint8_t detected;           /* 前 SICK 区域正方形矿检测结果，0=无矿，1=有矿 */
} PC_SickFrontOreFeedback_t;

typedef struct {
    uint32_t update_tick;       /* 最近一次 SICK 数据更新时间，单位 ms */
    float distance_mm[4];       /* 4 路 SICK 距离，单位 mm */
    uint16_t adc_raw[4];        /* 4 路 SICK 原始 ADC 值 */
    uint16_t miss_count;        /* 连续丢帧/失效计数 */
    uint8_t valid_mask;         /* 4 路有效位 bitmask，bit0~3 对应通道 0~3 */
    uint8_t reserved;           /* 保留字段，发送端固定为 0 */
} PC_SickRawFeedback_t;

typedef enum {
    PC_ORE_TYPE_UNKNOWN = 0,    /* 矿种未知或未收到 */
    PC_ORE_TYPE_R1 = 1,         /* R1 矿 */
    PC_ORE_TYPE_R2 = 2,         /* R2 矿 */
    PC_ORE_TYPE_FAKE = 3,       /* 假矿 */
} PC_OreType_t;

#define PC_IR_ORE_POSITION_COUNT (12u)
#define PC_IR_ORE_RAW_FRAME_SIZE (18u)
#define PC_IR_ORE_ACK_FRAME_SIZE (6u)

typedef struct {
    uint8_t frame[PC_IR_ORE_ACK_FRAME_SIZE];
} PC_IrOreAckCMD_t;

/* 旧红外矿种反馈帧：当前 R1/R2 对接新协议使用 PC_FEEDBACK_IR_DOCK。 */
typedef struct {
    uint8_t valid;                                           /* 矿种信息是否曾成功解析过一帧，0=未收到过 12 字节矿种包，1=有效 */
    uint8_t fresh;                                           /* 矿种信息是否新鲜，0=过期/未收到，1=在 IR_DOCK_ORE_INFO_FRESH_MS (1000ms) 内。注意：仅描述矿种包，不代表 status 状态命令 */
    uint8_t status;                                          /* 最近一次 1 字节状态命令，见 IrDock_Status_t (IDLE=0/DOCKING=1/DOCK_COMPLETE=2)，状态命令单独到达也会刷新此字段 */
    uint8_t count;                                           /* 矿位个数，固定等于 PC_IR_ORE_POSITION_COUNT (12) */
    uint8_t ore_type[PC_IR_ORE_POSITION_COUNT];              /* 12 个矿位种类，见 PC_OreType_t (UNKNOWN=0/R1=1/R2=2/FAKE=3)，下标为矿位编号 */
    uint32_t age_ms;                                         /* 距最近一次成功接收 12 字节矿种包的时间，单位 ms；未收到过为 0 */
    uint32_t rx_count;                                       /* 12 字节矿种包累计成功接收次数，CRC/格式错误不会计入；可用于观察对接链路健康度 */
} PC_IrOreFeedback_t;

typedef struct {
    uint8_t valid;                                           /* 是否曾成功解析 18 字节红外矿种帧，0=未成功，1=成功过 */
    uint8_t fresh;                                           /* 矿种帧是否新鲜，1 表示在 IR_DOCK_ORE_INFO_FRESH_MS 内收到 */
    uint8_t status;                                          /* 最近状态命令，见 IrDock_Status_t：0=IDLE，1=DOCKING，2=DOCK_COMPLETE */
    uint8_t count;                                           /* 矿位个数，固定为 PC_IR_ORE_POSITION_COUNT (12) */
    uint8_t msg_id;                                          /* 最近 18 字节矿种帧中的消息 ID，用于 ACK 匹配 */
    uint8_t side;                                            /* 最近 18 字节矿种帧中的侧别字段，0=左/默认，1=右，具体语义由红外对接端定义 */
    uint8_t ack_pending;                                     /* 红外对接端是否等待 ACK，0=否，1=是 */
    uint8_t parse_status;                                    /* 最近解析状态，见 IrDock_ParseStatus_t/IrDock_AckStatus_t */
    uint8_t ore_type[PC_IR_ORE_POSITION_COUNT];              /* 12 个矿位种类，见 PC_OreType_t，下标为矿位编号 */
    uint8_t raw_frame[PC_IR_ORE_RAW_FRAME_SIZE];             /* 最近一次 18 字节红外矿种原始帧 */
    uint32_t age_ms;                                         /* 距最近一次成功接收 18 字节矿种帧的时间，单位 ms */
    uint32_t rx_count;                                       /* 12 字节矿种包累计成功接收次数 */
    uint32_t frame_rx_count;                                 /* 18 字节红外矿种帧累计成功接收次数 */
    uint32_t ack_tx_count;                                   /* ACK 透传到红外 UART 的累计发送次数 */
} PC_IrOreBridgeFeedback_t;

typedef struct {
    uint8_t valid;                                           /* 是否曾成功解析过红外命令，0/1 */
    uint8_t fresh;                                           /* 最近合法命令是否仍在线，0/1 */
    uint8_t command;                                         /* 最近接收到的合法红外数据，1~4 */
} PC_IrDockFeedback_t;

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
    PC_CameraYawCMD_t camera_yaw;        /* 最近一次相机云台 yaw 命令 */
    PC_AbstractPositionCMD_t abstract_position; /* 最近一次抽象位置命令 */
    PC_AutoActionCMD_t auto_action;      /* 待消费的一键动作命令，消费后清零 */
    PC_StepCMD_t step;                   /* 待执行/最近一次自动台阶命令 */
    PC_ImuCMD_t imu;                     /* 最近一次 PC 姿态数据 */
    PC_IrOreAckCMD_t ir_ore_ack;             /* 待转发到红外对接 UART 的 ACK 原始帧 */
    PC_R2ReadyStateCMD_t r2_ready_state;     /* 最近一次 PC 下发的 R2 准备/重试状态 */
} MrlinkPc_CMD_Data_t;

typedef struct {
    PC_ChassisFeedback_t chassis;          /* 底盘反馈缓存 */
    PC_PoleFeedback_t pole;                /* 撑杆反馈缓存 */
    PC_ArmSimpleFeedback_t arm_simple;     /* 简易机械臂反馈缓存 */
    PC_RodNewFeedback_t rod_new;           /* 取矛头机构反馈缓存 */
    PC_OreStoreFeedback_t ore_store;          /* 矿仓反馈缓存 */
    PC_CameraYawFeedback_t camera_yaw;     /* 相机云台 yaw 反馈缓存 */
    PC_StepFeedback_t step;                /* 自动台阶反馈缓存 */
    PC_SickCorrectFeedback_t sick_correct; /* 取矛头 SICK 校正反馈缓存 */
    PC_SickFrontOreFeedback_t sick_front_ore; /* 前 SICK 区域正方形矿检测反馈缓存 */
    PC_SickRawFeedback_t sick_raw;         /* 4 路 SICK 原始数据反馈缓存 */
    PC_StatusFeedback_t status;            /* 通信/系统状态反馈缓存 */
    PC_IrDockFeedback_t ir_dock;           /* R1/R2 红外对接反馈缓存 */
    PC_IrOreBridgeFeedback_t ir_ore_bridge;  /* 红外对接桥接反馈缓存 */
} MrlinkPc_FeedbackData_t;

typedef struct {
    PC_ControlMode_t control_mode;       /* PC 控制模式，见 PC_ControlMode_t */
    bool heartbeat_valid;                /* 心跳有效标志，false=心跳超时 */
    uint32_t last_heartbeat_tick;        /* 最近一次心跳 tick，单位 ms */
    uint32_t last_recv_time;             /* 最近一次收到任意 PC 帧的 tick，单位 ms */
    uint32_t recv_count;                 /* 成功接收帧计数 */
    uint32_t error_count;                /* 通信解析错误计数 */
    uint32_t camera_yaw_cmd_tick;        /* 最近一次相机云台 yaw 命令 tick，单位 ms */
    bool online;                         /* PC 在线标志，false=离线，true=在线 */
    MrlinkPc_CMD_Data_t cmd;             /* PC 命令缓存 */
    MrlinkPc_FeedbackData_t feedback;    /* 反馈缓存 */
} MrlinkPc_State_t;

typedef struct {
    uint8_t usart10_config_ok;         /* PC UART(当前 USART1) 配置检查，0=异常，1=符合 PC 通信配置 */
    uint32_t usart10_baudrate;         /* PC UART(当前 USART1) 波特率 */
    uint32_t usart10_word_length;      /* PC UART(当前 USART1) 数据位配置，HAL UART_WORDLENGTH_* */
    uint32_t usart10_stop_bits;        /* PC UART(当前 USART1) 停止位配置，HAL UART_STOPBITS_* */
    uint32_t usart10_parity;           /* PC UART(当前 USART1) 校验位配置，HAL UART_PARITY_* */
    uint32_t usart10_mode;             /* PC UART(当前 USART1) 收发模式配置，HAL UART_MODE_* */

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
    uint8_t pole_cmd_recent;           /* 最近是否收到有效撑杆命令，0=否，1=是 */
    uint8_t arm_simple_cmd_recent;     /* 最近是否收到有效简易机械臂命令，0=否，1=是 */
    uint32_t pole_cmd_tick;            /* 最近一次撑杆命令 tick，单位 ms */
    uint32_t pole_cmd_age_ms;          /* 最近一次撑杆命令距当前时间，单位 ms */
    uint32_t arm_simple_cmd_tick;      /* 最近一次简易机械臂命令 tick，单位 ms */
    uint32_t arm_simple_cmd_age_ms;    /* 最近一次简易机械臂命令距当前时间，单位 ms */
    uint32_t rx_rod_new_count;         /* 收到取矛头机构命令次数 */
    uint32_t rx_ore_store_count;       /* 收到矿仓命令次数 */
    uint32_t rx_auto_action_count;     /* 收到一键动作命令次数 */
    uint32_t rx_camera_yaw_count;      /* 收到相机云台 yaw 命令次数 */
    uint32_t rx_abstract_position_count; /* 收到抽象位置命令次数 */
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

    uint8_t last_rx_cmd;               /* 最近一次接收帧命令字 */
    uint8_t last_rx_len;               /* 最近一次接收帧 payload 长度 */
    int8_t last_rx_result;             /* 最近一次接收处理结果，MRLINK_OK/错误码 */
    uint16_t last_rx_frame_size;       /* 最近一次接收帧完整长度 */
    uint16_t last_rx_crc_received;     /* 最近一次接收帧携带的 CRC */
    uint16_t last_rx_crc_calculated;   /* 最近一次接收帧计算出的 CRC */

    PC_ChassisCMD_t rx_chassis;             /* 调试用：最近一次底盘命令 */
    PC_PoleCMD_t rx_pole;                   /* 调试用：最近一次撑杆命令 */
    PC_ArmSimpleCMD_t rx_arm_simple;        /* 调试用：最近一次简易机械臂命令 */
    PC_RodNewCMD_t rx_rod_new;              /* 调试用：最近一次取矛头机构命令 */
    PC_OreStoreCMD_t rx_ore_store;          /* 调试用：最近一次矿仓命令 */
    PC_CameraYawCMD_t rx_camera_yaw;        /* 调试用：最近一次相机云台 yaw 命令 */
    PC_AbstractPositionCMD_t rx_abstract_position; /* 调试用：最近一次抽象位置命令 */
    PC_AutoActionCMD_t rx_auto_action;      /* 调试用：最近一次一键动作命令 */
    PC_StepCMD_t rx_step;                   /* 调试用：最近一次自动台阶命令 */
    PC_ImuCMD_t rx_imu;                     /* 调试用：最近一次 PC 姿态数据 */
    PC_R2ReadyStateCMD_t rx_r2_ready_state; /* 调试用：最近一次 R2 准备/重试状态 */
    uint32_t rx_r2_ready_state_count;        /* 收到 PC_CMD_R2_READY_STATE 的次数 */
    uint8_t r2_ready_light_mode;             /* 最近一次 R2 状态映射到的灯效模式 */

    uint32_t tx_count;                /* 反馈发送尝试次数 */
    uint8_t tx_dma_busy;              /* 发送 DMA 忙标志，0=空闲，1=忙 */
    uint32_t tx_busy_skip_count;      /* 因发送 DMA 忙而跳过发送的次数 */
    uint32_t tx_dma_error_count;      /* 发送 DMA 错误/超时次数 */
    uint16_t tx_len;                  /* 最近一次发送总长度 */
    uint8_t tx_frame_count;           /* 最近一次发送中打包的反馈帧数量 */
    int8_t tx_result;                 /* 最近一次发送结果，MRLINK_CHANNEL_OK/错误码 */

    MrLink_Stats_t mrlink_stats;      /* mrlink 协议层统计快照 */
    uint32_t rx_ir_ore_ack_count;              /* 收到 PC_CMD_IR_ORE_ACK 的次数 */
    uint32_t ir_ore_ack_submit_count;          /* ACK 成功提交到红外 UART 发送队列的次数 */
    uint32_t ir_ore_ack_submit_error_count;    /* ACK 提交失败次数，不含 BUSY 重试 */
    uint8_t ir_ore_ack_submit_result;          /* 最近一次 ACK 提交结果；0xFF 表示当前固件未启用旧 ACK 透传 */
    PC_IrOreAckCMD_t rx_ir_ore_ack;            /* 调试用：最近一次 PC 下发的 ACK 原始帧 */
} PC_CommDebug_t;

typedef struct {
    auto_ctrl_template_e template_id;       /* 映射后的自动台阶模板 */
    auto_ctrl_travel_dir_e travel_dir;      /* 映射后的行进方向 */
    float target_yaw_rad;                   /* 自动台阶目标航向角，单位 rad */
    float yaw_tolerance_rad;                /* 自动台阶航向允许误差，单位 rad */
} PC_AutoStepParams_t;

extern volatile PC_CommDebug_t g_pc_comm_debug;

typedef void (*MrlinkPc_TxCallback_t)(void); /* 发送完成/错误回调函数类型，无参数 */

/* 在通信初始化前选择 PC 通信使用的底层 UART；uart 见 BSP_UART_t。 */
bool MrlinkPc_SelectUart(BSP_UART_t uart);

/* 初始化 mrlink PC 通信；成功返回 true，失败原因记录到 g_pc_comm_debug.last_init_error。 */
bool MrlinkPc_CommInit(void);

/* 周期处理接收、解析、在线状态；now_ms 为当前系统毫秒时间戳。 */
void MrlinkPc_CommProcess(uint32_t now_ms);

/* 返回 PC 是否在线，在线条件由心跳超时逻辑维护。 */
bool MrlinkPc_CommIsOnline(void);

/* 获取 PC 通信内部状态快照指针，只读使用。 */
const MrlinkPc_State_t *MrlinkPc_GetState(void);

/* 将内部状态同步到 g_pc_comm_debug，供 Ozone/调试器查看。 */
void MrlinkPc_DebugUpdate(void);

/* 返回当前是否处于 PC 控制模式。 */
bool MrlinkPc_IsPCControlMode(void);

/* 返回 PC 心跳是否仍在有效期内。 */
bool MrlinkPc_IsHeartbeatValid(void);

/* 获取最近一次 PC 底盘命令。 */
const PC_ChassisCMD_t *MrlinkPc_GetChassisCMD(void);
/* true only when a chassis frame was received within the last 200 ms. */
bool MrlinkPc_HasChassisCMD(void);

/* 获取最近一次 PC 撑杆命令。 */
const PC_PoleCMD_t *MrlinkPc_GetPoleCMD(void);

bool MrlinkPc_HasPoleCMD(void);

/* 获取最近一次 PC 机械臂笛卡尔命令，当前保留。 */
const PC_ArmCMD_t *MrlinkPc_GetArmCMD(void);

/* 获取最近一次 PC 简易机械臂命令。 */
const PC_ArmSimpleCMD_t *MrlinkPc_GetArmSimpleCMD(void);

bool MrlinkPc_HasArmSimpleCMD(void);

/* 获取最近一次 PC 取矛头机构命令。 */
const PC_RodNewCMD_t *MrlinkPc_GetRodNewCMD(void);

bool MrlinkPc_HasRodNewCMD(void);

/* 获取最近一次 PC 矿仓命令。 */
const PC_OreStoreCMD_t *MrlinkPc_GetOreStoreCMD(void);

bool MrlinkPc_HasOreStoreCMD(void);

/* 获取最近一次 PC 相机云台 yaw 命令。 */
const PC_CameraYawCMD_t *MrlinkPc_GetCameraYawCMD(void);

/* 获取最近一次 PC 抽象位置命令；未启用任何模块时返回 NULL。 */
const PC_AbstractPositionCMD_t *MrlinkPc_GetAbstractPositionCMD(void);

/* 获取待消费的一键动作命令；无待消费命令时返回 NULL。 */
const PC_AutoActionCMD_t *MrlinkPc_GetAutoActionCMD(void);

/* 获取待消费的 V2 事务命令；无待消费命令时返回 NULL。 */
const PC_AutoActionV2CMD_t *MrlinkPc_GetAutoActionV2CMD(void);
const PC_AutoActionV3CMD_t *MrlinkPc_GetAutoActionV3CMD(void);

/* 获取最近一次 PC 自动台阶命令。 */
const PC_StepCMD_t *MrlinkPc_GetStepCMD(void);

/* 获取最近一次 PC 姿态/IMU 命令。 */
const PC_ImuCMD_t *MrlinkPc_GetImuCMD(void);

/* 获取待转发的红外对接 ACK 命令；无待转发命令时返回 NULL。 */
const PC_IrOreAckCMD_t *MrlinkPc_GetIrOreAckCMD(void);

/* 获取最近一次 PC 下发的 R2 准备/重试状态。 */
const PC_R2ReadyStateCMD_t *MrlinkPc_GetR2ReadyStateCMD(void);

/* 最近一次 PC_CMD_R2_READY_STATE 是否为 READY。 */
bool MrlinkPc_IsR2Ready(void);

/* 最近一次 PC_CMD_R2_READY_STATE 接收 tick，单位 ms；未收到过时为 0。 */
uint32_t MrlinkPc_GetR2ReadyStateTickMs(void);

/* 发布某个反馈 topic 的最新数据；topic 见 PC_FeedbackCMD_t，feedback 指向对应反馈结构体。 */
bool MrlinkPc_PublishFeedback(uint8_t topic, const void *feedback);

/* 将 PC_StepCMD_t 映射为 AutoCtrl 可直接使用的台阶参数；无有效模板时返回 NULL。 */
const PC_AutoStepParams_t *MrlinkPc_GetAutoStepParams(void);

/* 清除待执行自动台阶命令，通常在成功启动流程后调用。 */
void MrlinkPc_ClearStepCommand(void);

/* 清除待消费一键动作命令，通常在 pc_comm_task 分发后调用。 */
void MrlinkPc_ClearAutoActionCommand(void);

void MrlinkPc_ClearAutoActionV2Command(void);
void MrlinkPc_ClearAutoActionV3Command(void);

/* 清除待转发红外对接 ACK 命令，通常在 ACK 成功提交或不可恢复失败后调用。 */
void MrlinkPc_ClearIrOreAckCommand(void);

/* 请求向 PC 发送一次开始比赛命令；start 非 0 表示开始。 */
bool MrlinkPc_RequestStartMatch(uint8_t start);

/* 返回是否有待发送的开始比赛命令。 */
bool MrlinkPc_HasStartMatchRequest(void);

/* 清除待发送的开始比赛命令，通常在发送成功后调用。 */
void MrlinkPc_ClearStartMatchRequest(void);

/* 构造开始比赛命令 mrlink 帧；无待发送命令或缓存不足时返回 0。 */
uint16_t MrlinkPc_BuildStartMatchFrame(uint8_t *tx_buf, uint16_t buf_size);

/* 请求向 PC 发送一次重试命令；retry 非 0 表示需要启动重试逻辑。 */
bool MrlinkPc_RequestRetry(uint8_t retry);

/* 返回是否有待发送的重试命令。 */
bool MrlinkPc_HasRetryRequest(void);

/* 清除待发送的重试命令，通常在发送成功后调用。 */
void MrlinkPc_ClearRetryRequest(void);

/* 构造重试命令 mrlink 帧；无待发送命令或缓存不足时返回 0。 */
uint16_t MrlinkPc_BuildRetryFrame(uint8_t *tx_buf, uint16_t buf_size);

/* 构造指定反馈 cmd 的 mrlink 帧；tx_buf 为输出缓存，buf_size 为缓存字节数，返回帧长度。 */
uint16_t MrlinkPc_BuildFeedbackFrame(uint8_t cmd, uint8_t *tx_buf,
                                     uint16_t buf_size);

/* 构造指定 cmd/payload 的 mrlink 帧；用于向 PC 回传非反馈 topic 的当前命令镜像。 */
uint16_t MrlinkPc_BuildPayloadFrame(uint8_t cmd, const void *payload,
                                    uint16_t payload_len, uint8_t *tx_buf,
                                    uint16_t buf_size);

/* 发送已经打包好的 mrlink 数据；data 为帧缓存，len 为待发送字节数。 */
int8_t MrlinkPc_SendFrame(uint8_t *data, uint16_t len);

/* 注册底层发送完成/错误回调；tx_done 为成功回调，tx_error 为错误回调。 */
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
