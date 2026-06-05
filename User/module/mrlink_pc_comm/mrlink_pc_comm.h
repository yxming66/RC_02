#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "device/mrlink/mrlink_channel.h"
#include "module/autoCtrlAPI/core/auto_ctrl_def.h"
#include "mrlink/mrlink.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * PC 通信协议说明：
 * - 底层使用 MrLink 帧：0x4D 0x52 + payload_len + cmd + payload + CRC16(低字节在前)。
 * - PC_CMD_* 是 PC 发给 STM32 的 cmd 字节，PC_FEEDBACK_* 是 STM32 回传给 PC 的 cmd 字节。
 * - 本文件里的 PC_* 枚举值是协议固定值，PC 端必须按这些数值解析。
 * - C enum 的存储大小由编译器决定，不要直接把含 enum 字段的 STM32 内部结构体当串口 payload。
 * - 实际串口 payload 中需要压缩为 uint8_t 的结构体，见 pc_messages.hpp 里的 packed wire 类型。
 */

/* PC 控制模式，表示当前通信层是否认为 PC 正在接管控制。 */
typedef enum {
    PC_MODE_RC = 0,    /* 遥控器控制模式。心跳超时或未进入 PC 控制时使用。 */
    PC_MODE_PC = 1,    /* PC 控制模式。收到有效 PC 帧后置位，心跳超时后回到 RC。 */
} PC_ControlMode_t;

/* 当前整车命令来源，用于上位机显示 RC/PC 哪一侧正在生效。 */
typedef enum {
    PC_COMMAND_SOURCE_RC = 0,    /* 当前执行命令主要来自遥控器。 */
    PC_COMMAND_SOURCE_PC = 1,    /* 当前执行命令主要来自 PC。 */
} PC_CommandSource_t;

/* PC 发给 STM32 的 MrLink cmd 字节。 */
typedef enum {
    PC_CMD_HEARTBEAT = 0x01,     /* 心跳帧，payload 长度为 0；用于保持 PC 在线和进入 PC 控制模式。 */
    PC_CMD_CHASSIS = 0x10,       /* 底盘速度控制命令，payload 为 PC_ChassisCMD_t。 */
    PC_CMD_POLE = 0x11,          /* 撑杆控制命令，payload 为 pc_comm::wire::PoleCmd。 */
    PC_CMD_STEP = 0x12,          /* 自动上下台阶命令，payload 为 pc_comm::wire::StepCmd。 */
    PC_CMD_ARM_SIMPLE = 0x13,    /* 简易机械臂控制命令，payload 为 pc_comm::wire::ArmSimpleCmd。 */
    PC_CMD_ROD_NEW = 0x14,       /* 取矛头机构控制命令，payload 为 pc_comm::wire::RodNewCmd。 */
    PC_CMD_ORE_STORE = 0x15,     /* 矿仓平台控制命令，payload 为 pc_comm::wire::OreStoreCmd。 */
    PC_CMD_AUTO_ACTION = 0x16,   /* 一键取矿/存矿/上膛/放矿/取矛头命令，payload 为 pc_comm::wire::AutoActionCmd。 */
    PC_CMD_IMU = 0x20,           /* PC 下发姿态数据命令，payload 为 PC_ImuCMD_t。 */
} PC_CMD_t;

/* STM32 回传给 PC 的 MrLink cmd 字节。 */
typedef enum {
    PC_FEEDBACK_HEARTBEAT = 0x81,     /* STM32 心跳反馈，payload 长度为 0。 */
    PC_FEEDBACK_CHASSIS = 0x90,       /* 底盘速度反馈，payload 为 PC_ChassisFeedback_t。 */
    PC_FEEDBACK_POLE = 0x91,          /* 撑杆位置/电机角度反馈，payload 为 PC_PoleFeedback_t。 */
    PC_FEEDBACK_STEP = 0x92,          /* 自动上下台阶流程反馈，payload 为 pc_comm::wire::StepFeedback。 */
    PC_FEEDBACK_ARM_SIMPLE = 0x93,    /* 简易机械臂反馈，payload 为 pc_comm::wire::ArmSimpleFeedback。 */
    PC_FEEDBACK_ROD_NEW = 0x94,       /* 取矛头机构反馈，payload 为 pc_comm::wire::RodNewFeedback。 */
    PC_FEEDBACK_ORE_STORE = 0x95,     /* 矿仓平台反馈，payload 为 pc_comm::wire::OreStoreFeedback。 */
    PC_FEEDBACK_STATUS = 0xA0,        /* 通信/系统状态反馈，payload 为 pc_comm::wire::StatusFeedback。 */
    PC_FEEDBACK_AUTO_ACTION = 0x96,   /* 一键动作反馈，payload 为 PC_AutoActionFeedback_t。 */
    PC_FEEDBACK_IR_ORE = 0x97,        /* 红外对接矿种反馈，payload 为 PC_IrOreFeedback_t。 */
} PC_FeedbackCMD_t;

#define MRLINK_PC_MAX_PAYLOAD_SIZE (64u)  /* PC 通信单帧最大 payload 字节数。 */
#define MRLINK_PC_MAX_FRAME_SIZE (2u + 1u + 1u + MRLINK_PC_MAX_PAYLOAD_SIZE + 2u)
#define PC_COMM_DEBUG_RX_RAW_SIZE (256u)  /* 调试缓存：最近一批 RX 原始字节最大保存长度。 */
#define PC_COMM_DEBUG_TX_RAW_SIZE (96u)   /* 调试缓存：最近一批 TX 原始字节最大保存长度。 */

#ifndef MRLINK_PC_RX_DMA_SLOT_COUNT
#define MRLINK_PC_RX_DMA_SLOT_COUNT (4u)  /* RX DMA 双缓冲/多槽数量，至少为 2。 */
#endif

#ifndef MRLINK_PC_RX_DMA_BUF_SIZE
#define MRLINK_PC_RX_DMA_BUF_SIZE (256u)  /* 每个 RX DMA 槽的字节数，必须能容纳一个完整 MrLink 帧。 */
#endif

#ifndef MRLINK_PC_RX_STREAM_BUF_SIZE
#define MRLINK_PC_RX_STREAM_BUF_SIZE (512u)  /* MrLink RX 流缓存大小。 */
#endif

/* 底盘速度命令 payload，坐标系由底盘模块定义。 */
typedef struct {
    float vx;    /* x 方向目标速度，单位 m/s。正方向按底盘模块约定。 */
    float vy;    /* y 方向目标速度，单位 m/s。正方向按底盘模块约定。 */
    float wz;    /* z 轴目标角速度，单位 rad/s。正方向按底盘模块约定。 */
} PC_ChassisCMD_t;

/*
 * 撑杆命令内部缓存。
 * 注意：实际 PC_CMD_POLE 串口 payload 是 pc_comm::wire::PoleCmd，避免 C 结构体 padding 影响。
 */
typedef struct {
    uint8_t mode;      /* 撑杆模式，见 Pole_Mode_t：0=RELAX，1=ACTIVE。 */
    float lift[2];     /* 撑杆目标速度/高度输入，[0]=前组，[1]=后组；单位/含义由 Pole 模块控制模式决定。 */
} PC_PoleCMD_t;

/*
 * 旧机械臂笛卡尔命令缓存。
 * 当前 PC 通信主链路未订阅此 payload，保留给历史接口，不建议 PC 端继续使用。
 */
typedef struct {
    float y;        /* 机械臂笛卡尔 y 目标或速度，当前保留。 */
    float z;        /* 机械臂笛卡尔 z 目标或速度，当前保留。 */
    float pitch;    /* 机械臂末端 pitch 目标或速度，单位 rad，当前保留。 */
} PC_ArmCMD_t;

/*
 * 简易机械臂命令内部缓存。
 * 注意：实际 PC_CMD_ARM_SIMPLE 串口 payload 是 pc_comm::wire::ArmSimpleCmd。
 */
typedef struct {
    uint8_t mode;                /* 简易机械臂模式，见 ArmSimple_Mode_t：0=RELAX，1=JOINT，2=POS_VEL 兼容模式。 */
    uint8_t point_mode;          /* 预设点位模式，见 ArmSimple_PointMode_t；NONE 表示使用下面两个目标角。 */
    uint8_t suction;             /* 吸盘控制，见 Suction_State_t：0=关闭，1=开启。 */
    float target_joint1_rad;     /* 关节 1 目标角度，单位 rad。 */
    float target_joint2_rad;     /* 关节 2 目标角度，单位 rad。 */
} PC_ArmSimpleCMD_t;

/*
 * 取矛头机构命令内部缓存。
 * 注意：实际 PC_CMD_ROD_NEW 串口 payload 是 pc_comm::wire::RodNewCmd。
 */
typedef struct {
    uint8_t mode;              /* 取矛头机构模式，见 RodNew_Mode_t：0=RELAX，1=ACTIVE。 */
    uint8_t pose;              /* 取矛头机构姿态，见 RodNew_Pose_t：0=STANDBY，1=GRAB_HIGH，2=DOCK_WAIT，3=MANUAL。 */
    uint8_t grip;              /* 夹爪状态，见 RodNew_GripState_t：0=松开，1=夹紧。 */
    float target_angle_rad;    /* MANUAL 姿态下的舵机目标角度，单位 rad。 */
} PC_RodNewCMD_t;

/*
 * 矿仓命令内部缓存。
 * 注意：实际 PC_CMD_ORE_STORE 串口 payload 是 pc_comm::wire::OreStoreCmd。
 */
typedef struct {
    uint8_t mode;                  /* 矿仓模式，见 OreStore_Mode_t：0=RELAX，1=HOME，2=ACTIVE。 */
    uint8_t force_rehome;          /* 强制重新回零触发，0=不触发，1=触发一次。 */
    float platform_target_rad;     /* 矿仓平台轴目标位置，单位 rad。 */
} PC_OreStoreCMD_t;

/* PC 下发的一键动作类型，也是反馈中 action/ore_action 的取值。 */
typedef enum {
    PC_AUTO_ACTION_NONE = 0,             /* 无动作。 */
    PC_AUTO_ACTION_STORE = 1,            /* 一键存矿，将机械臂上的矿放入矿仓空位。 */
    PC_AUTO_ACTION_RELEASE = 2,          /* 一键放矿，将矿仓/机械臂矿释放到指定位置。 */
    PC_AUTO_ACTION_CHAMBER = 3,          /* 一键上膛，将矿仓中的矿转移到机械臂/发射准备位。 */
    PC_AUTO_ACTION_PICK_POS_400 = 4,     /* 一键取正方向 400mm 矿。 */
    PC_AUTO_ACTION_PICK_POS_200 = 5,     /* 一键取正方向 200mm 矿。 */
    PC_AUTO_ACTION_PICK_NEG_200 = 6,     /* 一键取反方向 200mm 矿。 */
    PC_AUTO_ACTION_ROD_SPEARHEAD = 7,    /* 一键取矛头。 */
    PC_AUTO_ACTION_ABORT = 8,            /* 中止当前一键动作。 */
} PC_AutoAction_t;

/* 一键动作所属子系统，用于 PC 判断当前反馈来自矿仓流程还是取矛头流程。 */
typedef enum {
    PC_AUTO_ACTION_SUBSYSTEM_NONE = 0,            /* 当前没有有效子系统或尚未执行过一键动作。 */
    PC_AUTO_ACTION_SUBSYSTEM_ORE = 1,             /* 矿仓/取矿/存矿/上膛子系统。 */
    PC_AUTO_ACTION_SUBSYSTEM_ROD_SPEARHEAD = 2,   /* 取矛头子系统。 */
} PC_AutoActionSubsystem_t;

/* 一键动作状态机状态。 */
typedef enum {
    PC_AUTO_ACTION_STATE_IDLE = 0,       /* 空闲，未运行。 */
    PC_AUTO_ACTION_STATE_RUNNING = 1,    /* 正在执行。 */
    PC_AUTO_ACTION_STATE_SUCCESS = 2,    /* 已成功结束。 */
    PC_AUTO_ACTION_STATE_FAIL = 3,       /* 已失败结束，fault 字段给出原因。 */
    PC_AUTO_ACTION_STATE_ABORT = 4,      /* 被外部中止。 */
} PC_AutoActionState_t;

/* 一键动作结果，描述当前或最近一次执行结果。 */
typedef enum {
    PC_AUTO_ACTION_RESULT_NONE = 0,       /* 无结果，通常表示尚未启动。 */
    PC_AUTO_ACTION_RESULT_RUNNING = 1,    /* 运行中。 */
    PC_AUTO_ACTION_RESULT_SUCCESS = 2,    /* 执行成功。 */
    PC_AUTO_ACTION_RESULT_FAIL = 3,       /* 执行失败。 */
    PC_AUTO_ACTION_RESULT_ABORTED = 4,    /* 被中止。 */
} PC_AutoActionResult_t;

/* 一键动作故障码。 */
typedef enum {
    PC_AUTO_ACTION_FAULT_NONE = 0,                  /* 无故障。 */
    PC_AUTO_ACTION_FAULT_INVALID_OCCUPANCY = 1,     /* 占矿状态不满足动作前置条件。 */
    PC_AUTO_ACTION_FAULT_INVALID_PARAM = 2,         /* 参数非法或控制对象未初始化。 */
    PC_AUTO_ACTION_FAULT_NOT_HOMED = 3,             /* 需要回零的机构尚未回零。 */
    PC_AUTO_ACTION_FAULT_TIMEOUT = 4,               /* 流程步骤或整体执行超时。 */
    PC_AUTO_ACTION_FAULT_ABORTED = 5,               /* 外部中止导致结束。 */
    PC_AUTO_ACTION_FAULT_NO_SPEARHEAD = 6,          /* 取矛头检测不到矛头或取矛头失败。 */
} PC_AutoActionFault_t;

#define PC_AUTO_ACTION_FLAG_ORE_INITED (1u << 0)  /* 矿仓一键子系统已初始化。 */
#define PC_AUTO_ACTION_FLAG_ORE_BUSY (1u << 1)    /* 矿仓一键子系统正在运行。 */
#define PC_AUTO_ACTION_FLAG_ROD_INITED (1u << 2)  /* 取矛头一键子系统已初始化。 */
#define PC_AUTO_ACTION_FLAG_ROD_BUSY (1u << 3)    /* 取矛头一键子系统正在运行。 */

/* 一键动作命令 payload。 */
typedef struct {
    uint8_t action;    /* 一键动作类型，见 PC_AutoAction_t。 */
} PC_AutoActionCMD_t;

/* 自动上下台阶模板。HEAD/TAIL 表示车头/车尾朝前，ASCEND/DESCEND 表示上/下台阶。 */
typedef enum {
    PC_STEP_TEMPLATE_NONE = 0,              /* 无自动台阶流程。 */
    PC_STEP_TEMPLATE_ASCEND_200_HEAD = 1,   /* 车头向前上 200mm 台阶。 */
    PC_STEP_TEMPLATE_ASCEND_400_HEAD = 2,   /* 车头向前上 400mm 台阶。 */
    PC_STEP_TEMPLATE_DESCEND_200_HEAD = 3,  /* 车头向前下 200mm 台阶。 */
    PC_STEP_TEMPLATE_DESCEND_400_HEAD = 4,  /* 车头向前下 400mm 台阶。 */
    PC_STEP_TEMPLATE_ASCEND_200_TAIL = 5,   /* 车尾向前上 200mm 台阶。 */
    PC_STEP_TEMPLATE_ASCEND_400_TAIL = 6,   /* 车尾向前上 400mm 台阶。 */
    PC_STEP_TEMPLATE_DESCEND_200_TAIL = 7,  /* 车尾向前下 200mm 台阶。 */
    PC_STEP_TEMPLATE_DESCEND_400_TAIL = 8,  /* 车尾向前下 400mm 台阶。 */
} PC_StepTemplate_t;

/* 自动台阶行进方向。 */
typedef enum {
    PC_STEP_DIR_HEAD_FORWARD = 0,  /* 车头为前进方向。 */
    PC_STEP_DIR_TAIL_FORWARD = 1,  /* 车尾为前进方向。 */
} PC_StepDir_t;

/*
 * 自动上下台阶命令内部缓存。
 * 注意：实际 PC_CMD_STEP 串口 payload 是 pc_comm::wire::StepCmd，template_id/travel_dir 为 uint8_t。
 */
typedef struct {
    PC_StepTemplate_t template_id;   /* 自动台阶模板，见 PC_StepTemplate_t。 */
    PC_StepDir_t travel_dir;         /* 行进方向，见 PC_StepDir_t。 */
    float target_yaw_rad;            /* 目标航向角，单位 rad。 */
    float yaw_tolerance_rad;         /* 航向允许误差，单位 rad；进入流程前用于对齐判定。 */
} PC_StepCMD_t;

/* PC 下发的姿态数据 payload。四元数和欧拉角均由 PC 端解算并下发。 */
typedef struct {
    float qw;       /* 四元数 w。 */
    float qx;       /* 四元数 x。 */
    float qy;       /* 四元数 y。 */
    float qz;       /* 四元数 z。 */
    float roll;     /* 横滚角，单位 rad。 */
    float pitch;    /* 俯仰角，单位 rad。 */
    float yaw;      /* 航向角，单位 rad。自动流程使用 PC yaw 时读取此字段。 */
} PC_ImuCMD_t;

/* 底盘速度反馈 payload。 */
typedef struct {
    float vx;    /* 底盘 x 方向当前速度，单位 m/s。 */
    float vy;    /* 底盘 y 方向当前速度，单位 m/s。 */
    float wz;    /* 底盘当前角速度，单位 rad/s。 */
} PC_ChassisFeedback_t;

/* 撑杆反馈 payload。 */
typedef struct {
    float lift[2];                /* 前后两组撑杆当前位置/角度，[0]=前组，[1]=后组，单位 rad。 */
    float motor_total_angle[4];   /* 4 个撑杆电机累计角度，单位 rad；电机顺序与 Pole 模块一致。 */
} PC_PoleFeedback_t;

/* 简易机械臂反馈内部格式，实际回传 payload 见 pc_comm::wire::ArmSimpleFeedback。 */
typedef struct {
    uint8_t mode;                  /* 当前简易机械臂模式，见 ArmSimple_Mode_t。 */
    uint8_t point_mode;            /* 当前点位模式，见 ArmSimple_PointMode_t。 */
    uint8_t suction;               /* 吸盘状态，见 Suction_State_t：0=关闭，1=开启。 */
    float joint1_angle_rad;        /* 关节 1 当前角度，单位 rad。 */
    float joint1_velocity_rad_s;   /* 关节 1 当前速度，单位 rad/s。 */
    float joint2_angle_rad;        /* 关节 2 当前角度，单位 rad。 */
} PC_ArmSimpleFeedback_t;

/* 取矛头机构反馈内部格式，实际回传 payload 见 pc_comm::wire::RodNewFeedback。 */
typedef struct {
    uint8_t mode;                    /* 当前取矛头机构模式，见 RodNew_Mode_t。 */
    uint8_t pose;                    /* 当前目标姿态，见 RodNew_Pose_t。 */
    uint8_t grip;                    /* 当前夹爪状态，见 RodNew_GripState_t。 */
    uint8_t at_target;               /* 舵机到位标志，0=未到位，1=已到位。 */
    float target_angle_rad;          /* 舵机目标角度，单位 rad。 */
    float tracked_angle_rad;         /* 速度/加速度限幅后的跟踪目标角度，单位 rad。 */
    float tracked_velocity_rad_s;    /* 限幅后的跟踪目标速度，单位 rad/s。 */
    float feedback_angle_rad;        /* 舵机反馈角度，单位 rad。 */
} PC_RodNewFeedback_t;

/* 矿仓反馈内部格式，实际回传 payload 见 pc_comm::wire::OreStoreFeedback。 */
typedef struct {
    uint8_t mode;                    /* 当前矿仓模式，见 OreStore_Mode_t。 */
    uint8_t all_homed;               /* 全部轴回零标志，0=未全部回零，1=全部已回零。 */
    uint8_t online_mask;             /* 轴在线 bitmask，bit0=平台轴在线，1 表示在线。 */
    uint8_t homed_mask;              /* 轴回零 bitmask，bit0=平台轴已回零，1 表示已回零。 */
    float platform_position_rad;     /* 平台轴当前位置，单位 rad。 */
} PC_OreStoreFeedback_t;

/*
 * 一键动作反馈 payload。
 * 汇总字段 action/state/result/fault 始终反映 subsystem 指向的子系统，便于 PC 端只看一组主状态。
 * ore_* 与 rod_* 字段保留两个子系统的独立状态，便于同时显示或排查问题。
 */
typedef struct {
    uint8_t action;                 /* 汇总动作类型，见 PC_AutoAction_t；当前展示子系统的动作或最近一次动作。 */
    uint8_t busy;                   /* 任一一键子系统忙标志：0=空闲，1=矿仓或取矛头流程运行中。 */
    uint8_t subsystem;              /* 当前展示/最近动作子系统，见 PC_AutoActionSubsystem_t。 */
    uint8_t state;                  /* 汇总状态，见 PC_AutoActionState_t；来自 subsystem 对应的 ore_state/rod_state。 */
    uint8_t result;                 /* 汇总结果，见 PC_AutoActionResult_t。 */
    uint8_t fault;                  /* 汇总故障，见 PC_AutoActionFault_t。 */
    uint8_t step_index;             /* 汇总流程步骤索引；矿仓取 ore_step_index，取矛头取 rod_step_index。 */
    uint8_t step_phase;             /* 汇总步骤内部阶段；仅矿仓流程使用，取矛头固定 0。 */
    uint8_t active_position;        /* 汇总矿位，见 AutoOre_Position_t：0=无，1=转换低位，2=转换高位，3=机械臂位。 */
    uint8_t occupancy_mask;         /* 汇总占矿 bitmask；bit0=转换低位，bit1=转换高位，bit2=机械臂位。 */
    uint8_t ore_action;             /* 矿仓一键动作，见 PC_AutoAction_t；当前动作或最近一次矿仓动作。 */
    uint8_t ore_state;              /* 矿仓流程状态，见 PC_AutoActionState_t。 */
    uint8_t ore_result;             /* 矿仓流程结果，见 PC_AutoActionResult_t。 */
    uint8_t ore_fault;              /* 矿仓流程故障，见 PC_AutoActionFault_t。 */
    uint8_t ore_step_index;         /* 矿仓流程步骤索引。 */
    uint8_t ore_step_phase;         /* 矿仓步骤内部阶段，具体含义由当前步骤决定。 */
    uint8_t ore_active_position;    /* 矿仓当前操作矿位，见 AutoOre_Position_t。 */
    uint8_t ore_occupancy_mask;     /* 矿仓占矿 bitmask；bit0=转换低位，bit1=转换高位，bit2=机械臂位。 */
    uint8_t rod_state;              /* 取矛头流程状态，见 PC_AutoActionState_t。 */
    uint8_t rod_result;             /* 取矛头流程结果，见 PC_AutoActionResult_t。 */
    uint8_t rod_fault;              /* 取矛头流程故障，见 PC_AutoActionFault_t。 */
    uint8_t rod_step_index;         /* 取矛头流程步骤索引。 */
    uint8_t flags;                  /* 子系统初始化/忙标志，见 PC_AUTO_ACTION_FLAG_*。 */
    uint8_t reserved;               /* 保留字段，发送端固定为 0，PC 端应忽略。 */
} PC_AutoActionFeedback_t;

/* 红外对接矿种类型。 */
typedef enum {
    PC_ORE_TYPE_UNKNOWN = 0,    /* 未知或未识别。 */
    PC_ORE_TYPE_R1 = 1,         /* R1 矿。 */
    PC_ORE_TYPE_R2 = 2,         /* R2 矿。 */
    PC_ORE_TYPE_FAKE = 3,       /* 假矿。 */
} PC_OreType_t;

#define PC_IR_ORE_POSITION_COUNT (12u)  /* 红外对接一次上报的固定矿位数量。 */

/*
 * 红外对接矿种反馈 payload。
 * 数据来自 UART7 红外对接模块：独立状态字节 + 12 字节矿种数组。
 * IR_ORE 不一定 50Hz 每次都发，默认数据变化或心跳周期到期才追加发送。
 */
typedef struct {
    uint8_t valid;                              /* 是否曾成功解析过 12 字节矿种包，0=从未收到有效矿种包，1=已有有效缓存。 */
    uint8_t fresh;                              /* 矿种信息是否新鲜，0=过期/未收到，1=在新鲜时间窗内。 */
    uint8_t status;                             /* 最近一次 1 字节状态命令，见 IrDock_Status_t：0=IDLE，1=DOCKING，2=DOCK_COMPLETE。 */
    uint8_t count;                              /* 矿位数量，固定为 PC_IR_ORE_POSITION_COUNT。 */
    uint8_t ore_type[PC_IR_ORE_POSITION_COUNT]; /* 12 个矿位类型，数组元素见 PC_OreType_t，下标为矿位编号。 */
    uint32_t age_ms;                            /* 距最近一次成功接收 12 字节矿种包的时间，单位 ms；未收到过时为 0。 */
    uint32_t rx_count;                          /* 12 字节矿种包累计成功接收次数。 */
} PC_IrOreFeedback_t;

/* 自动上下台阶流程状态。数值与 auto_ctrl_run_state_e 当前保持一致。 */
typedef enum {
    PC_STEP_STATE_IDLE = 0,       /* 空闲，未运行自动台阶流程。 */
    PC_STEP_STATE_PREALIGN = 1,   /* 姿态预对齐阶段。 */
    PC_STEP_STATE_RUNNING = 2,    /* 模板流程运行中。 */
    PC_STEP_STATE_SUCCESS = 3,    /* 流程成功完成。 */
    PC_STEP_STATE_FAIL = 4,       /* 流程失败。 */
    PC_STEP_STATE_ABORT = 5,      /* 流程已中止。 */
} PC_StepState_t;

/* 自动上下台阶流程结果。数值与 auto_ctrl_result_e 当前保持一致。 */
typedef enum {
    PC_STEP_RESULT_NONE = 0,       /* 无结果，通常表示尚未启动。 */
    PC_STEP_RESULT_RUNNING = 1,    /* 正在执行。 */
    PC_STEP_RESULT_SUCCESS = 2,    /* 执行成功。 */
    PC_STEP_RESULT_FAIL = 3,       /* 执行失败。 */
    PC_STEP_RESULT_ABORTED = 4,    /* 被中止。 */
} PC_StepResult_t;

/* 自动上下台阶故障码。数值与 auto_ctrl_fault_e 当前保持一致。 */
typedef enum {
    PC_STEP_FAULT_NONE = 0,                /* 无故障。 */
    PC_STEP_FAULT_INVALID_TEMPLATE = 1,    /* 无效自动台阶模板或模板未配置。 */
    PC_STEP_FAULT_PREALIGN_TIMEOUT = 2,    /* 姿态预对齐超时。 */
    PC_STEP_FAULT_TEMPLATE_TIMEOUT = 3,    /* 模板流程总超时。 */
    PC_STEP_FAULT_SENSOR_INVALID = 4,      /* 传感器触发条件异常或超时。 */
    PC_STEP_FAULT_UNSUPPORTED = 5,         /* 当前模板或参数不支持。 */
    PC_STEP_FAULT_ABORTED = 6,             /* 人为中止。 */
} PC_StepFault_t;

/* 自动上下台阶反馈内部格式，实际回传 payload 见 pc_comm::wire::StepFeedback。 */
typedef struct {
    PC_StepState_t state;           /* 当前状态，见 PC_StepState_t。 */
    PC_StepResult_t result;         /* 当前结果，见 PC_StepResult_t。 */
    PC_StepFault_t fault;           /* 当前故障，见 PC_StepFault_t。 */
    PC_StepTemplate_t template_id;  /* 当前执行的台阶模板，见 PC_StepTemplate_t。 */
    uint8_t step_index;             /* 当前模板内部步骤索引。 */
    float progress;                 /* 流程进度，0.0 到 1.0；当前未完整使用时为 0。 */
} PC_StepFeedback_t;

/* 通信/系统状态反馈内部格式，实际回传 payload 见 pc_comm::wire::StatusFeedback。 */
typedef struct {
    uint8_t online;                         /* PC 通信在线标志，0=离线，1=在线。 */
    uint32_t recv_count;                    /* 已成功接收的 PC 帧计数。 */
    float cpu_temp;                         /* STM32 CPU 温度，单位 degC。 */
    PC_CommandSource_t command_source;      /* 当前命令来源，见 PC_CommandSource_t。 */
} PC_StatusFeedback_t;

/* PC 命令缓存，供各控制任务读取。该结构体不是串口 payload。 */
typedef struct {
    PC_ChassisCMD_t chassis;             /* 最近一次底盘命令。 */
    PC_PoleCMD_t pole;                   /* 最近一次撑杆命令。 */
    PC_ArmCMD_t arm;                     /* 最近一次旧机械臂笛卡尔命令，当前保留。 */
    PC_ArmSimpleCMD_t arm_simple;        /* 最近一次简易机械臂命令。 */
    PC_RodNewCMD_t rod_new;              /* 最近一次取矛头机构命令。 */
    PC_OreStoreCMD_t ore_store;          /* 最近一次矿仓命令。 */
    PC_AutoActionCMD_t auto_action;      /* 待消费的一键动作命令，消费后清零。 */
    PC_StepCMD_t step;                   /* 待执行或最近一次自动台阶命令。 */
    PC_ImuCMD_t imu;                     /* 最近一次 PC 姿态数据。 */
} MrlinkPc_CMD_Data_t;

/* 反馈缓存，供打包任务构造反馈帧。该结构体不是完整串口 payload。 */
typedef struct {
    PC_ChassisFeedback_t chassis;          /* 底盘反馈缓存。 */
    PC_PoleFeedback_t pole;                /* 撑杆反馈缓存。 */
    PC_ArmSimpleFeedback_t arm_simple;     /* 简易机械臂反馈缓存。 */
    PC_RodNewFeedback_t rod_new;           /* 取矛头机构反馈缓存。 */
    PC_OreStoreFeedback_t ore_store;       /* 矿仓反馈缓存。 */
    PC_StepFeedback_t step;                /* 自动台阶反馈缓存。 */
    PC_StatusFeedback_t status;            /* 通信/系统状态反馈缓存。 */
} MrlinkPc_FeedbackData_t;

/* PC 通信模块运行状态。该结构体供 STM32 内部和调试器查看，不作为串口 payload。 */
typedef struct {
    PC_ControlMode_t control_mode;       /* 当前 PC 控制模式，见 PC_ControlMode_t。 */
    bool heartbeat_valid;                /* 心跳有效标志，false=心跳超时或尚未收到心跳。 */
    uint32_t last_heartbeat_tick;        /* 最近一次收到心跳的 tick，单位 ms。 */
    uint32_t last_recv_time;             /* 最近一次收到任意 PC 帧的 tick，单位 ms。 */
    uint32_t recv_count;                 /* 成功接收并识别的 PC 帧计数。 */
    uint32_t error_count;                /* 通信解析错误计数。 */
    bool online;                         /* PC 在线标志，false=离线，true=在线。 */
    MrlinkPc_CMD_Data_t cmd;             /* PC 命令缓存。 */
    MrlinkPc_FeedbackData_t feedback;    /* 反馈缓存。 */
} MrlinkPc_State_t;

/* PC 通信调试快照。用于 Ozone/调试器观察，不作为串口 payload。 */
typedef struct {
    uint8_t usart10_config_ok;         /* USART10 配置检查，0=异常，1=符合 PC 通信配置。 */
    uint32_t usart10_baudrate;         /* USART10 波特率。 */
    uint32_t usart10_word_length;      /* USART10 数据位配置，HAL UART_WORDLENGTH_*。 */
    uint32_t usart10_stop_bits;        /* USART10 停止位配置，HAL UART_STOPBITS_*。 */
    uint32_t usart10_parity;           /* USART10 校验位配置，HAL UART_PARITY_*。 */
    uint32_t usart10_mode;             /* USART10 收发模式配置，HAL UART_MODE_*。 */

    uint8_t online;                    /* PC 在线标志，0=离线，1=在线。 */
    uint8_t heartbeat_valid;           /* PC 心跳有效标志，0=超时/无效，1=有效。 */
    uint8_t control_mode;              /* PC 控制模式，见 PC_ControlMode_t。 */
    uint8_t command_source;            /* 当前命令来源，见 PC_CommandSource_t。 */
    uint32_t last_heartbeat_tick;      /* 最近一次 PC 心跳 tick，单位 ms。 */
    uint32_t last_recv_time;           /* 最近一次收到任意 PC 帧的 tick，单位 ms。 */
    uint32_t recv_count;               /* 成功接收并识别的 PC 帧总数。 */
    uint32_t error_count;              /* PC 通信解析错误总数。 */
    uint32_t rx_heartbeat_count;       /* 收到心跳命令次数。 */
    uint32_t rx_chassis_count;         /* 收到底盘命令次数。 */
    uint32_t rx_pole_count;            /* 收到撑杆命令次数。 */
    uint32_t rx_arm_simple_count;      /* 收到简易机械臂命令次数。 */
    uint32_t rx_rod_new_count;         /* 收到取矛头机构命令次数。 */
    uint32_t rx_ore_store_count;       /* 收到矿仓命令次数。 */
    uint32_t rx_auto_action_count;     /* 收到一键动作命令次数。 */
    uint32_t rx_step_count;            /* 收到自动台阶命令次数。 */
    uint32_t rx_imu_count;             /* 收到 PC 姿态数据次数。 */
    uint32_t init_fail_count;          /* PC 通信初始化失败次数。 */
    int8_t last_init_error;            /* 最近一次初始化错误码。 */
    uint32_t rx_restart_fail_count;    /* 底层接收重启失败次数。 */
    uint32_t rx_header_skip_count;     /* MrLink 跳过非帧头字节计数。 */
    uint32_t rx_irq_count;             /* 底层接收完成中断/事件次数。 */
    uint32_t rx_irq_byte_count;        /* 底层接收完成中断/事件累计字节数。 */
    uint32_t rx_queue_overflow_count;  /* 底层接收队列溢出次数。 */
    uint32_t rx_dma_start_fail_count;  /* UART DMA 接收启动失败次数。 */
    uint32_t mrlink_error_count;       /* MrLink 协议错误事件次数。 */
    uint8_t mrlink_last_error_code;    /* 最近一次 MrLink 错误类型，见 MrLink_ErrorCode_t。 */
    uint8_t mrlink_last_error_cmd;     /* 最近一次 MrLink 错误关联命令字。 */
    uint16_t mrlink_last_error_payload_len;       /* 最近一次错误帧 payload 长度。 */
    uint16_t mrlink_last_error_expected_size;     /* 最近一次错误期望长度或上限。 */
    uint16_t mrlink_last_error_available;         /* 最近一次错误发生时可用字节数。 */
    uint16_t mrlink_last_error_dropped_bytes;     /* 最近一次错误丢弃字节数。 */
    uint16_t mrlink_last_error_crc_received;      /* 最近一次错误帧携带的 CRC。 */
    uint16_t mrlink_last_error_crc_calculated;    /* 最近一次错误帧计算出的 CRC。 */

    uint32_t rx_batch_count;           /* 从底层通道弹出的接收批次数。 */
    uint16_t rx_batch_len;             /* 最近一批接收数据长度。 */
    uint8_t rx_batch_frame_count;      /* 最近一批数据中成功解析出的帧数。 */
    uint16_t rx_batch_raw_len;         /* rx_batch_raw 中保存的有效字节数。 */
    uint8_t rx_batch_raw[PC_COMM_DEBUG_RX_RAW_SIZE];    /* 最近一批原始接收数据。 */

    uint8_t last_rx_cmd;               /* 最近一次接收帧命令字。 */
    uint8_t last_rx_len;               /* 最近一次接收帧 payload 长度。 */
    int8_t last_rx_result;             /* 最近一次接收处理结果，MRLINK_OK 或错误码。 */
    uint16_t last_rx_frame_size;       /* 最近一次接收帧完整长度。 */
    uint16_t last_rx_crc_received;     /* 最近一次接收帧携带的 CRC。 */
    uint16_t last_rx_crc_calculated;   /* 最近一次接收帧计算出的 CRC。 */
    uint16_t last_rx_raw_len;          /* last_rx_raw 中保存的有效字节数。 */
    uint8_t last_rx_raw[MRLINK_PC_MAX_FRAME_SIZE];      /* 最近一次原始接收帧。 */

    PC_ChassisCMD_t rx_chassis;             /* 调试用：最近一次底盘命令。 */
    PC_PoleCMD_t rx_pole;                   /* 调试用：最近一次撑杆命令。 */
    PC_ArmSimpleCMD_t rx_arm_simple;        /* 调试用：最近一次简易机械臂命令。 */
    PC_RodNewCMD_t rx_rod_new;              /* 调试用：最近一次取矛头机构命令。 */
    PC_OreStoreCMD_t rx_ore_store;          /* 调试用：最近一次矿仓命令。 */
    PC_AutoActionCMD_t rx_auto_action;      /* 调试用：最近一次一键动作命令。 */
    PC_StepCMD_t rx_step;                   /* 调试用：最近一次自动台阶命令。 */
    PC_ImuCMD_t rx_imu;                     /* 调试用：最近一次 PC 姿态数据。 */

    uint32_t tx_count;                /* 反馈发送尝试次数。 */
    uint8_t tx_dma_busy;              /* 发送 DMA 忙标志，0=空闲，1=忙。 */
    uint32_t tx_busy_skip_count;      /* 因发送 DMA 忙而跳过发送的次数。 */
    uint32_t tx_dma_error_count;      /* 发送 DMA 错误或超时次数。 */
    uint16_t tx_len;                  /* 最近一次发送总长度。 */
    uint8_t tx_frame_count;           /* 最近一次发送中打包的反馈帧数量。 */
    int8_t tx_result;                 /* 最近一次发送结果，MRLINK_CHANNEL_OK 或错误码。 */
    uint16_t tx_raw_len;              /* tx_raw 中保存的有效字节数。 */
    uint8_t tx_raw[PC_COMM_DEBUG_TX_RAW_SIZE];          /* 最近一次原始发送数据。 */

    MrLink_Stats_t mrlink_stats;      /* MrLink 协议层统计快照。 */
} PC_CommDebug_t;

/* PC 自动台阶命令映射到 AutoCtrl 后的内部参数，不作为串口 payload。 */
typedef struct {
    auto_ctrl_template_e template_id;       /* 映射后的自动台阶模板。 */
    auto_ctrl_travel_dir_e travel_dir;      /* 映射后的行进方向。 */
    float target_yaw_rad;                   /* 自动台阶目标航向角，单位 rad。 */
    float yaw_tolerance_rad;                /* 自动台阶航向允许误差，单位 rad。 */
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
