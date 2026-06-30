#pragma once

#include <cstdint>
#include <type_traits>

#include "device/mrlink/mrlink.hpp"
#include "module/mrlink_pc_comm/mrlink_pc_comm.h"

namespace pc_comm::wire {

using Topic = uint8_t;

/*
 * PC_COMM wire payload definitions.
 *
 * These types are the exact byte layout used on the PC serial link. They are
 * intentionally packed because several firmware-side C structs contain enum
 * fields or natural alignment padding that must not leak into the wire format.
 *
 * Host-side packing rules:
 *   - All integer fields are unsigned unless explicitly documented otherwise.
 *   - float fields are IEEE-754 binary32, little-endian.
 *   - Multi-byte integer fields are little-endian.
 *   - Use the sizes asserted below, not sizeof() from a host compiler unless
 *     the host struct is also packed with the same field order.
 */

inline constexpr Topic kCmdHeartbeat = PC_CMD_HEARTBEAT;
inline constexpr Topic kCmdChassis = PC_CMD_CHASSIS;
inline constexpr Topic kCmdPole = PC_CMD_POLE;
inline constexpr Topic kCmdStep = PC_CMD_STEP;
inline constexpr Topic kCmdArmSimple = PC_CMD_ARM_SIMPLE;
inline constexpr Topic kCmdRodNew = PC_CMD_ROD_NEW;
inline constexpr Topic kCmdOreStore = PC_CMD_ORE_STORE;
inline constexpr Topic kCmdAutoAction = PC_CMD_AUTO_ACTION;
inline constexpr Topic kCmdCameraYaw = PC_CMD_CAMERA_YAW;
inline constexpr Topic kCmdAbstractPosition = PC_CMD_ABSTRACT_POSITION;
inline constexpr Topic kCmdImu = PC_CMD_IMU;
inline constexpr Topic kCmdIrOreAck = PC_CMD_IR_ORE_ACK;

inline constexpr Topic kFeedbackHeartbeat = PC_FEEDBACK_HEARTBEAT;
inline constexpr Topic kFeedbackStartMatch = PC_FEEDBACK_START_MATCH;
inline constexpr Topic kFeedbackChassis = PC_FEEDBACK_CHASSIS;
inline constexpr Topic kFeedbackPole = PC_FEEDBACK_POLE;
inline constexpr Topic kFeedbackStep = PC_FEEDBACK_STEP;
inline constexpr Topic kFeedbackArmSimple = PC_FEEDBACK_ARM_SIMPLE;
inline constexpr Topic kFeedbackRodNew = PC_FEEDBACK_ROD_NEW;
inline constexpr Topic kFeedbackOreStore = PC_FEEDBACK_ORE_STORE;
inline constexpr Topic kFeedbackAutoAction = PC_FEEDBACK_AUTO_ACTION;
inline constexpr Topic kFeedbackIrOre = PC_FEEDBACK_IR_ORE;
inline constexpr Topic kFeedbackIrOreBridge = PC_FEEDBACK_IR_ORE_BRIDGE;
inline constexpr Topic kFeedbackCameraYaw = PC_FEEDBACK_CAMERA_YAW;
inline constexpr Topic kFeedbackStatus = PC_FEEDBACK_STATUS;

struct __attribute__((packed)) PoleCmd {
  uint8_t mode;    /* 撑杆模式，0=放松，其它=主动控制 */
  float lift0;     /* 前组撑杆目标高度/角度，单位 rad */
  float lift1;     /* 后组撑杆目标高度/角度，单位 rad */
};

struct __attribute__((packed)) ArmSimpleCmd {
  uint8_t mode;                /* 简易机械臂模式，见 ArmSimple_Mode_t */
  uint8_t point_mode;          /* 点位模式，见 ArmSimple_PointMode_t */
  float target_joint1_rad;     /* 关节 1 目标角度，单位 rad */
  float target_joint2_rad;     /* 关节 2 目标角度，单位 rad */
};

struct __attribute__((packed)) RodNewCmd {
  uint8_t mode;              /* 取矛头机构模式，见 RodNew_Mode_t */
  uint8_t pose;              /* 取矛头机构姿态，见 RodNew_Pose_t */
  float target_angle_rad;    /* 舵机目标角度，单位 rad */
};

struct __attribute__((packed)) OreStoreCmd {
  uint8_t mode;                 /* 矿仓模式，见 OreStore_Mode_t */
  uint8_t force_rehome;         /* 强制重新回零，0=不触发，1=触发一次 */
  float platform_target_rad;    /* 平台轴目标位置，单位 rad */
};

struct __attribute__((packed)) CameraYawCmd {
  uint8_t mode;              /* 云台模式；当前接收缓存保留，控制任务固定按 ACTIVE 使用 */
  float target_yaw_rad;      /* 车身系目标 yaw，单位 rad */
};

struct __attribute__((packed)) AutoActionCmd {
  uint8_t action;    /* 一键动作类型，见 PC_AutoAction_t */
};

struct __attribute__((packed)) StartMatchCmd {
  uint8_t start;     /* 0=默认等待，1=开始/启动 */
};

struct __attribute__((packed)) StepCmd {
  uint8_t template_id;          /* 自动台阶模板，见 PC_StepTemplate_t */
  uint8_t travel_dir;           /* 行进方向，见 PC_StepDir_t */
  float target_yaw_rad;         /* 目标航向角，单位 rad */
  float yaw_tolerance_rad;      /* 航向允许误差，单位 rad */
};

struct __attribute__((packed)) ArmSimpleFeedback {
  uint8_t mode;                   /* 简易机械臂当前模式，见 ArmSimple_Mode_t */
  uint8_t point_mode;             /* 简易机械臂点位模式，见 ArmSimple_PointMode_t */
  uint8_t suction;                /* 吸盘状态，0=关闭，1=开启 */
  float joint1_angle_rad;         /* 关节 1 当前角度，单位 rad */
  float joint1_velocity_rad_s;    /* 关节 1 当前速度，单位 rad/s */
  float joint2_angle_rad;         /* 关节 2 当前角度，单位 rad */
};

struct __attribute__((packed)) RodNewFeedback {
  uint8_t mode;                    /* 取矛头机构当前模式，见 RodNew_Mode_t */
  uint8_t pose;                    /* 取矛头机构目标姿态，见 RodNew_Pose_t */
  uint8_t grip;                    /* 夹爪状态，0=松开，1=夹紧 */
  uint8_t at_target;               /* 到位标志，0=未到位，1=已到位 */
  float target_angle_rad;          /* 舵机目标角度，单位 rad */
  float tracked_angle_rad;         /* 轨迹规划后的跟踪角度，单位 rad */
  float tracked_velocity_rad_s;    /* 轨迹规划后的跟踪速度，单位 rad/s */
  float feedback_angle_rad;        /* 舵机反馈角度，单位 rad */
};

struct __attribute__((packed)) OreStoreFeedback {
  uint8_t mode;                    /* 矿仓当前模式，见 OreStore_Mode_t */
  uint8_t all_homed;               /* 全部轴回零标志，0=未全部回零，1=全部已回零 */
  uint8_t online_mask;             /* 轴在线 bitmask，bit0=平台轴在线，1 表示在线 */
  uint8_t homed_mask;              /* 轴回零 bitmask，bit0=平台轴已回零，1 表示已回零 */
  float platform_position_rad;     /* 平台轴当前位置，单位 rad */
  uint8_t transform_low_has_ore;    /* 变形机构低位占矿状态，0/1 */
  uint8_t transform_high_has_ore;   /* 变形机构高位占矿状态，0/1 */
  uint8_t arm_has_ore;              /* 机械臂持矿状态，0/1 */
  uint8_t release_grid_has_ore;     /* 放矿目标格占矿检测结果，0/1 */
};

struct __attribute__((packed)) StepFeedback {
  uint8_t state;          /* 当前状态，见 PC_StepState_t */
  uint8_t result;         /* 当前结果，见 PC_StepResult_t */
  uint8_t fault;          /* 当前故障，见 PC_StepFault_t */
  uint8_t template_id;    /* 当前执行的台阶模板，见 PC_StepTemplate_t */
  uint8_t step_index;     /* 当前模板内部步骤索引 */
  uint8_t reserved;       /* 保留字段，发送端固定为 0 */
  float progress;         /* 流程进度，0.0~1.0，当前未完整使用时为 0 */
};

struct __attribute__((packed)) StatusFeedback {
  uint8_t online;             /* PC 通信在线标志，0=离线，1=在线 */
  uint32_t recv_count;        /* 已成功接收的 PC 帧计数 */
  float cpu_temp;             /* STM32 CPU 温度，单位 degC */
  uint8_t command_source;     /* 当前命令来源，见 PC_CommandSource_t */
};

template <typename T>
constexpr bool IsValidWirePayload() {
  return std::is_trivially_copyable<T>::value &&
         sizeof(T) <= MRLINK_PC_MAX_PAYLOAD_SIZE;
}

static_assert(IsValidWirePayload<PC_ChassisCMD_t>(), "PC_ChassisCMD_t payload is invalid");
static_assert(IsValidWirePayload<PC_ImuCMD_t>(), "PC_ImuCMD_t payload is invalid");
static_assert(IsValidWirePayload<PC_IrOreAckCMD_t>(), "PC_IrOreAckCMD_t payload is invalid");
static_assert(IsValidWirePayload<StartMatchCmd>(), "StartMatchCmd payload is invalid");
static_assert(IsValidWirePayload<PoleCmd>(), "PoleCmd payload is invalid");
static_assert(IsValidWirePayload<ArmSimpleCmd>(), "ArmSimpleCmd payload is invalid");
static_assert(IsValidWirePayload<RodNewCmd>(), "RodNewCmd payload is invalid");
static_assert(IsValidWirePayload<OreStoreCmd>(), "OreStoreCmd payload is invalid");
static_assert(IsValidWirePayload<AutoActionCmd>(), "AutoActionCmd payload is invalid");
static_assert(IsValidWirePayload<CameraYawCmd>(), "CameraYawCmd payload is invalid");
static_assert(IsValidWirePayload<PC_AbstractPositionCMD_t>(), "PC_AbstractPositionCMD_t payload is invalid");
static_assert(IsValidWirePayload<StepCmd>(), "StepCmd payload is invalid");
static_assert(IsValidWirePayload<PC_ChassisFeedback_t>(), "PC_ChassisFeedback_t payload is invalid");
static_assert(IsValidWirePayload<PC_PoleFeedback_t>(), "PC_PoleFeedback_t payload is invalid");
static_assert(IsValidWirePayload<PC_AutoActionFeedback_t>(), "PC_AutoActionFeedback_t payload is invalid");
static_assert(IsValidWirePayload<PC_IrOreFeedback_t>(), "PC_IrOreFeedback_t payload is invalid");
static_assert(IsValidWirePayload<PC_IrOreBridgeFeedback_t>(), "PC_IrOreBridgeFeedback_t payload is invalid");
static_assert(IsValidWirePayload<PC_CameraYawFeedback_t>(), "PC_CameraYawFeedback_t payload is invalid");
static_assert(IsValidWirePayload<ArmSimpleFeedback>(), "ArmSimpleFeedback payload is invalid");
static_assert(IsValidWirePayload<RodNewFeedback>(), "RodNewFeedback payload is invalid");
static_assert(IsValidWirePayload<OreStoreFeedback>(), "OreStoreFeedback payload is invalid");
static_assert(IsValidWirePayload<StepFeedback>(), "StepFeedback payload is invalid");
static_assert(IsValidWirePayload<StatusFeedback>(), "StatusFeedback payload is invalid");

static_assert(sizeof(PC_ChassisCMD_t) == 12u, "PC_CMD_CHASSIS wire size changed");
static_assert(sizeof(PC_ImuCMD_t) == 28u, "PC_CMD_IMU wire size changed");
static_assert(sizeof(PC_IrOreAckCMD_t) == 6u, "PC_CMD_IR_ORE_ACK wire size changed");
static_assert(sizeof(StartMatchCmd) == 1u, "PC_FEEDBACK_START_MATCH wire size changed");
static_assert(sizeof(PoleCmd) == 9u, "PC_CMD_POLE wire size changed");
static_assert(sizeof(ArmSimpleCmd) == 10u, "PC_CMD_ARM_SIMPLE wire size changed");
static_assert(sizeof(RodNewCmd) == 6u, "PC_CMD_ROD_NEW wire size changed");
static_assert(sizeof(OreStoreCmd) == 6u, "PC_CMD_ORE_STORE wire size changed");
static_assert(sizeof(AutoActionCmd) == 1u, "PC_CMD_AUTO_ACTION wire size changed");
static_assert(sizeof(CameraYawCmd) == 5u, "PC_CMD_CAMERA_YAW wire size changed");
static_assert(sizeof(PC_AbstractPositionCMD_t) == 5u,
              "PC_CMD_ABSTRACT_POSITION wire size changed");
static_assert(sizeof(StepCmd) == 10u, "PC_CMD_STEP wire size changed");
static_assert(sizeof(PC_ChassisFeedback_t) == 12u,
              "PC_FEEDBACK_CHASSIS wire size changed");
static_assert(sizeof(PC_PoleFeedback_t) == 24u,
              "PC_FEEDBACK_POLE wire size changed");
static_assert(sizeof(PC_AutoActionFeedback_t) == 6u,
              "PC_FEEDBACK_AUTO_ACTION wire size changed");
static_assert(sizeof(PC_IrOreFeedback_t) == 24u,
              "PC_FEEDBACK_IR_ORE wire size changed");
static_assert(sizeof(PC_IrOreBridgeFeedback_t) == 56u,
              "PC_FEEDBACK_IR_ORE_BRIDGE wire size changed");
static_assert(sizeof(PC_CameraYawFeedback_t) == 32u,
              "PC_FEEDBACK_CAMERA_YAW wire size changed");
static_assert(sizeof(ArmSimpleFeedback) == 15u,
              "PC_FEEDBACK_ARM_SIMPLE wire size changed");
static_assert(sizeof(RodNewFeedback) == 20u,
              "PC_FEEDBACK_ROD_NEW wire size changed");
static_assert(sizeof(OreStoreFeedback) == 12u,
              "PC_FEEDBACK_ORE_STORE wire size changed");
static_assert(sizeof(StepFeedback) == 10u,
              "PC_FEEDBACK_STEP wire size changed");
static_assert(sizeof(StatusFeedback) == 10u,
              "PC_FEEDBACK_STATUS wire size changed");

}  // namespace pc_comm::wire

namespace mr::link {

#define MRLINK_PC_MESSAGE_TRAIT(PayloadType, TopicValue) \
  template <>                                             \
  struct MessageTraits<PayloadType> {                     \
    static constexpr Topic topic = static_cast<Topic>(TopicValue); \
  }

MRLINK_PC_MESSAGE_TRAIT(PC_ChassisCMD_t, PC_CMD_CHASSIS);
MRLINK_PC_MESSAGE_TRAIT(PC_ImuCMD_t, PC_CMD_IMU);
MRLINK_PC_MESSAGE_TRAIT(PC_IrOreAckCMD_t, PC_CMD_IR_ORE_ACK);
MRLINK_PC_MESSAGE_TRAIT(pc_comm::wire::PoleCmd, PC_CMD_POLE);
MRLINK_PC_MESSAGE_TRAIT(pc_comm::wire::ArmSimpleCmd, PC_CMD_ARM_SIMPLE);
MRLINK_PC_MESSAGE_TRAIT(pc_comm::wire::RodNewCmd, PC_CMD_ROD_NEW);
MRLINK_PC_MESSAGE_TRAIT(pc_comm::wire::OreStoreCmd, PC_CMD_ORE_STORE);
MRLINK_PC_MESSAGE_TRAIT(pc_comm::wire::AutoActionCmd, PC_CMD_AUTO_ACTION);
MRLINK_PC_MESSAGE_TRAIT(pc_comm::wire::CameraYawCmd, PC_CMD_CAMERA_YAW);
MRLINK_PC_MESSAGE_TRAIT(PC_AbstractPositionCMD_t, PC_CMD_ABSTRACT_POSITION);
MRLINK_PC_MESSAGE_TRAIT(pc_comm::wire::StepCmd, PC_CMD_STEP);
MRLINK_PC_MESSAGE_TRAIT(PC_ChassisFeedback_t, PC_FEEDBACK_CHASSIS);
MRLINK_PC_MESSAGE_TRAIT(PC_PoleFeedback_t, PC_FEEDBACK_POLE);
MRLINK_PC_MESSAGE_TRAIT(PC_AutoActionFeedback_t, PC_FEEDBACK_AUTO_ACTION);
MRLINK_PC_MESSAGE_TRAIT(PC_IrOreFeedback_t, PC_FEEDBACK_IR_ORE);
MRLINK_PC_MESSAGE_TRAIT(PC_IrOreBridgeFeedback_t, PC_FEEDBACK_IR_ORE_BRIDGE);
MRLINK_PC_MESSAGE_TRAIT(PC_CameraYawFeedback_t, PC_FEEDBACK_CAMERA_YAW);
MRLINK_PC_MESSAGE_TRAIT(pc_comm::wire::ArmSimpleFeedback, PC_FEEDBACK_ARM_SIMPLE);
MRLINK_PC_MESSAGE_TRAIT(pc_comm::wire::RodNewFeedback, PC_FEEDBACK_ROD_NEW);
MRLINK_PC_MESSAGE_TRAIT(pc_comm::wire::OreStoreFeedback, PC_FEEDBACK_ORE_STORE);
MRLINK_PC_MESSAGE_TRAIT(pc_comm::wire::StepFeedback, PC_FEEDBACK_STEP);
MRLINK_PC_MESSAGE_TRAIT(pc_comm::wire::StatusFeedback, PC_FEEDBACK_STATUS);

#undef MRLINK_PC_MESSAGE_TRAIT

}  // namespace mr::link
