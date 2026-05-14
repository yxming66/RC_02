#pragma once

#include "motor.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "device/device.h"
#include "device/motor.h"
#include "bsp/can.h"

/* Exported constants ------------------------------------------------------- */
#define MOTOR_RM_MAX_MOTORS 11

/* Exported macro ----------------------------------------------------------- */
/* Exported types ----------------------------------------------------------- */
typedef enum {
    MOTOR_M2006,
    MOTOR_M3508,
    MOTOR_GM6020,
} MOTOR_RM_Module_t;

typedef enum {
    MOTOR_RM_C610_ERROR_NONE = 0,
    MOTOR_RM_C610_ERROR_SUPPLY_OVERVOLTAGE = 2,
    MOTOR_RM_C610_ERROR_PHASE_LOSS = 3,
    MOTOR_RM_C610_ERROR_SENSOR_LOST = 4,
    MOTOR_RM_C610_ERROR_STALL = 6,
    MOTOR_RM_C610_ERROR_CALIBRATION_FAILED = 7,
} MOTOR_RM_C610_ErrorCode_t;

/*一个can最多控制11个电机*/
typedef union {
  int16_t output[MOTOR_RM_MAX_MOTORS];
  struct {
    int16_t m3508_m2006_id201;
    int16_t m3508_m2006_id202;
    int16_t m3508_m2006_id203;
    int16_t m3508_m2006_id204;
    int16_t m3508_m2006_gm6020_id205;
    int16_t m3508_m2006_gm6020_id206;
    int16_t m3508_m2006_gm6020_id207;
    int16_t m3508_m2006_gm6020_id208;
    int16_t gm6020_id209;
    int16_t gm6020_id20A;
    int16_t gm6020_id20B;
  } named;
} MOTOR_RM_MsgOutput_t;

/*每个电机需要的参数*/
typedef struct {
    BSP_CAN_t can;
    uint16_t id;
    MOTOR_RM_Module_t module;
    bool reverse;
    bool gear;
} MOTOR_RM_Param_t;

typedef MOTOR_Feedback_t MOTOR_RM_Feedback_t;

typedef MOTOR_RawFeedback_t MOTOR_RM_RawFeedback_t;

typedef struct {
    MOTOR_RM_Param_t param;
    MOTOR_RM_Feedback_t feedback;
    MOTOR_t motor;
    // 多圈相关变量，仅gear模式下有效
    uint16_t last_raw_angle;
    int32_t gearbox_round_count;
    float gearbox_total_angle;
} MOTOR_RM_t;

/*CAN管理器，管理一个CAN总线上所有的电机*/
typedef struct {
    BSP_CAN_t can;
    MOTOR_RM_MsgOutput_t output_msg;
    MOTOR_RM_t *motors[MOTOR_RM_MAX_MOTORS];
    uint8_t motor_count;
  MOTOR_RM_t *external_motors[MOTOR_RM_MAX_MOTORS];
  uint8_t external_motor_count;
} MOTOR_RM_CANManager_t;

typedef struct {
    uint8_t valid;
    BSP_CAN_t can;
    uint16_t source_motor_id;
    uint16_t tx_frame_id;
    int8_t logical_index;
    int16_t requested_current;
    int16_t grouped_output[MOTOR_RM_MAX_MOTORS];
    uint8_t tx_data[8];
} MOTOR_RM_TxDebug_t;

  typedef struct {
    uint8_t valid;
    BSP_CAN_t can;
    uint16_t tx_frame_id;
    int8_t logical_index;
    int16_t output_value;
    uint8_t tx_data[8];
  } MOTOR_RM_SlotTxDebug_t;

  extern MOTOR_RM_SlotTxDebug_t g_motor_rm_slot_tx_debug[MOTOR_RM_MAX_MOTORS];

/* Exported functions prototypes -------------------------------------------- */

/**
 * @brief 注册一个RM电机
 * @param param 电机参数
 * @return 
 */
int8_t MOTOR_RM_Register(MOTOR_RM_Param_t *param);

/**
 * @brief 将外部分配的 RM 电机实例附着到底层驱动
 * @param param 电机参数
 * @param external_motor 外部实例存储，生命周期需覆盖整个使用期
 * @return 设备状态码
 */
int8_t MOTOR_RM_AttachExternal(MOTOR_RM_Param_t *param, MOTOR_RM_t *external_motor);

/**
 * @brief 更新指定电机数据
 * @param param 电机参数
 * @return 
 */
int8_t MOTOR_RM_Update(MOTOR_RM_Param_t *param);

/**
 * @brief 设置一个电机的归一化输出（底层兼容接口）
 * @param param 电机参数
 * @param value 输出值，范围[-1.0, 1.0]
 * @note 这是底层发送接口，保留给旧代码或直接比例输出场景使用。
 *       C++ 电机驱动的力矩控制链路不应直接调用它，而应优先使用
 *       MOTOR_RM_SetTorqueCurrent()，先完成“输出轴力矩 -> 转子侧电流”的物理量换算。
 * @return 
 */
int8_t MOTOR_RM_SetOutput(MOTOR_RM_Param_t *param, float value);

/**
 * @brief 设置一个电机的转子侧电流命令（C++ 力矩控制主入口）
 * @param param 电机参数
 * @param current_a 转子侧目标电流，单位 A
 * @note C++ RM 驱动会先将目标输出轴力矩按转矩常数、减速比、外部传动比
 *       换算为转子侧电流（A），本接口再统一完成 A -> RM raw 指令值 的换算。
 * @return
 */
int8_t MOTOR_RM_SetTorqueCurrent(MOTOR_RM_Param_t *param, float current_a);

/**
 * @brief 发送控制命令到电机，注意一个CAN可以控制多个电机，所以只需要发送一次即可
 * @param param 电机参数
 * @return 
 */
int8_t MOTOR_RM_Ctrl(MOTOR_RM_Param_t *param);

/**
 * @brief 获取指定电机的实例指针
 * @param param 电机参数
 * @return 
 */
MOTOR_RM_t* MOTOR_RM_GetMotor(MOTOR_RM_Param_t *param);

/**
 * @brief 使电机松弛（设置输出为0）
 * @param param 
 * @return 
 */
int8_t MOTOR_RM_Relax(MOTOR_RM_Param_t *param);

/**
 * @brief 使电机离线（设置在线状态为false）
 * @param param 
 * @return 
 */
int8_t MOTOR_RM_Offine(MOTOR_RM_Param_t *param);

/**
 * @brief 
 * @param  
 * @return 
 */
int8_t MOTOR_RM_UpdateAll(void);

const MOTOR_RM_RawFeedback_t* MOTOR_RM_GetRawFeedback(MOTOR_RM_Param_t *param);

const MOTOR_RM_TxDebug_t* MOTOR_RM_GetTxDebug(void);

const MOTOR_RM_SlotTxDebug_t* MOTOR_RM_GetSlotTxDebug(uint8_t logical_index);

#ifdef __cplusplus
}
#endif
