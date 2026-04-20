#pragma once

/**
 * @file auto_ctrl_math.h
 * @brief AutoCtrl 使用的 yaw 角度数学工具函数。
 *
 * 本文件提供轻量角度计算：
 * - 角度归一化到 [-180, 180]；
 * - 目标/当前 yaw 的最短误差；
 * - 是否在容差内完成对齐。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* 将任意角度折返到 [-180, 180] 区间。 */
float AutoCtrlMath_WrapYawDeg(float yaw_deg);

/* 计算 target - current 的有符号最短 yaw 误差（度）。 */
float AutoCtrlMath_GetYawErrorDeg(float target_yaw_deg, float current_yaw_deg);

/* 判断当前 yaw 是否已落入容差范围。 */
bool AutoCtrlMath_IsYawAligned(float target_yaw_deg, float current_yaw_deg,
                               float tolerance_deg);

#ifdef __cplusplus
}
#endif