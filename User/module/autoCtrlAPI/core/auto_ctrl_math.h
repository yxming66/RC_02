#pragma once

/**
 * @file auto_ctrl_math.h
 * @brief AutoCtrl 使用的 yaw 弧度数学工具函数。
 *
 * 本文件提供轻量弧度计算：
 * - 弧度归一化到 [-pi, pi]；
 * - 目标/当前 yaw 的最短误差；
 * - 是否在容差内完成对齐。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* 将任意 yaw 折返到 [-pi, pi] 区间。 */
float AutoCtrlMath_WrapYawRad(float yaw_rad);

/* 计算 target - current 的有符号最短 yaw 误差（弧度）。 */
float AutoCtrlMath_GetYawErrorRad(float target_yaw_rad, float current_yaw_rad);

/* 判断当前 yaw 是否已落入容差范围。 */
bool AutoCtrlMath_IsYawAlignedRad(float target_yaw_rad, float current_yaw_rad,
                                  float tolerance_rad);

#ifdef __cplusplus
}
#endif