/**
 ******************************************************************************
 * @file    arm.h
 * @brief   6-DOF robotic arm kinematics control
 ******************************************************************************
 */

#ifndef ARM_H
#define ARM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    // 6个运动关节的DH参数
    float theta;
    float d;
    float a;
    float alpha;
    float theta_offset;
    float qmin;
    float qmax;
    float m;        // 连杆质量 (kg)
    float rc[3];    // 质心在 DH 连杆坐标系下的坐标 (m): {x, y, z}
                    // 由 URDF inertial/origin 经 Rz(-theta_offset) 旋转得到
} Arm6dof_DHParams_t;
typedef struct {
    float x, y, z;           // 位置 (m)
    float roll, pitch, yaw;  // 姿态 (rad) - RPY欧拉角
} Arm6dof_Pose_t;

typedef struct {
    float dx, dy, dz;              // 位置增量 (m)
    float droll, dpitch, dyaw;     // 姿态增量 (rad)
} Arm6dof_PoseDelta_t;

typedef struct {
    float q[6];  // 6个关节角度 (rad)
} Arm6dof_JointAngles_t;

typedef enum {
    ARM6DOF_FRAME_WORLD = 1,
    ARM6DOF_FRAME_TOOL = 2,
    ARM6DOF_FRAME_HEADING = 3,
} Arm6dof_ControlFrame_t;

typedef enum {
    ARM6DOF_IK_NUMERICAL = 0,
    ARM6DOF_IK_POSITION_DECOUPLED,
} Arm6dof_IkMode_t;

typedef struct {
    float max_lin_vel;
    float max_ang_vel;
    float joint_limit_tolerance;
    float max_joint_delta;
    float ik_tol;
    uint16_t ik_max_iter;
} Arm6dof_TrajectoryConfig_t;

typedef struct {
    Arm6dof_Pose_t start;
    Arm6dof_Pose_t goal;
    float t;
    bool active;
    bool valid;
    Arm6dof_JointAngles_t angles;
} Arm6dof_TrajectoryState_t;

/**
 * @brief 初始化机械臂运动学模型
 * @param dh_params 6个关节的DH参数数组
 * @note 必须先调用此函数再使用正/逆运动学
 */
void Arm6dof_Init(const Arm6dof_DHParams_t dh_params[6]);

/**
 * @brief 正运动学：根据关节角度计算末端位姿
 * @param q 输入的6个关节角度 (rad)
 * @param pose 输出的末端位姿 (位置m, 姿态rad)
 * @retval 0: 成功, -1: 失败(未初始化)
 */
int Arm6dof_ForwardKinematics(const Arm6dof_JointAngles_t* q, Arm6dof_Pose_t* pose);

/**
 * @brief 逆运动学：根据末端位姿计算关节角度
 * @param pose 输入的目标末端位姿 (位置m, 姿态rad)
 * @param q_init 初始关节角度猜测值 (用于数值解迭代)
 * @param q_result 输出的关节角度解 (rad)
 * @param tol 收敛误差容限，位置分量单位为(m)，建议0.001f(即1mm精度)
 * @param max_iter 最大迭代次数，默认10
 * @retval 0: 成功, -1: 失败(未初始化或无解)
 * @note 使用牛顿法数值求解，需要提供合理的初始猜测值
 */
int Arm6dof_InverseKinematics_NumericalSolution(const Arm6dof_Pose_t* pose, const Arm6dof_JointAngles_t* q_init,
                          Arm6dof_JointAngles_t* q_result, float tol, uint16_t max_iter);

/**
 * @brief 解析逆运动学：枚举全部有效解
 * @param pose 输入目标末端位姿（位置+姿态）
 * @param q_seed 初始关节角度（可为NULL，用于生成更连续的候选分支）
 * @param q_solutions 输出的有效解数组
 * @param max_solutions 输出数组容量（最多写入的解数量）
 * @param solution_count 实际求得的有效解数量
 * @retval 0: 成功, -1: 失败
 * @note 当前实现采用“解析分支枚举 + 数值精修 + 模型校验”的混合策略。
 */
int Arm6dof_InverseKinematics_AnalyticalEnumerate(const Arm6dof_Pose_t* pose,
                                                  const Arm6dof_JointAngles_t* q_seed,
                                                  Arm6dof_JointAngles_t* q_solutions,
                                                  uint16_t max_solutions,
                                                  uint16_t* solution_count);

/**
 * @brief 从多组解析解中选择离当前角度最近的一组
 * @param q_current 当前关节角度
 * @param q_solutions 候选解数组
 * @param solution_count 候选解数量
 * @param q_result 输出选中的关节角度
 * @retval 0: 成功, -1: 失败
 */
int Arm6dof_SelectNearestSolution(const Arm6dof_JointAngles_t* q_current,
                                  const Arm6dof_JointAngles_t* q_solutions,
                                  uint16_t solution_count,
                                  Arm6dof_JointAngles_t* q_result);

/**
 * @brief 解析逆运动学：枚举全部有效解后，选择离当前角度最近的一组
 * @param pose 输入目标末端位姿（位置+姿态）
 * @param q_current 当前关节角度（用于决策，可为NULL）
 * @param q_result 输出关节角度（rad）
 * @retval 0: 成功, -1: 失败
 */
int Arm6dof_InverseKinematics_AnalyticalSolution(const Arm6dof_Pose_t* pose,
                                                 const Arm6dof_JointAngles_t* q_current,
                                                 Arm6dof_JointAngles_t* q_result);

/**
 * @brief 位置优先的降维逆运动学（J1~J3主控，J4~J6保持）
 * @param pose 输入目标末端位姿（位置用于主约束）
 * @param q_seed 初始关节角度（可为NULL，NULL时内部取零）
 * @param q_result 输出关节角度（rad）
 * @retval 0: 成功, -1: 失败
 */
int Arm6dof_InverseKinematics_PositionDecoupled(const Arm6dof_Pose_t* pose,
                                               const Arm6dof_JointAngles_t* q_seed,
                                               Arm6dof_JointAngles_t* q_result);

/**
 * @brief 获取指定关节的位姿 (用于可视化或调试)
 * @param q 输入的6个关节角度 (rad)
 * @param joint_num 关节编号 (1-6)
 * @param pose 输出的该关节位姿
 * @retval 0: 成功, -1: 失败
 */
int Arm6dof_GetJointPose(const Arm6dof_JointAngles_t* q, uint8_t joint_num, Arm6dof_Pose_t* pose);

/**
 * @brief 将指定控制坐标系下的增量变换到世界系，并叠加到当前位姿得到新目标
 */
int Arm6dof_ApplyPoseDelta(const Arm6dof_Pose_t* current_world,
                           const Arm6dof_PoseDelta_t* delta_in_frame,
                           Arm6dof_ControlFrame_t frame,
                           Arm6dof_Pose_t* target_world);

/**
 * @brief 将当前世界系位姿反算为指定控制坐标系下目标（用于无跳变同步）
 */
int Arm6dof_SyncWorldPoseToFrameTarget(const Arm6dof_Pose_t* current_world,
                                       Arm6dof_ControlFrame_t frame,
                                       Arm6dof_Pose_t* target_in_frame);

/**
 * @brief 轨迹推进一步（插值 + IK + 连续性/限位检查）
 * @param state 轨迹状态（输入输出）
 * @param cfg 轨迹与IK配置
 * @param ik_mode IK求解模式
 * @param q_current 当前关节角（首帧或上帧无效时作为IK初值）
 * @param interp_pose 输出当前插值位姿
 * @retval 0: 成功, -1: 失败
 */
int Arm6dof_TrajectoryStep(Arm6dof_TrajectoryState_t* state,
                          const Arm6dof_TrajectoryConfig_t* cfg,
                          Arm6dof_IkMode_t ik_mode,
                          const Arm6dof_JointAngles_t* q_current,
                          Arm6dof_Pose_t* interp_pose,
                          float dt);
/**
 * @brief 计算重力补偿力矩（基于牛顿-欧拉逆动力学，速度和加速度为零）
 * @param q 当前6个关节角度 (rad)
 * @param gravity_torques 输出的6个关节重力补偿力矩 (N·m，DH参数单位为m时)
 * @retval 0: 成功, -1: 失败(未初始化)
 */
int Arm6dof_GravityCompensation(const Arm6dof_JointAngles_t* q, float gravity_torques[6]);

#ifdef __cplusplus
}
#endif

#endif // ARM_H
