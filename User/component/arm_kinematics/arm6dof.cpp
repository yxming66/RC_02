/**
 ******************************************************************************
 * @file    arm.cpp
 * @brief   6-DOF robotic arm kinematics control implementation
 ******************************************************************************
 */

#include "arm6dof.h"
#include "arm_math.h"
#include "toolbox/robotics.h"
#include "toolbox/matrix.h"
#include "user_math.h"
#include <string.h>

/* 静态变量 */
static robotics::Link* s_links = nullptr;
static robotics::Serial_Link<6>* s_robot = nullptr;
static bool s_initialized = false;

// 当前机械臂URDF(j1~j6)关节原始定义：parent->child 先 origin，再绕 joint axis 旋转。
// 这里显式按URDF链式连乘实现FK，避免将URDF joint origin强行映射到标准DH后产生几何失真。
static const float s_urdf_joint_origin_xyz[6][3] = {
    {0.0f,      0.0f,      0.024f},
    {-0.001395f,0.0f,      0.1015f},
    {0.3265f,   0.0f,     -0.0071975f},
    {0.0905f,   0.052775f, 0.0058025f},
    {0.001627f, 0.0f,      0.18467f},
    {0.10487f,  0.0013347f,0.0f},
};

// URDF顺序为(roll, pitch, yaw)。robotics::rpy2r输入顺序为(yaw, pitch, roll)。
static const float s_urdf_joint_origin_rpy[6][3] = {
    {0.0f,    0.0f,    0.0f},
    {1.5708f, 0.0f,   -1.5708f},
    {0.0f,    0.0f,    0.0f},
    {1.5708f, 0.0f,    3.1416f},
    {1.5708f, 0.0f,    0.0f},
    {1.5708f, 0.0f,    1.5708f},
};

// Link1~Link6 质心在各自link坐标系下的位置（直接取自 fixed_params.urdf 的 inertial/origin）。
static const float s_urdf_link_com_xyz[6][3] = {
    {-0.0042875f,  0.00018527f,  0.07765488f},
    { 0.22947f,   -0.0031632f,  -0.0022707f},
    { 0.08776404f,-0.00131433f, -0.02765140f},
    { 0.000030615f,-0.001914004f,0.13111748f},
    { 0.06854738f, 0.00130293f,  0.00052857f},
    {-0.0000059654f,0.000031386f,0.09366087f},
};

// 计算单个URDF关节的齐次变换：先固定origin，再绕本地z轴转动q。
static Matrixf<4, 4> Arm6dof_UrdfJointTransform(uint8_t index, float q) {
    const float roll  = s_urdf_joint_origin_rpy[index][0];
    const float pitch = s_urdf_joint_origin_rpy[index][1];
    const float yaw   = s_urdf_joint_origin_rpy[index][2];

    float ypr_origin[3] = {yaw, pitch, roll};
    Matrixf<3, 3> R_origin = robotics::rpy2r(Matrixf<3, 1>(ypr_origin));

    float p_raw[3] = {
        s_urdf_joint_origin_xyz[index][0],
        s_urdf_joint_origin_xyz[index][1],
        s_urdf_joint_origin_xyz[index][2],
    };
    Matrixf<3, 1> p_origin(p_raw);

    // 当前URDF关节轴均为 local-z：R_joint = RotZ(q)
    float ypr_joint[3] = {q, 0.0f, 0.0f};
    Matrixf<3, 3> R_joint = robotics::rpy2r(Matrixf<3, 1>(ypr_joint));

    return robotics::rp2t(R_origin, p_origin)
         * robotics::rp2t(R_joint, matrixf::zeros<3, 1>());
}

// 连乘前joint_num个关节，得到从基座到指定关节的URDF链式变换。
static Matrixf<4, 4> Arm6dof_UrdfForwardTransform(const Arm6dof_JointAngles_t* q,
                                                  uint8_t joint_num) {
    Matrixf<4, 4> T = matrixf::eye<4, 4>();
    for (uint8_t i = 0; i < joint_num; ++i) {
        T = T * Arm6dof_UrdfJointTransform(i, q->q[i]);
    }
    return T;
}

// 计算当前姿态的总重力势能，用于数值求重力补偿。
static float Arm6dof_UrdfPotentialEnergy(const Arm6dof_JointAngles_t* q) {
    constexpr float g = 9.81f;
    float U = 0.0f;

    Matrixf<4, 4> T_link = matrixf::eye<4, 4>();
    for (uint8_t i = 0; i < 6; ++i) {
        T_link = T_link * Arm6dof_UrdfJointTransform(i, q->q[i]);

        float p_com_raw[3] = {
            s_urdf_link_com_xyz[i][0],
            s_urdf_link_com_xyz[i][1],
            s_urdf_link_com_xyz[i][2],
        };
        Matrixf<3, 1> p_com_local(p_com_raw);
        Matrixf<4, 4> T_com = T_link * robotics::p2t(p_com_local);
        Matrixf<3, 1> p_com_world = robotics::t2p(T_com);

        U += s_links[i].m() * g * p_com_world[2][0];
    }

    return U;
}

// 计算目标末端位姿与当前末端位姿之间的6维误差（位置+姿态）。
static Matrixf<6, 1> Arm6dof_UrdfPoseError(const Matrixf<4, 4>& T_target,
                                           const Matrixf<4, 4>& T_cur) {
    Matrixf<6, 1> err;
    Matrixf<3, 1> pe = robotics::t2p(T_target) - robotics::t2p(T_cur);
    Matrixf<3, 1> we = robotics::t2twist(T_target * robotics::invT(T_cur)).block<3, 1>(3, 0);
    for (int k = 0; k < 3; ++k) {
        err[k][0] = pe[k][0];
        err[k + 3][0] = we[k][0];
    }
    return err;
}

static Matrixf<4, 4> Arm6dof_PoseToTransform(const Arm6dof_Pose_t* pose) {
    float rpy[3] = {pose->yaw, pose->pitch, pose->roll};
    float p[3] = {pose->x, pose->y, pose->z};
    return robotics::rp2t(robotics::rpy2r(Matrixf<3, 1>(rpy)), Matrixf<3, 1>(p));
}

static Arm6dof_Pose_t Arm6dof_TransformToPose(const Matrixf<4, 4>& T) {
    Arm6dof_Pose_t pose = {};
    Matrixf<3, 1> pos = robotics::t2p(T);
    Matrixf<3, 1> rpy = robotics::t2rpy(T);
    pose.x = pos[0][0];
    pose.y = pos[1][0];
    pose.z = pos[2][0];
    pose.yaw = rpy[0][0];
    pose.pitch = rpy[1][0];
    pose.roll = rpy[2][0];
    return pose;
}

static Matrixf<4, 4> Arm6dof_PoseDeltaToTransform(const Arm6dof_PoseDelta_t* delta) {
    float rpy[3] = {delta->dyaw, delta->dpitch, delta->droll};
    float p[3] = {delta->dx, delta->dy, delta->dz};
    return robotics::rp2t(robotics::rpy2r(Matrixf<3, 1>(rpy)), Matrixf<3, 1>(p));
}

// 通过数值微分计算当前关节姿态下的6x6雅可比矩阵。
static Matrixf<6, 6> Arm6dof_UrdfNumericJacobian(const Arm6dof_JointAngles_t* q) {
    Matrixf<6, 6> J = matrixf::zeros<6, 6>();
    const float eps = 1e-4f;

    for (int i = 0; i < 6; ++i) {
        Arm6dof_JointAngles_t q_plus = *q;
        Arm6dof_JointAngles_t q_minus = *q;
        q_plus.q[i] += eps;
        q_minus.q[i] -= eps;

        Matrixf<4, 4> T_plus = Arm6dof_UrdfForwardTransform(&q_plus, 6);
        Matrixf<4, 4> T_minus = Arm6dof_UrdfForwardTransform(&q_minus, 6);

        Matrixf<3, 1> p_plus = robotics::t2p(T_plus);
        Matrixf<3, 1> p_minus = robotics::t2p(T_minus);
        Matrixf<3, 1> dp = (p_plus - p_minus) / (2.0f * eps);

        Matrixf<3, 3> R_plus = robotics::t2r(T_plus);
        Matrixf<3, 3> R_minus = robotics::t2r(T_minus);
        Matrixf<3, 3> R_rel = R_plus * R_minus.trans();
        Matrixf<4, 1> angvec = robotics::r2angvec(R_rel);

        Matrixf<3, 1> w;
        w[0][0] = angvec[0][0] * angvec[3][0] / (2.0f * eps);
        w[1][0] = angvec[1][0] * angvec[3][0] / (2.0f * eps);
        w[2][0] = angvec[2][0] * angvec[3][0] / (2.0f * eps);

        J[0][i] = dp[0][0];
        J[1][i] = dp[1][0];
        J[2][i] = dp[2][0];
        J[3][i] = w[0][0];
        J[4][i] = w[1][0];
        J[5][i] = w[2][0];
    }

    return J;
}

// 将单个关节角限制在模型配置的上下限范围内。
static float Arm6dof_ClampToJointLimit(size_t idx, float qv) {
    if (qv < s_links[idx].qmin_) {
        qv = s_links[idx].qmin_;
    }
    if (qv > s_links[idx].qmax_) {
        qv = s_links[idx].qmax_;
    }
    return qv;
}

// 仅计算末端位置误差范数，用于解析候选解筛选。
static float Arm6dof_PositionErrorNorm(const Arm6dof_Pose_t* target,
                                       const Arm6dof_JointAngles_t* q) {
    Arm6dof_Pose_t cur = {};
    if (Arm6dof_ForwardKinematics(q, &cur) != 0) {
        return 1e9f;
    }
    const float ex = target->x - cur.x;
    const float ey = target->y - cur.y;
    const float ez = target->z - cur.z;
    return sqrtf(ex * ex + ey * ey + ez * ez);
}

// 求解3x3线性方程组，供降维位置IK内部使用。
static bool Arm6dof_Solve3x3(const float A[3][3], const float b[3], float x[3]) {
    float aug[3][4] = {
        {A[0][0], A[0][1], A[0][2], b[0]},
        {A[1][0], A[1][1], A[1][2], b[1]},
        {A[2][0], A[2][1], A[2][2], b[2]},
    };

    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        float max_abs = fabsf(aug[col][col]);
        for (int row = col + 1; row < 3; ++row) {
            const float v = fabsf(aug[row][col]);
            if (v > max_abs) {
                max_abs = v;
                pivot = row;
            }
        }
        if (max_abs < 1e-8f) {
            return false;
        }

        if (pivot != col) {
            for (int k = col; k < 4; ++k) {
                const float tmp = aug[col][k];
                aug[col][k] = aug[pivot][k];
                aug[pivot][k] = tmp;
            }
        }

        const float diag = aug[col][col];
        for (int k = col; k < 4; ++k) {
            aug[col][k] /= diag;
        }

        for (int row = 0; row < 3; ++row) {
            if (row == col) {
                continue;
            }
            const float factor = aug[row][col];
            for (int k = col; k < 4; ++k) {
                aug[row][k] -= factor * aug[col][k];
            }
        }
    }

    x[0] = aug[0][3];
    x[1] = aug[1][3];
    x[2] = aug[2][3];
    return true;
}

/**
 * @brief 初始化机械臂运动学模型
 */
void Arm6dof_Init(const Arm6dof_DHParams_t dh_params[6]) {
    if (dh_params == nullptr) {
        return;
    }
    
    // 创建DH链表
    if (s_links != nullptr) {
        delete[] s_links;
    }
    s_links = new robotics::Link[6];
    
    for (int i = 0; i < 6; i++) {
        // 构建质心向量（DH连杆坐标系下，由URDF经Rz(-theta_offset)旋转得到）
        Matrixf<3, 1> rc_vec;
        rc_vec[0][0] = dh_params[i].rc[0];
        rc_vec[1][0] = dh_params[i].rc[1];
        rc_vec[2][0] = dh_params[i].rc[2];

        s_links[i] = robotics::Link(
            dh_params[i].theta,         // 初始theta值（通常为0）
            dh_params[i].d,             // 连杆偏移 (m)
            dh_params[i].a,             // 连杆长度 (m)
            dh_params[i].alpha,         // 扭转角 (rad)
            robotics::R,                // 旋转关节
            dh_params[i].theta_offset,  // theta偏移 (rad)
            dh_params[i].qmin,          // 最小角度 (rad)
            dh_params[i].qmax,          // 最大角度 (rad)
            dh_params[i].m,             // 质量 (kg)
            rc_vec                      // 质心坐标（DH连杆坐标系，m）
        );
    }
    
    // 创建机器人模型
    if (s_robot != nullptr) {
        delete s_robot;
    }
    s_robot = new robotics::Serial_Link<6>(s_links);
    
    s_initialized = true;
}

/**
 * @brief 正运动学
 */
int Arm6dof_ForwardKinematics(const Arm6dof_JointAngles_t* q, Arm6dof_Pose_t* pose) {
    if (!s_initialized || q == nullptr || pose == nullptr) {
        return -1;
    }

    // 使用URDF原生链式FK：T = Π( T_origin_i * RotZ(q_i) )
    Matrixf<4, 4> T = Arm6dof_UrdfForwardTransform(q, 6);
    
    // 提取位置 (x, y, z)
    Matrixf<3, 1> pos = robotics::t2p(T);
    pose->x = pos[0][0];
    pose->y = pos[1][0];
    pose->z = pos[2][0];
    
    // 提取姿态 (RPY欧拉角)
    Matrixf<3, 1> rpy = robotics::t2rpy(T);
    pose->yaw   = rpy[0][0];
    pose->pitch = rpy[1][0];
    pose->roll  = rpy[2][0];
    
    return 0;
}

/**
 * @brief 获取指定关节的位姿
 */
int Arm6dof_GetJointPose(const Arm6dof_JointAngles_t* q, uint8_t joint_num, Arm6dof_Pose_t* pose) {
    if (!s_initialized || q == nullptr || pose == nullptr) {
        return -1;
    }
    
    if (joint_num < 1 || joint_num > 6) {
        return -1;
    }
    
    // 计算到第k个关节的URDF链式变换矩阵
    Matrixf<4, 4> T = Arm6dof_UrdfForwardTransform(q, joint_num);
    
    // 提取位置和姿态
    Matrixf<3, 1> pos = robotics::t2p(T);
    pose->x = pos[0][0];
    pose->y = pos[1][0];
    pose->z = pos[2][0];
    
    Matrixf<3, 1> rpy = robotics::t2rpy(T);
    pose->yaw   = rpy[0][0];
    pose->pitch = rpy[1][0];
    pose->roll  = rpy[2][0];
    
    return 0;
}


int Arm6dof_InverseKinematics_NumericalSolution(const Arm6dof_Pose_t* pose,
                                                const Arm6dof_JointAngles_t* q_init,
                                                Arm6dof_JointAngles_t* q_result,
                                                float tol,
                                                uint16_t max_iter) {
    if (!s_initialized || pose == nullptr || q_result == nullptr) {
        return -1;
    }
    
    // 构造目标变换矩阵
    Matrixf<3, 1> rpy_vec;
    rpy_vec[0][0] = pose->yaw;
    rpy_vec[1][0] = pose->pitch;
    rpy_vec[2][0] = pose->roll;
    Matrixf<3, 3> R_target = robotics::rpy2r(rpy_vec);
    
    Matrixf<3, 1> p_target;
    p_target[0][0] = pose->x;
    p_target[1][0] = pose->y;
    p_target[2][0] = pose->z;
    
    Matrixf<4, 4> T_target = robotics::rp2t(R_target, p_target);
    
    // 初始猜测值
    Matrixf<6, 1> q_guess = matrixf::zeros<6, 1>();
    if (q_init != nullptr) {
        for (int i = 0; i < 6; i++) {
            q_guess[i][0] = q_init->q[i];
        }
    }

    // 基于URDF链式FK的LM数值迭代，避免FK/IK模型不一致。
    Matrixf<6, 1> q_cur = q_guess;
    float lambda = 1.0f;
    constexpr float lambda_up = 10.0f;
    constexpr float lambda_down = 0.1f;
    constexpr float lambda_max = 1e6f;
    constexpr float lambda_min = 1e-6f;

    for (uint16_t iter = 0; iter < max_iter; ++iter) {
        Arm6dof_JointAngles_t q_cur_struct;
        for (int i = 0; i < 6; ++i) {
            q_cur_struct.q[i] = q_cur[i][0];
        }

        Matrixf<4, 4> T_cur = Arm6dof_UrdfForwardTransform(&q_cur_struct, 6);
        Matrixf<6, 1> err = Arm6dof_UrdfPoseError(T_target, T_cur);
        float err_norm = err.norm();
        if (err_norm < tol) {
            break;
        }

        Matrixf<6, 6> J = Arm6dof_UrdfNumericJacobian(&q_cur_struct);
        Matrixf<6, 6> H = J.trans() * J + lambda * matrixf::eye<6, 6>();
        Matrixf<6, 1> g = J.trans() * err;
        Matrixf<6, 1> dq = matrixf::inv(H) * g;

        if (dq[0][0] == INFINITY || dq[0][0] != dq[0][0]) {
            lambda *= lambda_up;
            if (lambda > lambda_max) {
                return -1;
            }
            continue;
        }

        Matrixf<6, 1> q_new = q_cur + dq;

        // 对循环关节折叠，避免等效角导致数值发散。
        for (int i = 0; i < 6; ++i) {
            constexpr float TWO_PI = 2.0f * (float)M_PI;
            bool is_cyclic = (s_links[i].qmin_ < 0.1f) && (s_links[i].qmax_ > TWO_PI - 0.1f);
            if (is_cyclic) {
                q_new[i][0] = math::loopLimit(q_new[i][0], s_links[i].qmin_, s_links[i].qmax_);
            }
        }

        Arm6dof_JointAngles_t q_new_struct;
        for (int i = 0; i < 6; ++i) {
            q_new_struct.q[i] = q_new[i][0];
        }

        Matrixf<4, 4> T_new = Arm6dof_UrdfForwardTransform(&q_new_struct, 6);
        Matrixf<6, 1> new_err = Arm6dof_UrdfPoseError(T_target, T_new);
        float new_err_norm = new_err.norm();

        if (new_err_norm < err_norm) {
            q_cur = q_new;
            lambda *= lambda_down;
            if (lambda < lambda_min) {
                lambda = lambda_min;
            }
        } else {
            lambda *= lambda_up;
            if (lambda > lambda_max) {
                return -1;
            }
        }
    }

    for (int i = 0; i < 6; ++i) {
        q_result->q[i] = q_cur[i][0];
    }

    // 验证解的精度：位置误差 + 姿态误差双重校验
    Arm6dof_JointAngles_t q_verify_struct;
    for (int i = 0; i < 6; ++i) {
        q_verify_struct.q[i] = q_cur[i][0];
    }
    Matrixf<4, 4> T_verify = Arm6dof_UrdfForwardTransform(&q_verify_struct, 6);
    Matrixf<3, 1> p_verify = robotics::t2p(T_verify);
    
    float pos_error = (p_verify - p_target).norm();
    
    if (pos_error > tol * 10.0f) {  // 位置误差过大，未收敛
        return -1;
    }
    
    // 姿态误差：计算两旋转矩阵的测地距离 theta = arccos((trace(R_v * R_t^T) - 1) / 2)
    // 使用旋转矩阵方法避免 RPY 奇异点和角度折叠问题
    Matrixf<3, 3> R_verify = robotics::t2r(T_verify);
    Matrixf<3, 3> R_diff   = R_verify * R_target.trans();
    float trace_val = R_diff[0][0] + R_diff[1][1] + R_diff[2][2];
    float cos_theta = (trace_val - 1.0f) * 0.5f;
    // 数值钳位防止 acosf 域溢出
    if (cos_theta >  1.0f) cos_theta =  1.0f;
    if (cos_theta < -1.0f) cos_theta = -1.0f;
    float ang_error = acosf(cos_theta);  // [0, π]
    
    const float ang_tol = 0.1f;  // 姿态收敛阈值 (rad ≈ 5.7°)
    if (ang_error > ang_tol) {   // 姿态未收敛（收敛到错误分支或近奇异点）
        return -1;
    }
    
    return 0;
}

// 枚举解析候选分支，并通过数值精修与去重得到全部有效解。
int Arm6dof_InverseKinematics_AnalyticalEnumerate(const Arm6dof_Pose_t* pose,
                                                  const Arm6dof_JointAngles_t* q_seed,
                                                  Arm6dof_JointAngles_t* q_solutions,
                                                  uint16_t max_solutions,
                                                  uint16_t* solution_count) {
    if (!s_initialized || pose == nullptr || q_solutions == nullptr ||
        solution_count == nullptr || max_solutions == 0) {
        return -1;
    }

    *solution_count = 0;

    Arm6dof_JointAngles_t seed = {};
    if (q_seed != nullptr) {
        seed = *q_seed;
    }

    Arm6dof_DHParams_t dh_params[6];
    for (int i = 0; i < 6; ++i) {
        dh_params[i].qmin = s_links[i].qmin_;
        dh_params[i].qmax = s_links[i].qmax_;
    }

    Arm6dof_JointAngles_t candidates[36];
    size_t candidate_count = 0;
    ArmMath_BuildAnalyticalSeedVariants(pose, q_seed, dh_params, candidates, &candidate_count);

    for (size_t i = 0; i < candidate_count; ++i) {
        Arm6dof_JointAngles_t q_tmp = {};
        if (Arm6dof_InverseKinematics_NumericalSolution(pose,
                                                        &candidates[i],
                                                        &q_tmp,
                                                        0.001f,
                                                        40) != 0) {
            continue;
        }

        const float err = Arm6dof_PositionErrorNorm(pose, &q_tmp);
        if (err > 0.002f) {
            continue;
        }

        bool duplicated = false;
        for (uint16_t k = 0; k < *solution_count; ++k) {
            if (ArmMath_IsSameSolution(&q_solutions[k], &q_tmp, 0.02f)) {
                duplicated = true;
                break;
            }
        }
        if (duplicated) {
            continue;
        }

        if (*solution_count >= max_solutions) {
            continue;
        }

        q_solutions[*solution_count] = q_tmp;
        ++(*solution_count);
    }

    return (*solution_count > 0) ? 0 : -1;
}

// 从多组解中选择离当前关节角最近的一组。
int Arm6dof_SelectNearestSolution(const Arm6dof_JointAngles_t* q_current,
                                  const Arm6dof_JointAngles_t* q_solutions,
                                  uint16_t solution_count,
                                  Arm6dof_JointAngles_t* q_result) {
    return ArmMath_SelectNearestSolution(q_current, q_solutions, solution_count, q_result);
}

// 对外解析IK接口：先枚举全部有效解，再按策略选最优解。
int Arm6dof_InverseKinematics_AnalyticalSolution(const Arm6dof_Pose_t* pose,
                                                 const Arm6dof_JointAngles_t* q_current,
                                                 Arm6dof_JointAngles_t* q_result) {
    if (!s_initialized || pose == nullptr || q_result == nullptr) {
        return -1;
    }

    Arm6dof_JointAngles_t solutions[36];
    uint16_t solution_count = 0;
    if (Arm6dof_InverseKinematics_AnalyticalEnumerate(
            pose, q_current, solutions, 36, &solution_count) != 0) {
        return -1;
    }

    return Arm6dof_SelectNearestSolution(q_current, solutions, solution_count, q_result);
}

// 文件内降维位置IK：优先调整J1~J3逼近目标位置，J4~J6保持seed。
int Arm6dof_InverseKinematicsPositionDecoupled(const Arm6dof_Pose_t* pose,
                                               const Arm6dof_JointAngles_t* q_seed,
                                               Arm6dof_JointAngles_t* q_result) {
    if (!s_initialized || pose == nullptr || q_result == nullptr) {
        return -1;
    }

    Arm6dof_JointAngles_t seed = {};
    if (q_seed != nullptr) {
        seed = *q_seed;
    }

    Arm6dof_JointAngles_t q = seed;
    constexpr float eps = 1e-3f;
    constexpr float lambda = 3e-3f;
    constexpr float max_joint_step = 0.12f;
    constexpr int max_iter = 20;
    constexpr float pos_tol = 0.015f;
    constexpr float fallback_tol = 0.05f;

    float best_err = 1e9f;
    Arm6dof_JointAngles_t best_q = q;

    for (int iter = 0; iter < max_iter; ++iter) {
        Arm6dof_Pose_t cur_pose;
        if (Arm6dof_ForwardKinematics(&q, &cur_pose) != 0) {
            break;
        }

        const float ex = pose->x - cur_pose.x;
        const float ey = pose->y - cur_pose.y;
        const float ez = pose->z - cur_pose.z;
        const float err_norm = sqrtf(ex * ex + ey * ey + ez * ez);

        if (err_norm < best_err) {
            best_err = err_norm;
            best_q = q;
        }
        if (err_norm <= pos_tol) {
            break;
        }

        float J[3][3];
        for (int j = 0; j < 3; ++j) {
            Arm6dof_JointAngles_t q_pert = q;
            q_pert.q[j] += eps;
            Arm6dof_Pose_t pose_pert;
            if (Arm6dof_ForwardKinematics(&q_pert, &pose_pert) != 0) {
                return -1;
            }
            J[0][j] = (pose_pert.x - cur_pose.x) / eps;
            J[1][j] = (pose_pert.y - cur_pose.y) / eps;
            J[2][j] = (pose_pert.z - cur_pose.z) / eps;
        }

        float H[3][3] = {{0.0f}};
        float g[3] = {0.0f};
        const float e[3] = {ex, ey, ez};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                H[r][c] = J[0][r] * J[0][c] + J[1][r] * J[1][c] + J[2][r] * J[2][c];
            }
            H[r][r] += lambda;
            g[r] = J[0][r] * e[0] + J[1][r] * e[1] + J[2][r] * e[2];
        }

        float dq[3] = {0.0f};
        if (!Arm6dof_Solve3x3(H, g, dq)) {
            break;
        }

        for (int j = 0; j < 3; ++j) {
            if (dq[j] > max_joint_step) {
                dq[j] = max_joint_step;
            }
            if (dq[j] < -max_joint_step) {
                dq[j] = -max_joint_step;
            }
            q.q[j] = Arm6dof_ClampToJointLimit((size_t)j, q.q[j] + dq[j]);
        }
    }

    if (best_err > fallback_tol) {
        return -1;
    }

    best_q.q[3] = seed.q[3];
    best_q.q[4] = seed.q[4];
    best_q.q[5] = seed.q[5];

    for (size_t i = 0; i < 6; ++i) {
        best_q.q[i] = Arm6dof_ClampToJointLimit(i, best_q.q[i]);
    }

    *q_result = best_q;
    return 0;
}

// 对外位置降维IK接口。
int Arm6dof_InverseKinematics_PositionDecoupled(const Arm6dof_Pose_t* pose,
                                                const Arm6dof_JointAngles_t* q_seed,
                                                Arm6dof_JointAngles_t* q_result) {
    return Arm6dof_InverseKinematicsPositionDecoupled(pose, q_seed, q_result);
}

// 将世界系/工具系/航向系增量叠加到当前位姿，得到新的世界系目标位姿。
int Arm6dof_ApplyPoseDelta(const Arm6dof_Pose_t* current_world,
                           const Arm6dof_PoseDelta_t* delta_in_frame,
                           Arm6dof_ControlFrame_t frame,
                           Arm6dof_Pose_t* target_world) {
    if (current_world == nullptr || delta_in_frame == nullptr || target_world == nullptr) {
        return -1;
    }

    Matrixf<4, 4> T_cur = Arm6dof_PoseToTransform(current_world);
    Matrixf<4, 4> T_delta = Arm6dof_PoseDeltaToTransform(delta_in_frame);
    Matrixf<4, 4> T_target = T_cur;

    if (frame == ARM6DOF_FRAME_TOOL) {
        T_target = T_cur * T_delta;
    } else if (frame == ARM6DOF_FRAME_HEADING) {
        float yaw_only[3] = {current_world->yaw, 0.0f, 0.0f};
        Matrixf<4, 4> T_heading = robotics::rp2t(robotics::rpy2r(Matrixf<3, 1>(yaw_only)),
                                                 matrixf::zeros<3, 1>());
        T_target = T_heading * T_delta * robotics::invT(T_heading) * T_cur;
    } else {
        T_target = T_delta * T_cur;
    }

    *target_world = Arm6dof_TransformToPose(T_target);
    return 0;
}

// 将当前世界系位姿同步回指定控制坐标系，避免切换控制模式时跳变。
int Arm6dof_SyncWorldPoseToFrameTarget(const Arm6dof_Pose_t* current_world,
                                       Arm6dof_ControlFrame_t frame,
                                       Arm6dof_Pose_t* target_in_frame) {
    if (current_world == nullptr || target_in_frame == nullptr) {
        return -1;
    }

    if (frame == ARM6DOF_FRAME_TOOL) {
        float rpy[3] = {current_world->yaw, current_world->pitch, current_world->roll};
        Matrixf<3, 3> RT = robotics::rpy2r(Matrixf<3, 1>(rpy)).trans();
        float p[3] = {current_world->x, current_world->y, current_world->z};
        Matrixf<3, 1> pt = RT * Matrixf<3, 1>(p);
        target_in_frame->x = pt[0][0];
        target_in_frame->y = pt[1][0];
        target_in_frame->z = pt[2][0];
        target_in_frame->yaw = 0.0f;
        target_in_frame->pitch = 0.0f;
        target_in_frame->roll = 0.0f;
        return 0;
    }

    if (frame == ARM6DOF_FRAME_HEADING) {
        float cy = cosf(current_world->yaw);
        float sy = sinf(current_world->yaw);
        target_in_frame->x = cy * current_world->x + sy * current_world->y;
        target_in_frame->y = -sy * current_world->x + cy * current_world->y;
        target_in_frame->z = current_world->z;

        float rpy_neg[3] = {-current_world->yaw, 0.0f, 0.0f};
        float rpy_c[3] = {current_world->yaw, current_world->pitch, current_world->roll};
        Matrixf<3, 3> Rn = robotics::rpy2r(Matrixf<3, 1>(rpy_neg))
                         * robotics::rpy2r(Matrixf<3, 1>(rpy_c));
        Matrixf<3, 1> rn = robotics::r2rpy(Rn);
        target_in_frame->yaw = rn[0][0];
        target_in_frame->pitch = rn[1][0];
        target_in_frame->roll = rn[2][0];
        return 0;
    }

    *target_in_frame = *current_world;
    return 0;
}

// 轨迹推进一步：插值目标位姿、执行IK、检查连续性与限位。
int Arm6dof_TrajectoryStep(Arm6dof_TrajectoryState_t* state,
                           const Arm6dof_TrajectoryConfig_t* cfg,
                           Arm6dof_IkMode_t ik_mode,
                           const Arm6dof_JointAngles_t* q_current,
                           Arm6dof_Pose_t* interp_pose,
                           float dt) {
    if (!s_initialized || state == nullptr || cfg == nullptr || q_current == nullptr || interp_pose == nullptr) {
        return -1;
    }
    if (!state->active) {
        return 0;
    }

    constexpr float TWO_PI = 2.0f * (float)M_PI;

    float tdx = state->goal.x - state->start.x;
    float tdy = state->goal.y - state->start.y;
    float tdz = state->goal.z - state->start.z;
    float total_dist = sqrtf(tdx * tdx + tdy * tdy + tdz * tdz);

    float tdr = state->goal.roll - state->start.roll;
    float tdp = state->goal.pitch - state->start.pitch;
    float tdyw = state->goal.yaw - state->start.yaw;
    tdr -= roundf(tdr / TWO_PI) * TWO_PI;
    tdp -= roundf(tdp / TWO_PI) * TWO_PI;
    tdyw -= roundf(tdyw / TWO_PI) * TWO_PI;
    float total_ang = sqrtf(tdr * tdr + tdp * tdp + tdyw * tdyw);

    float dt_p = (total_dist > 1e-6f) ? (cfg->max_lin_vel * dt / total_dist) : 1.0f;
    float dt_a = (total_ang > 1e-6f) ? (cfg->max_ang_vel * dt / total_ang) : 1.0f;
    float dt_step = dt_p < dt_a ? dt_p : dt_a;
    if (dt_step > 1.0f) {
        dt_step = 1.0f;
    }

    state->t += dt_step;
    if (state->t >= 1.0f) {
        state->t = 1.0f;
        state->active = false;
    }

    Arm6dof_Pose_t interp;
    interp.x = state->start.x + state->t * tdx;
    interp.y = state->start.y + state->t * tdy;
    interp.z = state->start.z + state->t * tdz;
    interp.roll = state->start.roll + state->t * tdr;
    interp.pitch = state->start.pitch + state->t * tdp;
    interp.yaw = state->start.yaw + state->t * tdyw;
    *interp_pose = interp;

    Arm6dof_JointAngles_t q_init;
    if (state->valid) {
        q_init = state->angles;
    } else {
        q_init = *q_current;
    }

    bool ik_success = false;
    if (ik_mode == ARM6DOF_IK_POSITION_DECOUPLED) {
        ik_success = (Arm6dof_InverseKinematicsPositionDecoupled(&interp, &q_init, &state->angles) == 0);
    } else {
        ik_success = (Arm6dof_InverseKinematics_NumericalSolution(&interp,
                                                                  &q_init,
                                                                  &state->angles,
                                                                  cfg->ik_tol,
                                                                  cfg->ik_max_iter) == 0);
    }

    if (!ik_success) {
        state->valid = false;
        state->active = false;
        return -1;
    }

    for (size_t i = 0; i < 6; ++i) {
        if ((s_links[i].qmax_ - s_links[i].qmin_) <= TWO_PI + 0.1f) {
            continue;
        }
        float diff = state->angles.q[i] - q_init.q[i];
        diff -= roundf(diff / TWO_PI) * TWO_PI;
        state->angles.q[i] = q_init.q[i] + diff;
    }

    for (size_t i = 0; i < 6; ++i) {
        if (state->angles.q[i] < (s_links[i].qmin_ - cfg->joint_limit_tolerance) ||
            state->angles.q[i] > (s_links[i].qmax_ + cfg->joint_limit_tolerance)) {
            state->valid = false;
            state->active = false;
            return -1;
        }
    }

    for (size_t i = 0; i < 6; ++i) {
        float delta = fabsf(state->angles.q[i] - q_init.q[i]);
        if (delta > cfg->max_joint_delta) {
            state->valid = false;
            state->active = false;
            return -1;
        }
    }

    state->valid = true;
    return 0;
}

/**
 * @brief 计算重力补偿力矩
 * @details 与当前URDF链式运动学保持一致：通过势能 U(q)=Σ(mgz) 的数值梯度
 *          τg = ∂U/∂q 计算重力补偿力矩。
 */
int Arm6dof_GravityCompensation(const Arm6dof_JointAngles_t* q, float gravity_torques[6]) {
    if (!s_initialized || q == nullptr || gravity_torques == nullptr) {
        return -1;
    }
    
    // // 关节角度
    // Matrixf<6, 1> q_mat;
    // for (int i = 0; i < 6; i++) {
    //     q_mat[i][0] = q->q[i];
    // }
    
    // // 速度和加速度均为零 —— 只计算重力项
    // Matrixf<6, 1> qv = matrixf::zeros<6, 1>();
    // Matrixf<6, 1> qa = matrixf::zeros<6, 1>();
    
    // // 调用牛顿-欧拉逆动力学
    // Matrixf<6, 1> torques = s_robot->rne(q_mat, qv, qa);
    
    // for (int i = 0; i < 6; i++) {
    //     gravity_torques[i] = torques[i][0];
    // }

        constexpr float eps = 1e-4f;
    for (int i = 0; i < 6; ++i) {
        Arm6dof_JointAngles_t q_plus = *q;
        Arm6dof_JointAngles_t q_minus = *q;
        q_plus.q[i] += eps;
        q_minus.q[i] -= eps;

        float U_plus = Arm6dof_UrdfPotentialEnergy(&q_plus);
        float U_minus = Arm6dof_UrdfPotentialEnergy(&q_minus);
        gravity_torques[i] = (U_plus - U_minus) / (2.0f * eps);
    }
    
    return 0;
}
