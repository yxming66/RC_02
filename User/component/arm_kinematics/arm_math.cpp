#include "arm_math.h"
#include "user_math.h"

static float ArmMath_ClampToJointLimit(const Arm6dof_DHParams_t dh_params[6],
                                       size_t idx,
                                       float qv) {
    if (qv < dh_params[idx].qmin) {
        qv = dh_params[idx].qmin;
    }
    if (qv > dh_params[idx].qmax) {
        qv = dh_params[idx].qmax;
    }
    return qv;
}

float ArmMath_WrapNear(float value, float reference) {
    float diff = value - reference;
    diff -= roundf(diff / M_2PI) * M_2PI;
    return reference + diff;
}

float ArmMath_AngleDistance(float a, float b) {
    float diff = a - b;
    diff -= roundf(diff / M_2PI) * M_2PI;
    return fabsf(diff);
}

float ArmMath_JointDistance(const Arm6dof_JointAngles_t* lhs,
                            const Arm6dof_JointAngles_t* rhs) {
    float sum = 0.0f;
    for (int i = 0; i < 6; ++i) {
        const float d = ArmMath_AngleDistance(lhs->q[i], rhs->q[i]);
        sum += d * d;
    }
    return sqrtf(sum);
}

bool ArmMath_IsSameSolution(const Arm6dof_JointAngles_t* lhs,
                            const Arm6dof_JointAngles_t* rhs,
                            float tol) {
    for (int i = 0; i < 6; ++i) {
        if (ArmMath_AngleDistance(lhs->q[i], rhs->q[i]) > tol) {
            return false;
        }
    }
    return true;
}

void ArmMath_BuildAnalyticalSeedVariants(const Arm6dof_Pose_t* pose,
                                         const Arm6dof_JointAngles_t* q_seed,
                                         const Arm6dof_DHParams_t dh_params[6],
                                         Arm6dof_JointAngles_t variants[],
                                         size_t* variant_count) {
    *variant_count = 0;

    Arm6dof_JointAngles_t base = {};
    if (q_seed != nullptr) {
        base = *q_seed;
    }

    const float planar_r = sqrtf(pose->x * pose->x + pose->y * pose->y);
    float q1_candidates[4] = {
        atan2f(pose->y, pose->x),
        atan2f(pose->y, pose->x) + M_PI,
        base.q[0],
        (planar_r < 1e-5f) ? base.q[0] : atan2f(pose->y, pose->x) - M_PI,
    };

    const float q2_bias = (pose->z >= 0.20f) ? 0.35f : -0.35f;
    const float q3_bias = (planar_r >= 0.28f) ? 0.85f : 1.65f;
    const float q2_candidates[3] = {
        base.q[1],
        q2_bias,
        -q2_bias,
    };
    const float q3_candidates[3] = {
        base.q[2],
        q3_bias,
        -0.5f * q3_bias,
    };

    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            for (size_t k = 0; k < 3; ++k) {
                Arm6dof_JointAngles_t q = base;
                q.q[0] = ArmMath_ClampToJointLimit(dh_params, 0, ArmMath_WrapNear(q1_candidates[i], base.q[0]));
                q.q[1] = ArmMath_ClampToJointLimit(dh_params, 1, ArmMath_WrapNear(q2_candidates[j], base.q[1]));
                q.q[2] = ArmMath_ClampToJointLimit(dh_params, 2, ArmMath_WrapNear(q3_candidates[k], base.q[2]));
                variants[*variant_count] = q;
                ++(*variant_count);
            }
        }
    }
}

int ArmMath_SelectNearestSolution(const Arm6dof_JointAngles_t* q_current,
                                  const Arm6dof_JointAngles_t* q_solutions,
                                  uint16_t solution_count,
                                  Arm6dof_JointAngles_t* q_result) {
    if (q_solutions == nullptr || q_result == nullptr || solution_count == 0) {
        return -1;
    }

    if (q_current == nullptr) {
        *q_result = q_solutions[0];
        return 0;
    }

    uint16_t best_index = 0;
    float best_dist = ArmMath_JointDistance(q_current, &q_solutions[0]);
    for (uint16_t i = 1; i < solution_count; ++i) {
        const float dist = ArmMath_JointDistance(q_current, &q_solutions[i]);
        if (dist < best_dist) {
            best_dist = dist;
            best_index = i;
        }
    }

    *q_result = q_solutions[best_index];
    return 0;
}
