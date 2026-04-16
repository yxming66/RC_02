#pragma once

#include "arm6dof.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

float ArmMath_WrapNear(float value, float reference);
float ArmMath_AngleDistance(float a, float b);
float ArmMath_JointDistance(const Arm6dof_JointAngles_t* lhs,
                            const Arm6dof_JointAngles_t* rhs);
bool ArmMath_IsSameSolution(const Arm6dof_JointAngles_t* lhs,
                            const Arm6dof_JointAngles_t* rhs,
                            float tol);
void ArmMath_BuildAnalyticalSeedVariants(const Arm6dof_Pose_t* pose,
                                         const Arm6dof_JointAngles_t* q_seed,
                                         const Arm6dof_DHParams_t dh_params[6],
                                         Arm6dof_JointAngles_t variants[],
                                         size_t* variant_count);
int ArmMath_SelectNearestSolution(const Arm6dof_JointAngles_t* q_current,
                                  const Arm6dof_JointAngles_t* q_solutions,
                                  uint16_t solution_count,
                                  Arm6dof_JointAngles_t* q_result);

#ifdef __cplusplus
}
#endif
