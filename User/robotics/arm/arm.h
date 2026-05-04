#ifndef ARM_LIB_H
#define ARM_LIB_H

#include "arm_config.h"

#include "core/arm_common.h"
#include "core/arm_command.h"
#include "core/arm_limits.h"
#include "core/arm_status.h"
#include "core/arm_types.h"

#include "adapter/toolbox_adapter.h"

#if ARM_LIB_ENABLE_MATH
#include "math/linear_solve.h"
#endif

#include "model/link.h"
#include "model/tool_frame.h"
#include "model/serial_chain.h"
#include "model/chain_builder.h"

#if ARM_LIB_ENABLE_DYNAMICS
#include "dynamics/gravity.h"
#endif

#if ARM_LIB_ENABLE_FRAMES
#include "frames/arm_frames.h"
#endif

#if ARM_LIB_ENABLE_SAFETY
#include "safety/joint_command_safety.h"
#endif

#include "kinematics/fk.h"
#include "kinematics/ik.h"
#include "kinematics/ik_dispatch.h"
#include "kinematics/jacobian.h"
#include "kinematics/ik_redundancy.h"
#include "kinematics/ik_solution_scoring.h"
#include "kinematics/ik_numeric.h"
#include "kinematics/pose_error.h"
#include "kinematics/task_constraints.h"

#if ARM_LIB_ENABLE_ANALYTIC_IK
#include "kinematics/ik_analytic.h"
#include "kinematics/analytic_solvers/solver_registry.h"
#endif

#include "solver/lm_solver.h"
#include "solver/nr_solver.h"
#include "solver/seed_policy.h"

#if ARM_LIB_ENABLE_PLANNING
#include "planning/move_j_planner.h"
#include "planning/move_l_planner.h"
#endif

#if ARM_LIB_ENABLE_SERVO
#include "servo/weighted_dls.h"
#include "servo/servo_l_controller.h"
#endif

#include "controller/arm_controller.h"

#endif
