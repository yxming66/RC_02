#ifndef ARM_LIB_H
#define ARM_LIB_H

#include "arm_lib_config.h"

#include "core/arm_common.h"
#include "core/arm_limits.h"
#include "core/arm_status.h"
#include "core/arm_types.h"

#include "adapter/toolbox_adapter.h"

#include "model/link.h"
#include "model/tool_frame.h"
#include "model/serial_chain.h"

#if ARM_LIB_ENABLE_DYNAMICS
#include "dynamics/gravity.h"
#endif

#include "kinematics/fk.h"
#include "kinematics/ik.h"
#include "kinematics/ik_dispatch.h"
#include "kinematics/jacobian.h"
#include "kinematics/ik_redundancy.h"
#include "kinematics/ik_numeric.h"
#include "kinematics/pose_error.h"
#include "kinematics/task_constraints.h"

#if ARM_LIB_ENABLE_ANALYTIC_IK
#include "kinematics/ik_analytic.h"
#include "kinematics/ik_3r_planar.h"
#include "kinematics/analytic_solvers/solver_registry.h"
#endif

#include "solver/lm_solver.h"
#include "solver/nr_solver.h"
#include "solver/seed_policy.h"

#include "parser/chain_builder.h"
#include "parser/dh_loader.h"
#include "parser/urdf_loader.h"

#if ARM_LIB_ENABLE_TRAJECTORY
#include "trajectory/interpolation.h"
#include "trajectory/joint_traj.h"
#include "trajectory/cartesian_traj.h"
#endif

#if ARM_LIB_ENABLE_CONTROL
#include "control/cartesian_control_types.h"
#include "control/joint_servo.h"
#include "control/cartesian_servo.h"
#endif

#endif
