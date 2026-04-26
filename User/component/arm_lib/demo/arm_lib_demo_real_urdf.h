#ifndef ARM_LIB_DEMO_ARM_LIB_DEMO_REAL_URDF_H
#define ARM_LIB_DEMO_ARM_LIB_DEMO_REAL_URDF_H

#include "../arm_lib.h"

namespace arm_lib {
namespace demo {

struct DemoRealUrdfReport {
  SerialChain<4> full_chain;
  SerialChain<3> movable_chain;
  JointVec<4> full_q;
  JointVec<3> movable_q;
  Transform full_pose;
  Transform movable_pose;
  JointVec<4> full_gravity;
  JointVec<3> movable_gravity;
  JointVec<3> hybrid_q_solution;
  Scalar fk_position_error;
  Scalar fk_orientation_error;
  Scalar gravity_error_norm;
  Scalar hybrid_position_error;
  Scalar hybrid_orientation_error;
  Scalar link1_mass_error;
  Scalar link2_mass_error;
  Scalar link3_mass_error;
  kinematics::IKResult<3> hybrid_ik_result;
  bool full_parse_ok;
  bool movable_parse_ok;
  bool hybrid_ik_valid;

  DemoRealUrdfReport()
      : full_chain(),
        movable_chain(),
        full_q(toolbox_adapter::zero_joint_vec<4>()),
        movable_q(toolbox_adapter::zero_joint_vec<3>()),
        full_pose(toolbox_adapter::identity_transform()),
        movable_pose(toolbox_adapter::identity_transform()),
        full_gravity(toolbox_adapter::zero_joint_vec<4>()),
        movable_gravity(toolbox_adapter::zero_joint_vec<3>()),
        hybrid_q_solution(toolbox_adapter::zero_joint_vec<3>()),
        fk_position_error(0.0f),
        fk_orientation_error(0.0f),
        gravity_error_norm(0.0f),
        hybrid_position_error(0.0f),
        hybrid_orientation_error(0.0f),
        link1_mass_error(0.0f),
        link2_mass_error(0.0f),
        link3_mass_error(0.0f),
        hybrid_ik_result(),
        full_parse_ok(false),
        movable_parse_ok(false),
        hybrid_ik_valid(false) {}
};

DemoRealUrdfReport run_demo_real_urdf();
bool run_demo_real_urdf_success();

}  // namespace demo
}  // namespace arm_lib

#endif
