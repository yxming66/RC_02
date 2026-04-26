#ifndef ARM_LIB_DEMO_ARM_LIB_DEMO_URDF_H
#define ARM_LIB_DEMO_ARM_LIB_DEMO_URDF_H

#include "../arm_lib.h"

namespace arm_lib {
namespace demo {

struct DemoUrdfReport {
  SerialChain<4> full_chain;
  SerialChain<3> movable_chain;
  JointVec<4> full_q;
  JointVec<3> movable_q;
  Transform full_pose;
  Transform movable_pose;
  Scalar full_x_error;
  Scalar full_y_error;
  Scalar full_yaw_error;
  Scalar movable_x_error;
  Scalar movable_y_error;
  Scalar movable_yaw_error;
  Scalar full_mass_error;
  Scalar movable_mass_error;
  Scalar movable_com_error;
  bool full_parse_ok;
  bool movable_parse_ok;

  DemoUrdfReport()
      : full_chain(),
        movable_chain(),
        full_q(toolbox_adapter::zero_joint_vec<4>()),
        movable_q(toolbox_adapter::zero_joint_vec<3>()),
        full_pose(toolbox_adapter::identity_transform()),
        movable_pose(toolbox_adapter::identity_transform()),
        full_x_error(0.0f),
        full_y_error(0.0f),
        full_yaw_error(0.0f),
        movable_x_error(0.0f),
        movable_y_error(0.0f),
        movable_yaw_error(0.0f),
        full_mass_error(0.0f),
        movable_mass_error(0.0f),
        movable_com_error(0.0f),
        full_parse_ok(false),
        movable_parse_ok(false) {}
};

DemoUrdfReport run_demo_urdf();
bool run_demo_urdf_success();

}  // namespace demo
}  // namespace arm_lib

#endif
