#include "arm_lib_demo_urdf.h"

#include <cmath>

namespace arm_lib {
namespace demo {
namespace {

const char kPlanarUrdf[] =
    "<robot name=\"planar_3r\">"
    "<link name=\"base\"/>"
    "<link name=\"link1\">"
    "<inertial>"
    "<origin xyz=\"0.11 0 0\" rpy=\"0 0 0\"/>"
    "<mass value=\"0.60\"/>"
    "<inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/>"
    "</inertial>"
    "</link>"
    "<link name=\"link2\">"
    "<inertial>"
    "<origin xyz=\"0.09 0 0\" rpy=\"0 0 0\"/>"
    "<mass value=\"0.45\"/>"
    "<inertia ixx=\"0.02\" ixy=\"0\" ixz=\"0\" iyy=\"0.02\" iyz=\"0\" izz=\"0.02\"/>"
    "</inertial>"
    "</link>"
    "<link name=\"link3\">"
    "<inertial>"
    "<origin xyz=\"0.06 0 0\" rpy=\"0 0 0\"/>"
    "<mass value=\"0.25\"/>"
    "<inertia ixx=\"0.03\" ixy=\"0\" ixz=\"0\" iyy=\"0.03\" iyz=\"0\" izz=\"0.03\"/>"
    "</inertial>"
    "</link>"
    "<link name=\"tool\">"
    "<inertial>"
    "<origin xyz=\"0.01 0 0\" rpy=\"0 0 0\"/>"
    "<mass value=\"0.05\"/>"
    "<inertia ixx=\"0.001\" ixy=\"0\" ixz=\"0\" iyy=\"0.001\" iyz=\"0\" izz=\"0.001\"/>"
    "</inertial>"
    "</link>"
    "<joint name=\"j1\" type=\"revolute\">"
    "<parent link=\"base\"/>"
    "<child link=\"link1\"/>"
    "<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
    "<axis xyz=\"0 0 1\"/>"
    "<limit lower=\"-3.14159\" upper=\"3.14159\"/>"
    "</joint>"
    "<joint name=\"j2\" type=\"revolute\">"
    "<parent link=\"link1\"/>"
    "<child link=\"link2\"/>"
    "<origin xyz=\"0.22 0 0\" rpy=\"0 0 0\"/>"
    "<axis xyz=\"0 0 1\"/>"
    "<limit lower=\"-3.14159\" upper=\"3.14159\"/>"
    "</joint>"
    "<joint name=\"j3\" type=\"revolute\">"
    "<parent link=\"link2\"/>"
    "<child link=\"link3\"/>"
    "<origin xyz=\"0.18 0 0\" rpy=\"0 0 0\"/>"
    "<axis xyz=\"0 0 1\"/>"
    "<limit lower=\"-3.14159\" upper=\"3.14159\"/>"
    "</joint>"
    "<joint name=\"tool_fixed\" type=\"fixed\">"
    "<parent link=\"link3\"/>"
    "<child link=\"tool\"/>"
    "<origin xyz=\"0.12 0 0\" rpy=\"0 0 0\"/>"
    "</joint>"
    "</robot>";

}  // namespace

DemoUrdfReport run_demo_urdf() {
  DemoUrdfReport report;
  report.full_parse_ok = parser::load_chain_from_urdf<4>(
      kPlanarUrdf, "base", "tool", &report.full_chain);
  report.movable_parse_ok = parser::load_movable_chain_from_urdf<3>(
      kPlanarUrdf, "base", "tool", &report.movable_chain);
  if (!report.full_parse_ok || !report.movable_parse_ok) {
    return report;
  }

  report.full_q[0][0] = 0.35f;
  report.full_q[1][0] = -0.55f;
  report.full_q[2][0] = 0.45f;
  report.full_q[3][0] = 0.0f;
  report.movable_q[0][0] = 0.35f;
  report.movable_q[1][0] = -0.55f;
  report.movable_q[2][0] = 0.45f;
  report.full_pose = kinematics::fk(report.full_chain, report.full_q);
  report.movable_pose = kinematics::fk(report.movable_chain, report.movable_q);

  const Scalar expected_x = 0.22f * cosf(0.35f) + 0.18f * cosf(-0.20f) +
                            0.12f * cosf(0.25f);
  const Scalar expected_y = 0.22f * sinf(0.35f) + 0.18f * sinf(-0.20f) +
                            0.12f * sinf(0.25f);
  const Vec3 full_ypr = toolbox_adapter::rpy_from_rotation(
      toolbox_adapter::rotation_of(report.full_pose));
  const Vec3 movable_ypr = toolbox_adapter::rpy_from_rotation(
      toolbox_adapter::rotation_of(report.movable_pose));

  report.full_x_error = abs_scalar(report.full_pose[0][3] - expected_x);
  report.full_y_error = abs_scalar(report.full_pose[1][3] - expected_y);
  report.full_yaw_error = abs_scalar(full_ypr[0][0] - 0.25f);
  report.movable_x_error = abs_scalar(report.movable_pose[0][3] - expected_x);
  report.movable_y_error = abs_scalar(report.movable_pose[1][3] - expected_y);
  report.movable_yaw_error = abs_scalar(movable_ypr[0][0] - 0.25f);
  report.full_mass_error =
      abs_scalar(report.full_chain.link(0).mass() - 0.60f) +
      abs_scalar(report.full_chain.link(1).mass() - 0.45f) +
      abs_scalar(report.full_chain.link(2).mass() - 0.25f) +
      abs_scalar(report.full_chain.link(3).mass() - 0.05f);
  report.movable_mass_error =
      abs_scalar(report.movable_chain.link(0).mass() - 0.60f) +
      abs_scalar(report.movable_chain.link(1).mass() - 0.45f) +
      abs_scalar(report.movable_chain.link(2).mass() - 0.25f);
  report.movable_com_error =
      abs_scalar(report.movable_chain.link(0).com()[0][0] - 0.11f) +
      abs_scalar(report.movable_chain.link(1).com()[0][0] - 0.09f) +
      abs_scalar(report.movable_chain.link(2).com()[0][0] - 0.06f);
  return report;
}

bool run_demo_urdf_success() {
  const DemoUrdfReport report = run_demo_urdf();
  return report.full_parse_ok && report.movable_parse_ok &&
         report.full_x_error <= 1.0e-4f && report.full_y_error <= 1.0e-4f &&
         report.full_yaw_error <= 1.0e-4f &&
         report.movable_x_error <= 1.0e-4f &&
         report.movable_y_error <= 1.0e-4f &&
         report.movable_yaw_error <= 1.0e-4f &&
         report.full_mass_error <= 1.0e-5f &&
         report.movable_mass_error <= 1.0e-5f &&
         report.movable_com_error <= 1.0e-5f;
}

}  // namespace demo
}  // namespace arm_lib
