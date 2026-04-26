#include "arm_lib_demo_real_urdf.h"

namespace arm_lib {
namespace demo {
namespace {

const char kRealUrdf[] =
    "<robot name=\"armurdf_standard_motor\">"
    "<link name=\"base_link\">"
    "<inertial>"
    "<origin xyz=\"0.06401797 0.00093364 0.06018482\" rpy=\"0 0 0\"/>"
    "<mass value=\"1.19404\"/>"
    "<inertia ixx=\"0.00054518\" ixy=\"-8.8651E-07\" ixz=\"0\" iyy=\"0.00099237\" iyz=\"3.2702E-08\" izz=\"0.0011813\"/>"
    "</inertial>"
    "</link>"
    "<link name=\"Link1\">"
    "<inertial>"
    "<origin xyz=\"-0.00047343 0.28954791 0.0005662\" rpy=\"0 0 0\"/>"
    "<mass value=\"0.54717\"/>"
    "<inertia ixx=\"0.0023759\" ixy=\"3.3021E-06\" ixz=\"0\" iyy=\"6.4486E-05\" iyz=\"0\" izz=\"0.0024336\"/>"
    "</inertial>"
    "</link>"
    "<link name=\"Link2\">"
    "<inertial>"
    "<origin xyz=\"-0.0003464 0.2219719 0.00658453\" rpy=\"0 0 0\"/>"
    "<mass value=\"0.5273\"/>"
    "<inertia ixx=\"0.00075992\" ixy=\"1.0161E-06\" ixz=\"0\" iyy=\"4.9865E-05\" iyz=\"0\" izz=\"0.00078302\"/>"
    "</inertial>"
    "</link>"
    "<link name=\"Link3\">"
    "<inertial>"
    "<origin xyz=\"0.002693 0.11635 0.0019852\" rpy=\"0 0 0\"/>"
    "<mass value=\"0.24563\"/>"
    "<inertia ixx=\"0.00013404\" ixy=\"1.2628E-06\" ixz=\"6.3612E-09\" iyy=\"0.0001778\" iyz=\"-1.8653E-10\" izz=\"0.00013348\"/>"
    "</inertial>"
    "</link>"
    "<link name=\"tool_Link\">"
    "<inertial>"
    "<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
    "<mass value=\"0.0001\"/>"
    "<inertia ixx=\"1E-06\" ixy=\"1E-06\" ixz=\"0\" iyy=\"1E-06\" iyz=\"0\" izz=\"0\"/>"
    "</inertial>"
    "</link>"
    "<joint name=\"j1\" type=\"revolute\">"
    "<origin xyz=\"0 0 0.07\" rpy=\"1.5708 0 -1.5708\"/>"
    "<parent link=\"base_link\"/>"
    "<child link=\"Link1\"/>"
    "<axis xyz=\"0 0 -1\"/>"
    "<limit lower=\"-1.57\" upper=\"1.57\"/>"
    "</joint>"
    "<joint name=\"j2\" type=\"revolute\">"
    "<origin xyz=\"-0.0005 0.35 0\" rpy=\"0 0 0\"/>"
    "<parent link=\"Link1\"/>"
    "<child link=\"Link2\"/>"
    "<axis xyz=\"0 0 -1\"/>"
    "<limit lower=\"-2.32\" upper=\"2.32\"/>"
    "</joint>"
    "<joint name=\"j3\" type=\"revolute\">"
    "<origin xyz=\"-0.000357142857063231 0.24999974489785 -0.00262500000000032\" rpy=\"0 0 0\"/>"
    "<parent link=\"Link2\"/>"
    "<child link=\"Link3\"/>"
    "<axis xyz=\"0 0 -1\"/>"
    "<limit lower=\"-1.95\" upper=\"1.95\"/>"
    "</joint>"
    "<joint name=\"j_tool\" type=\"fixed\">"
    "<origin xyz=\"0.0034791 0.14696 -2.5E-05\" rpy=\"0 0 0\"/>"
    "<parent link=\"Link3\"/>"
    "<child link=\"tool_Link\"/>"
    "</joint>"
    "</robot>";

}  // namespace

DemoRealUrdfReport run_demo_real_urdf() {
  DemoRealUrdfReport report;
  report.full_parse_ok = parser::load_chain_from_urdf<4>(
      kRealUrdf, "base_link", "tool_Link", &report.full_chain);
  report.movable_parse_ok = parser::load_movable_chain_from_urdf<3>(
      kRealUrdf, "base_link", "tool_Link", &report.movable_chain);
  if (!report.full_parse_ok || !report.movable_parse_ok) {
    return report;
  }

  report.full_q[0][0] = 0.20f;
  report.full_q[1][0] = -0.35f;
  report.full_q[2][0] = 0.30f;
  report.full_q[3][0] = 0.0f;
  report.movable_q[0][0] = report.full_q[0][0];
  report.movable_q[1][0] = report.full_q[1][0];
  report.movable_q[2][0] = report.full_q[2][0];

  report.full_pose = kinematics::fk(report.full_chain, report.full_q);
  report.movable_pose = kinematics::fk(report.movable_chain, report.movable_q);
  const kinematics::PoseError6 fk_error =
      kinematics::evaluate_pose_error(report.full_pose, report.movable_pose);
  report.fk_position_error = fk_error.position_norm;
  report.fk_orientation_error = fk_error.orientation_norm;

  report.full_gravity = dynamics::gravity_torques(report.full_chain, report.full_q);
  report.movable_gravity =
      dynamics::gravity_torques(report.movable_chain, report.movable_q);
  JointVec<3> gravity_delta = toolbox_adapter::zero_joint_vec<3>();
  for (uint16_t i = 0; i < 3; ++i) {
    gravity_delta[i][0] = report.full_gravity[i][0] - report.movable_gravity[i][0];
  }
  report.gravity_error_norm = gravity_delta.norm();

  report.link1_mass_error = abs_scalar(report.movable_chain.link(0).mass() - 0.54717f);
  report.link2_mass_error = abs_scalar(report.movable_chain.link(1).mass() - 0.5273f);
  report.link3_mass_error = abs_scalar(report.movable_chain.link(2).mass() - 0.24563f);

  kinematics::IKRequest<3> hybrid_request;
  hybrid_request.target = report.movable_pose;
  hybrid_request.reference = report.movable_q;
  hybrid_request.use_reference = true;
  kinematics::HybridIKOptions hybrid_options;
  hybrid_options.numeric_options.max_iterations = 80U;
  hybrid_options.numeric_options.error_tolerance = 1.0e-5f;
  hybrid_options.numeric_options.step_tolerance = 1.0e-6f;
  hybrid_options.numeric_options.enable_multi_start = true;
  hybrid_options.numeric_options.max_retries = 5U;
  report.hybrid_ik_result = kinematics::solve_ik_hybrid(
      report.movable_chain, hybrid_request,
      kinematics::analytic::make_planar_3r_plugin(), hybrid_options);
  report.hybrid_q_solution = report.hybrid_ik_result.q;
  const Transform hybrid_pose =
      kinematics::fk(report.movable_chain, report.hybrid_q_solution);
  const kinematics::PoseError6 hybrid_error =
      kinematics::evaluate_pose_error(report.movable_pose, hybrid_pose);
  report.hybrid_position_error = hybrid_error.position_norm;
  report.hybrid_orientation_error = hybrid_error.orientation_norm;
  report.hybrid_ik_valid = is_success(report.hybrid_ik_result.status);
  return report;
}

bool run_demo_real_urdf_success() {
  const DemoRealUrdfReport report = run_demo_real_urdf();
  return report.full_parse_ok && report.movable_parse_ok &&
         report.fk_position_error <= 1.0e-5f &&
         report.fk_orientation_error <= 1.0e-5f &&
         report.gravity_error_norm <= 5.0e-3f &&
         report.hybrid_ik_valid &&
         report.hybrid_position_error <= 1.0e-4f &&
         report.hybrid_orientation_error <= 1.0e-4f &&
         report.hybrid_ik_result.used_numeric_fallback &&
         report.link1_mass_error <= 1.0e-5f &&
         report.link2_mass_error <= 1.0e-5f &&
         report.link3_mass_error <= 1.0e-5f;
}

}  // namespace demo
}  // namespace arm_lib
