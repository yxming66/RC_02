#ifndef ARM_LIB_KINEMATICS_TASK_CONSTRAINTS_H
#define ARM_LIB_KINEMATICS_TASK_CONSTRAINTS_H

#include "ik.h"

namespace mr::robotics::arm {
namespace kinematics {

enum class OrientationConstraintMode : uint8_t {
  kFull = 0,
  kToolZAxis = 1,
  kYawPitchOnly = 2,
  kPitchOnly = 3,
  kPlanarPitchOnly = 4,
};

struct TaskConstraintProfile {
  Vec3 position_weights;
  Scalar orientation_weight;
  OrientationConstraintMode orientation_mode;

  TaskConstraintProfile()
      : position_weights(toolbox_adapter::zero_vec3()),
        orientation_weight(1.0f),
        orientation_mode(OrientationConstraintMode::kFull) {
    for (uint16_t i = 0; i < 3U; ++i) {
      position_weights[i][0] = 1.0f;
    }
  }
};

TaskMap6 identity_task_map();
void apply_task_constraint_profile_to_ik_options(
    const Transform& target_pose, const TaskConstraintProfile& profile,
    IKOptions* options);

inline TaskConstraintProfile make_task_constraint_profile(
    Scalar weight_x, Scalar weight_y, Scalar weight_z,
    OrientationConstraintMode orientation_mode =
        OrientationConstraintMode::kFull,
    Scalar orientation_weight = 1.0f) {
  TaskConstraintProfile profile;
  profile.position_weights[0][0] = weight_x;
  profile.position_weights[1][0] = weight_y;
  profile.position_weights[2][0] = weight_z;
  profile.orientation_mode = orientation_mode;
  profile.orientation_weight = orientation_weight;
  return profile;
}

inline TaskConstraintProfile make_xyz_full_profile() {
  return make_task_constraint_profile(1.0f, 1.0f, 1.0f,
                                      OrientationConstraintMode::kFull, 1.0f);
}

inline TaskConstraintProfile make_xyz_tool_z_profile(
    Scalar orientation_weight = 1.0f) {
  return make_task_constraint_profile(
      1.0f, 1.0f, 1.0f, OrientationConstraintMode::kToolZAxis,
      orientation_weight);
}

inline TaskConstraintProfile make_xyz_yaw_pitch_profile(
    Scalar orientation_weight = 1.0f) {
  return make_task_constraint_profile(
      1.0f, 1.0f, 1.0f, OrientationConstraintMode::kYawPitchOnly,
      orientation_weight);
}

inline TaskConstraintProfile make_yz_full_profile(
    Scalar orientation_weight = 1.0f) {
  return make_task_constraint_profile(0.0f, 1.0f, 1.0f,
                                      OrientationConstraintMode::kFull,
                                      orientation_weight);
}

inline TaskConstraintProfile make_yz_pitch_profile(
    Scalar orientation_weight = 1.0f) {
  return make_task_constraint_profile(
      0.0f, 1.0f, 1.0f, OrientationConstraintMode::kPlanarPitchOnly,
      orientation_weight);
}

inline TaskConstraintProfile make_position_only_profile() {
  return make_task_constraint_profile(1.0f, 1.0f, 1.0f,
                                      OrientationConstraintMode::kFull, 0.0f);
}

inline void reset_task_weights(IKOptions* options) {
  if (options == nullptr) {
    return;
  }

  options->task_projection = identity_task_map();
  options->enable_task_projection = false;
  options->enable_task_weighting = false;
  for (uint16_t i = 0; i < 6U; ++i) {
    options->task_weights[i][0] = 1.0f;
  }
}

inline IKOptions make_constrained_ik_options(
    const Transform& target_pose, const TaskConstraintProfile& profile,
    IkProfile ik_profile = IkProfile::kDefault) {
  IKOptions options = make_ik_options(ik_profile);
  apply_task_constraint_profile_to_ik_options(target_pose, profile, &options);
  return options;
}

inline IKOptions make_position_only_ik_options(
    const Transform& target_pose,
    IkProfile ik_profile = IkProfile::kPositionOnly) {
  return make_constrained_ik_options(target_pose, make_position_only_profile(),
                                     ik_profile);
}

inline IKOptions make_yz_pitch_ik_options(
    const Transform& target_pose,
    IkProfile ik_profile = IkProfile::kPositionOnly,
    Scalar orientation_weight = 1.0f) {
  return make_constrained_ik_options(
      target_pose, make_yz_pitch_profile(orientation_weight), ik_profile);
}

PoseError6 evaluate_constrained_pose_error(
    const Transform& target_pose, const Transform& current_pose,
    PoseErrorFrame error_frame, const TaskConstraintProfile& profile);

void apply_task_constraint_profile_to_ik_options(
    const Transform& target_pose, const TaskConstraintProfile& profile,
    IKOptions* options);

inline Vec3 unit_x_axis() {
  Vec3 axis = toolbox_adapter::zero_vec3();
  axis[0][0] = 1.0f;
  return axis;
}

inline Vec3 unit_z_axis() {
  Vec3 axis = toolbox_adapter::zero_vec3();
  axis[2][0] = 1.0f;
  return axis;
}

inline Rotation axis_rejection_matrix(const Vec3& axis) {
  Rotation projection = matrixf::eye<3, 3>();
  Vec3 normalized_axis = axis;
  const Scalar norm = axis.norm();
  if (norm <= ARM_LIB_EPSILON) {
    return projection;
  }

  normalized_axis /= norm;
  for (uint16_t row = 0; row < 3U; ++row) {
    for (uint16_t col = 0; col < 3U; ++col) {
      projection[row][col] -=
          normalized_axis[row][0] * normalized_axis[col][0];
    }
  }
  return projection;
}

inline TaskMap6 identity_task_map() { return matrixf::eye<6, 6>(); }

inline TaskMap6 make_orientation_task_projection(
    const Transform& target_pose, OrientationConstraintMode mode) {
  TaskMap6 projection = identity_task_map();
  if (mode == OrientationConstraintMode::kFull) {
    return projection;
  }

  const Rotation target_rotation = toolbox_adapter::rotation_of(target_pose);
  Vec3 unconstrained_axis = toolbox_adapter::zero_vec3();
  if (mode == OrientationConstraintMode::kToolZAxis) {
    unconstrained_axis = target_rotation * unit_z_axis();
  } else if (mode == OrientationConstraintMode::kYawPitchOnly) {
    unconstrained_axis = target_rotation * unit_x_axis();
  } else if (mode == OrientationConstraintMode::kPitchOnly) {
    projection[3][3] = 0.0f;
    projection[4][4] = 1.0f;
    projection[5][5] = 0.0f;
    return projection;
  } else if (mode == OrientationConstraintMode::kPlanarPitchOnly) {
    projection[3][3] = 1.0f;
    projection[4][4] = 0.0f;
    projection[5][5] = 0.0f;
    return projection;
  }

  const Rotation angular_projection = axis_rejection_matrix(unconstrained_axis);
  for (uint16_t row = 0; row < 3U; ++row) {
    for (uint16_t col = 0; col < 3U; ++col) {
      projection[row + 3U][col + 3U] = angular_projection[row][col];
    }
  }
  return projection;
}

inline Vec3 constrained_orientation_error(
    const Transform& target_pose, const Transform& current_pose,
    PoseErrorFrame error_frame,
    OrientationConstraintMode mode = OrientationConstraintMode::kFull) {
  if (mode == OrientationConstraintMode::kFull) {
    return orientation_error(target_pose, current_pose, error_frame);
  }

  const Rotation target_rotation = toolbox_adapter::rotation_of(target_pose);
  const Rotation current_rotation = toolbox_adapter::rotation_of(current_pose);

  if (mode == OrientationConstraintMode::kToolZAxis) {
    const Vec3 local_z = unit_z_axis();
    if (error_frame == PoseErrorFrame::kBody) {
      const Vec3 target_z_in_current =
          current_rotation.trans() * (target_rotation * local_z);
      return vector3f::cross(local_z, target_z_in_current);
    }

    const Vec3 current_z = current_rotation * local_z;
    const Vec3 target_z = target_rotation * local_z;
    return vector3f::cross(current_z, target_z);
  }

  if (mode == OrientationConstraintMode::kPlanarPitchOnly) {
    Vec3 planar_pitch_error =
        orientation_error(target_pose, current_pose, error_frame);
    planar_pitch_error[1][0] = 0.0f;
    planar_pitch_error[2][0] = 0.0f;
    return planar_pitch_error;
  }

  Rotation delta_rotation = toolbox_adapter::identity_rotation();
  if (error_frame == PoseErrorFrame::kBody) {
    delta_rotation = current_rotation.trans() * target_rotation;
  } else {
    delta_rotation = target_rotation * current_rotation.trans();
  }
  Vec3 rpy_error = toolbox_adapter::rpy_from_rotation(delta_rotation);
  if (mode == OrientationConstraintMode::kYawPitchOnly) {
    rpy_error[2][0] = 0.0f;
  } else {
    rpy_error[0][0] = 0.0f;
    rpy_error[2][0] = 0.0f;
  }
  return rpy_error;
}

inline PoseError6 evaluate_constrained_pose_error(
    const Transform& target_pose, const Transform& current_pose,
    PoseErrorFrame error_frame,
    OrientationConstraintMode mode = OrientationConstraintMode::kFull) {
  TaskConstraintProfile profile;
  profile.orientation_mode = mode;
  return evaluate_constrained_pose_error(target_pose, current_pose, error_frame,
                                         profile);
}

inline PoseError6 evaluate_constrained_pose_error(
    const Transform& target_pose, const Transform& current_pose,
    PoseErrorFrame error_frame, const TaskConstraintProfile& profile) {
  PoseError6 error = evaluate_pose_error(target_pose, current_pose, error_frame);
  for (uint16_t i = 0; i < 3U; ++i) {
    error.twist[i][0] *= profile.position_weights[i][0];
  }
  error.position_norm = error.twist.block<3, 1>(0, 0).norm();

  const Vec3 orientation = constrained_orientation_error(
      target_pose, current_pose, error_frame, profile.orientation_mode);
  for (uint16_t i = 0; i < 3U; ++i) {
    error.twist[i + 3U][0] =
        orientation[i][0] * profile.orientation_weight;
  }
  error.orientation_norm =
      error.twist.block<3, 1>(3, 0).norm();
  return error;
}

inline void apply_orientation_constraint_to_ik_options(
    const Transform& target_pose, OrientationConstraintMode mode,
    Scalar orientation_weight, IKOptions* options) {
  TaskConstraintProfile profile;
  profile.orientation_mode = mode;
  profile.orientation_weight = orientation_weight;
  apply_task_constraint_profile_to_ik_options(target_pose, profile, options);
}

inline void apply_task_constraint_profile_to_ik_options(
    const Transform& target_pose, const TaskConstraintProfile& profile,
    IKOptions* options) {
  if (options == nullptr) {
    return;
  }

  reset_task_weights(options);
  for (uint16_t i = 0; i < 3U; ++i) {
    options->task_weights[i][0] = profile.position_weights[i][0];
  }
  for (uint16_t i = 3; i < 6U; ++i) {
    options->task_weights[i][0] = profile.orientation_weight;
  }

  if (profile.orientation_mode != OrientationConstraintMode::kFull) {
    options->task_projection =
        make_orientation_task_projection(target_pose, profile.orientation_mode);
    options->enable_task_projection = true;
  }
  bool nontrivial_position = false;
  for (uint16_t i = 0; i < 3U; ++i) {
    if (abs_scalar(profile.position_weights[i][0] - 1.0f) > ARM_LIB_EPSILON) {
      nontrivial_position = true;
      break;
    }
  }
  if (profile.orientation_mode != OrientationConstraintMode::kFull ||
      abs_scalar(profile.orientation_weight - 1.0f) > ARM_LIB_EPSILON ||
      nontrivial_position) {
    options->enable_task_weighting = true;
  }
}

}  // namespace kinematics
}  // namespace mr::robotics::arm

#endif
