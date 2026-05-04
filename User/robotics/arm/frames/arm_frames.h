#ifndef ARM_LIB_FRAMES_ARM_FRAMES_H
#define ARM_LIB_FRAMES_ARM_FRAMES_H

#include <cmath>

#include "../adapter/toolbox_adapter.h"

namespace mr::robotics::arm {
namespace frames {

enum class CartesianCommandFrame : uint8_t {
  kBase = 0,
  kUser,
  kFlange,
  kTool,
  kTcp,
  kHeadingYaw,
};

class ArmFrames {
 public:
  ArmFrames()
      : t_world_base_(toolbox_adapter::identity_transform()),
        t_base_user_(toolbox_adapter::identity_transform()),
        t_flange_tool_(toolbox_adapter::identity_transform()) {}

  const Transform& world_base() const { return t_world_base_; }
  const Transform& base_user() const { return t_base_user_; }
  const Transform& flange_tool() const { return t_flange_tool_; }

  void set_world_base(const Transform& transform) {
    t_world_base_ = transform;
  }

  void set_base_user(const Transform& transform) {
    t_base_user_ = transform;
  }

  void set_flange_tool(const Transform& transform) {
    t_flange_tool_ = transform;
  }

  Transform base_flange_from_base_tcp(const Transform& t_base_tcp) const {
    return t_base_tcp * toolbox_adapter::inverse_transform(t_flange_tool_);
  }

  Transform base_tcp_from_base_flange(const Transform& t_base_flange) const {
    return t_base_flange * t_flange_tool_;
  }

  Transform resolve_absolute_tcp_pose(
      const Transform& target,
      CartesianCommandFrame frame) const {
    if (frame == CartesianCommandFrame::kUser) {
      return t_base_user_ * target;
    }
    if (frame == CartesianCommandFrame::kBase) {
      return target;
    }

    return target;
  }

  Transform resolve_delta_tcp_pose(
      const Transform& current_base_tcp,
      const Transform& delta,
      CartesianCommandFrame frame) const {
    switch (frame) {
      case CartesianCommandFrame::kBase:
        return delta * current_base_tcp;
      case CartesianCommandFrame::kUser:
        return (t_base_user_ * delta *
                toolbox_adapter::inverse_transform(t_base_user_)) *
               current_base_tcp;
      case CartesianCommandFrame::kFlange: {
        const Transform current_base_flange =
            base_flange_from_base_tcp(current_base_tcp);
        return base_tcp_from_base_flange(current_base_flange * delta);
      }
      case CartesianCommandFrame::kTool:
      case CartesianCommandFrame::kTcp:
        return current_base_tcp * delta;
      case CartesianCommandFrame::kHeadingYaw:
        return apply_heading_yaw_delta(current_base_tcp, delta);
      default:
        return current_base_tcp;
    }
  }

 private:
  static Transform make_yaw_transform(Scalar yaw) {
    Transform transform = toolbox_adapter::identity_transform();
    const Scalar c = cosf(yaw);
    const Scalar s = sinf(yaw);
    transform[0][0] = c;
    transform[0][1] = -s;
    transform[1][0] = s;
    transform[1][1] = c;
    return transform;
  }

  static Transform apply_heading_yaw_delta(const Transform& current_base_tcp,
                                           const Transform& delta) {
    Transform target = current_base_tcp;
    const Vec3 current_rpy =
        toolbox_adapter::rpy_from_rotation(
            toolbox_adapter::rotation_of(current_base_tcp));
    const Transform heading = make_yaw_transform(current_rpy[0][0]);
    const Vec3 delta_translation = toolbox_adapter::translation_of(delta);
    const Vec3 base_delta =
        toolbox_adapter::rotation_of(heading) * delta_translation;

    for (uint16_t i = 0; i < 3U; ++i) {
      target[i][3] = current_base_tcp[i][3] + base_delta[i][0];
    }

    const Rotation target_rotation =
        toolbox_adapter::rotation_of(current_base_tcp) *
        toolbox_adapter::rotation_of(delta);
    const Vec3 target_translation = toolbox_adapter::translation_of(target);
    target = toolbox_adapter::make_transform(target_rotation,
                                             target_translation);
    return target;
  }

  Transform t_world_base_;
  Transform t_base_user_;
  Transform t_flange_tool_;
};

inline Transform resolve_delta_tcp_pose(
    const Transform& current_base_tcp,
    const Transform& delta,
    CartesianCommandFrame frame) {
  ArmFrames frames;
  return frames.resolve_delta_tcp_pose(current_base_tcp, delta, frame);
}

inline Transform resolve_absolute_tcp_pose(
    const Transform& target,
    CartesianCommandFrame frame) {
  ArmFrames frames;
  return frames.resolve_absolute_tcp_pose(target, frame);
}

}  // namespace frames
}  // namespace mr::robotics::arm

#endif
