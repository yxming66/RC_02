#ifndef ARM_LIB_MODEL_TOOL_FRAME_H
#define ARM_LIB_MODEL_TOOL_FRAME_H

#include "../adapter/toolbox_adapter.h"

namespace arm_lib {

class ToolFrame {
 public:
  ToolFrame();
  explicit ToolFrame(const Transform& transform);
  ToolFrame(const Rotation& rotation, const Vec3& translation);

  static ToolFrame identity();

  const Transform& transform() const { return transform_; }
  Rotation rotation() const;
  Vec3 translation() const;

  void set_transform(const Transform& transform) { transform_ = transform; }
  void set_pose(const Rotation& rotation, const Vec3& translation);

 private:
  Transform transform_;
};

}  // namespace arm_lib

#endif
