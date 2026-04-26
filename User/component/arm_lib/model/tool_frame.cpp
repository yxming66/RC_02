#include "tool_frame.h"

namespace arm_lib {

ToolFrame::ToolFrame()
    : transform_(toolbox_adapter::identity_transform()) {}

ToolFrame::ToolFrame(const Transform& transform) : transform_(transform) {}

ToolFrame::ToolFrame(const Rotation& rotation, const Vec3& translation)
    : transform_(toolbox_adapter::make_transform(rotation, translation)) {}

ToolFrame ToolFrame::identity() {
  return ToolFrame(toolbox_adapter::identity_transform());
}

Rotation ToolFrame::rotation() const {
  return toolbox_adapter::rotation_of(transform_);
}

Vec3 ToolFrame::translation() const {
  return toolbox_adapter::translation_of(transform_);
}

void ToolFrame::set_pose(const Rotation& rotation, const Vec3& translation) {
  transform_ = toolbox_adapter::make_transform(rotation, translation);
}

}  // namespace arm_lib
