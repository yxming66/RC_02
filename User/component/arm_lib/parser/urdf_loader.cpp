#include "urdf_loader.h"

#include <cstdlib>
#include <cstdio>
#include <cstring>

namespace arm_lib {
namespace parser {
namespace {

bool copy_attribute_value(const char* text,
                          const char* attribute_name,
                          char* out,
                          uint16_t out_size) {
  if (text == nullptr || attribute_name == nullptr || out == nullptr ||
      out_size == 0U) {
    return false;
  }

  char pattern[32];
  snprintf(pattern, sizeof(pattern), "%s=\"", attribute_name);
  const char* begin = strstr(text, pattern);
  if (begin == nullptr) {
    return false;
  }
  begin += strlen(pattern);
  const char* end = strchr(begin, '"');
  if (end == nullptr) {
    return false;
  }

  const size_t length = static_cast<size_t>(end - begin);
  const size_t copy_len =
      (length < static_cast<size_t>(out_size - 1U)) ? length : (out_size - 1U);
  memcpy(out, begin, copy_len);
  out[copy_len] = '\0';
  return true;
}

bool parse_xyz_string(const char* text, Vec3* out) {
  if (text == nullptr || out == nullptr) {
    return false;
  }
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  if (sscanf(text, "%f %f %f", &x, &y, &z) != 3) {
    return false;
  }
  *out = toolbox_adapter::zero_vec3();
  (*out)[0][0] = x;
  (*out)[1][0] = y;
  (*out)[2][0] = z;
  return true;
}

Transform parse_origin_transform(const char* origin_tag) {
  Vec3 xyz = toolbox_adapter::zero_vec3();
  Vec3 urdf_rpy = toolbox_adapter::zero_vec3();
  char value[96];

  if (copy_attribute_value(origin_tag, "xyz", value, sizeof(value))) {
    (void)parse_xyz_string(value, &xyz);
  }
  if (copy_attribute_value(origin_tag, "rpy", value, sizeof(value))) {
    (void)parse_xyz_string(value, &urdf_rpy);
  }

  Vec3 ypr = toolbox_adapter::zero_vec3();
  ypr[0][0] = urdf_rpy[2][0];
  ypr[1][0] = urdf_rpy[1][0];
  ypr[2][0] = urdf_rpy[0][0];
  return toolbox_adapter::transform_from_rpy_translation(ypr, xyz);
}

Rotation parse_inertia_matrix(const char* inertia_tag) {
  Rotation inertia = toolbox_adapter::identity_rotation() * 0.0f;
  char value[64];

  if (copy_attribute_value(inertia_tag, "ixx", value, sizeof(value))) {
    inertia[0][0] = static_cast<Scalar>(atof(value));
  }
  if (copy_attribute_value(inertia_tag, "ixy", value, sizeof(value))) {
    inertia[0][1] = static_cast<Scalar>(atof(value));
    inertia[1][0] = inertia[0][1];
  }
  if (copy_attribute_value(inertia_tag, "ixz", value, sizeof(value))) {
    inertia[0][2] = static_cast<Scalar>(atof(value));
    inertia[2][0] = inertia[0][2];
  }
  if (copy_attribute_value(inertia_tag, "iyy", value, sizeof(value))) {
    inertia[1][1] = static_cast<Scalar>(atof(value));
  }
  if (copy_attribute_value(inertia_tag, "iyz", value, sizeof(value))) {
    inertia[1][2] = static_cast<Scalar>(atof(value));
    inertia[2][1] = inertia[1][2];
  }
  if (copy_attribute_value(inertia_tag, "izz", value, sizeof(value))) {
    inertia[2][2] = static_cast<Scalar>(atof(value));
  }

  return inertia;
}

ChainJointType parse_joint_type(const char* type_text) {
  if (type_text == nullptr) {
    return ChainJointType::kFixed;
  }
  if (strcmp(type_text, "revolute") == 0 ||
      strcmp(type_text, "continuous") == 0) {
    return ChainJointType::kRevolute;
  }
  if (strcmp(type_text, "prismatic") == 0) {
    return ChainJointType::kPrismatic;
  }
  return ChainJointType::kFixed;
}

bool string_equals(const char* lhs, const char* rhs) {
  if (lhs == nullptr || rhs == nullptr) {
    return false;
  }
  return strcmp(lhs, rhs) == 0;
}

bool find_path_dfs(const UrdfJointDesc* joints,
                   uint16_t joint_count,
                   const char* current_link,
                   const char* tip_link,
                   uint16_t* path_indices,
                   uint16_t max_path_length,
                   uint16_t depth,
                   bool* used_joint,
                   uint16_t* out_depth) {
  if (string_equals(current_link, tip_link)) {
    *out_depth = depth;
    return true;
  }
  if (depth >= max_path_length) {
    return false;
  }

  for (uint16_t i = 0; i < joint_count; ++i) {
    if (used_joint[i] || !string_equals(joints[i].parent_link, current_link)) {
      continue;
    }

    used_joint[i] = true;
    path_indices[depth] = i;
    if (find_path_dfs(joints, joint_count, joints[i].child_link, tip_link,
                      path_indices, max_path_length, depth + 1U, used_joint,
                      out_depth)) {
      return true;
    }
    used_joint[i] = false;
  }

  return false;
}

}  // namespace

bool parse_urdf_joints(const char* urdf_xml,
                       UrdfJointDesc* joints,
                       uint16_t max_joint_count,
                       uint16_t* joint_count_out) {
  if (urdf_xml == nullptr || joints == nullptr || joint_count_out == nullptr ||
      max_joint_count == 0U) {
    return false;
  }

  uint16_t count = 0U;
  const char* cursor = urdf_xml;
  while (count < max_joint_count) {
    const char* joint_begin = strstr(cursor, "<joint");
    if (joint_begin == nullptr) {
      break;
    }
    const char* joint_end = strstr(joint_begin, "</joint>");
    if (joint_end == nullptr) {
      return false;
    }
    joint_end += strlen("</joint>");

    const size_t block_len = static_cast<size_t>(joint_end - joint_begin);
    char block[1024];
    if (block_len >= sizeof(block)) {
      return false;
    }
    memcpy(block, joint_begin, block_len);
    block[block_len] = '\0';

    UrdfJointDesc joint;
    char value[96];
    if (copy_attribute_value(block, "name", joint.name, sizeof(joint.name))) {
    }
    if (copy_attribute_value(block, "type", value, sizeof(value))) {
      joint.joint.type = parse_joint_type(value);
    }
    const char* parent_tag = strstr(block, "<parent");
    const char* child_tag = strstr(block, "<child");
    if (parent_tag == nullptr || child_tag == nullptr) {
      return false;
    }
    if (!copy_attribute_value(parent_tag, "link", joint.parent_link,
                              sizeof(joint.parent_link)) ||
        !copy_attribute_value(child_tag, "link", joint.child_link,
                              sizeof(joint.child_link))) {
      return false;
    }

    const char* origin_tag = strstr(block, "<origin");
    if (origin_tag != nullptr) {
      joint.origin = parse_origin_transform(origin_tag);
    }

    const char* axis_tag = strstr(block, "<axis");
    if (axis_tag != nullptr &&
        copy_attribute_value(axis_tag, "xyz", value, sizeof(value))) {
      (void)parse_xyz_string(value, &joint.axis);
    }

    const char* limit_tag = strstr(block, "<limit");
    if (limit_tag != nullptr) {
      char lower[48];
      char upper[48];
      if (copy_attribute_value(limit_tag, "lower", lower, sizeof(lower)) &&
          copy_attribute_value(limit_tag, "upper", upper, sizeof(upper))) {
        joint.joint.limit_enabled =
            (joint.joint.type != ChainJointType::kFixed);
        joint.joint.lower = static_cast<Scalar>(atof(lower));
        joint.joint.upper = static_cast<Scalar>(atof(upper));
      }
    }
    joint.joint.participate_in_ik =
        (joint.joint.type != ChainJointType::kFixed);

    joints[count] = joint;
    ++count;
    cursor = joint_end;
  }

  *joint_count_out = count;
  return count > 0U;
}

bool parse_urdf_links(const char* urdf_xml,
                      UrdfLinkDesc* links,
                      uint16_t max_link_count,
                      uint16_t* link_count_out) {
  if (urdf_xml == nullptr || links == nullptr || link_count_out == nullptr ||
      max_link_count == 0U) {
    return false;
  }

  uint16_t count = 0U;
  const char* cursor = urdf_xml;
  while (count < max_link_count) {
    const char* link_begin = strstr(cursor, "<link");
    if (link_begin == nullptr) {
      break;
    }

    const char* link_close = strchr(link_begin, '>');
    if (link_close == nullptr) {
      return false;
    }

    const bool self_closing =
        (link_close > link_begin) && (*(link_close - 1) == '/');
    const char* link_end = link_close + 1;
    if (!self_closing) {
      link_end = strstr(link_begin, "</link>");
      if (link_end == nullptr) {
        return false;
      }
      link_end += strlen("</link>");
    }

    const size_t block_len = static_cast<size_t>(link_end - link_begin);
    char block[1536];
    if (block_len >= sizeof(block)) {
      return false;
    }
    memcpy(block, link_begin, block_len);
    block[block_len] = '\0';

    UrdfLinkDesc link;
    if (!copy_attribute_value(block, "name", link.name, sizeof(link.name))) {
      return false;
    }

    const char* inertial_tag = strstr(block, "<inertial");
    if (inertial_tag != nullptr) {
      const char* inertial_end = strstr(inertial_tag, "</inertial>");
      if (inertial_end != nullptr) {
        const size_t inertial_len =
            static_cast<size_t>(inertial_end + strlen("</inertial>") -
                                inertial_tag);
        char inertial_block[768];
        if (inertial_len >= sizeof(inertial_block)) {
          return false;
        }
        memcpy(inertial_block, inertial_tag, inertial_len);
        inertial_block[inertial_len] = '\0';

        const char* origin_tag = strstr(inertial_block, "<origin");
        if (origin_tag != nullptr) {
          const Transform inertial_origin = parse_origin_transform(origin_tag);
          link.inertial.com = toolbox_adapter::translation_of(inertial_origin);
        }

        const char* mass_tag = strstr(inertial_block, "<mass");
        char value[64];
        if (mass_tag != nullptr &&
            copy_attribute_value(mass_tag, "value", value, sizeof(value))) {
          link.inertial.mass = static_cast<Scalar>(atof(value));
        }

        const char* inertia_tag = strstr(inertial_block, "<inertia");
        if (inertia_tag != nullptr) {
          link.inertial.inertia = parse_inertia_matrix(inertia_tag);
        }

        link.has_inertial = true;
      }
    }

    links[count] = link;
    ++count;
    cursor = link_end;
  }

  *link_count_out = count;
  return count > 0U;
}

const UrdfLinkDesc* find_urdf_link_by_name(const UrdfLinkDesc* links,
                                           uint16_t link_count,
                                           const char* link_name) {
  if (links == nullptr || link_name == nullptr) {
    return nullptr;
  }

  for (uint16_t i = 0; i < link_count; ++i) {
    if (string_equals(links[i].name, link_name)) {
      return &links[i];
    }
  }
  return nullptr;
}

bool find_urdf_joint_path(const UrdfJointDesc* joints,
                          uint16_t joint_count,
                          const char* root_link,
                          const char* tip_link,
                          uint16_t* path_indices,
                          uint16_t max_path_length,
                          uint16_t* path_length_out) {
  if (joints == nullptr || root_link == nullptr || tip_link == nullptr ||
      path_indices == nullptr || path_length_out == nullptr) {
    return false;
  }

  bool used_joint[ARM_LIB_URDF_MAX_JOINTS] = {false};
  uint16_t depth = 0U;
  if (!find_path_dfs(joints, joint_count, root_link, tip_link, path_indices,
                     max_path_length, 0U, used_joint, &depth)) {
    return false;
  }

  *path_length_out = depth;
  return true;
}

uint16_t count_movable_urdf_joints_on_path(const UrdfJointDesc* joints,
                                           const uint16_t* path_indices,
                                           uint16_t path_length) {
  if (joints == nullptr || path_indices == nullptr) {
    return 0U;
  }

  uint16_t count = 0U;
  for (uint16_t i = 0; i < path_length; ++i) {
    if (joints[path_indices[i]].joint.type != ChainJointType::kFixed) {
      ++count;
    }
  }
  return count;
}

Link make_link_from_urdf_joint(const UrdfJointDesc& joint,
                               const UrdfLinkDesc* child_link_desc) {
  Link link(joint.origin, joint.axis, joint.joint);
  if (child_link_desc != nullptr && child_link_desc->has_inertial) {
    link.set_inertial(child_link_desc->inertial);
  }
  return link;
}

}  // namespace parser
}  // namespace arm_lib
