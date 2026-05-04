#ifndef ARM_LIB_PARSER_URDF_LOADER_H
#define ARM_LIB_PARSER_URDF_LOADER_H

#include "chain_builder.h"

namespace mr::robotics::arm {
namespace parser {

struct UrdfJointDesc {
  char name[ARM_LIB_URDF_NAME_LENGTH];
  char parent_link[ARM_LIB_URDF_NAME_LENGTH];
  char child_link[ARM_LIB_URDF_NAME_LENGTH];
  ChainJointSpec joint;
  Transform origin;
  Vec3 axis;

  UrdfJointDesc()
      : name(),
        parent_link(),
        child_link(),
        joint(),
        origin(toolbox_adapter::identity_transform()),
        axis(toolbox_adapter::zero_vec3()) {
    axis[2][0] = 1.0f;
  }
};

struct UrdfLinkDesc {
  char name[ARM_LIB_URDF_NAME_LENGTH];
  InertialParams inertial;
  bool has_inertial;

  UrdfLinkDesc() : name(), inertial(), has_inertial(false) {}
};

struct UrdfParserWorkspace {
  UrdfJointDesc joints[ARM_LIB_URDF_MAX_JOINTS];
  UrdfLinkDesc links[ARM_LIB_URDF_MAX_JOINTS + 1U];
  uint16_t path_indices[ARM_LIB_URDF_MAX_JOINTS];
  bool used_joints[ARM_LIB_URDF_MAX_JOINTS];
  char joint_block[1024];
  char link_block[1536];
  char inertial_block[768];

  UrdfParserWorkspace()
      : joints(),
        links(),
        path_indices(),
        used_joints(),
        joint_block(),
        link_block(),
        inertial_block() {}
};

bool parse_urdf_joints(const char* urdf_xml,
                       UrdfJointDesc* joints,
                       uint16_t max_joint_count,
                       uint16_t* joint_count_out);

bool parse_urdf_joints(const char* urdf_xml,
                       UrdfJointDesc* joints,
                       uint16_t max_joint_count,
                       uint16_t* joint_count_out,
                       UrdfParserWorkspace* workspace);

bool parse_urdf_links(const char* urdf_xml,
                      UrdfLinkDesc* links,
                      uint16_t max_link_count,
                      uint16_t* link_count_out);

bool parse_urdf_links(const char* urdf_xml,
                      UrdfLinkDesc* links,
                      uint16_t max_link_count,
                      uint16_t* link_count_out,
                      UrdfParserWorkspace* workspace);

const UrdfLinkDesc* find_urdf_link_by_name(const UrdfLinkDesc* links,
                                           uint16_t link_count,
                                           const char* link_name);

bool find_urdf_joint_path(const UrdfJointDesc* joints,
                          uint16_t joint_count,
                          const char* root_link,
                          const char* tip_link,
                          uint16_t* path_indices,
                          uint16_t max_path_length,
                          uint16_t* path_length_out);

bool find_urdf_joint_path(const UrdfJointDesc* joints,
                          uint16_t joint_count,
                          const char* root_link,
                          const char* tip_link,
                          uint16_t* path_indices,
                          uint16_t max_path_length,
                          uint16_t* path_length_out,
                          UrdfParserWorkspace* workspace);

uint16_t count_movable_urdf_joints_on_path(const UrdfJointDesc* joints,
                                           const uint16_t* path_indices,
                                           uint16_t path_length);

Link make_link_from_urdf_joint(const UrdfJointDesc& joint,
                               const UrdfLinkDesc* child_link_desc = nullptr);

template <int N>
inline bool load_chain_from_urdf(const char* urdf_xml,
                                 const char* root_link,
                                 const char* tip_link,
                                 SerialChain<N>* chain_out,
                                 UrdfParserWorkspace* workspace) {
  if (urdf_xml == nullptr || root_link == nullptr || tip_link == nullptr ||
      chain_out == nullptr || workspace == nullptr) {
    return false;
  }

  uint16_t joint_count = 0U;
  uint16_t link_count = 0U;
  if (!parse_urdf_joints(urdf_xml, workspace->joints,
                         ARM_LIB_URDF_MAX_JOINTS, &joint_count, workspace)) {
    return false;
  }
  (void)parse_urdf_links(urdf_xml, workspace->links,
                         ARM_LIB_URDF_MAX_JOINTS + 1U, &link_count, workspace);

  uint16_t path_length = 0U;
  if (!find_urdf_joint_path(workspace->joints, joint_count, root_link, tip_link,
                            workspace->path_indices,
                            ARM_LIB_URDF_MAX_JOINTS, &path_length,
                            workspace)) {
    return false;
  }
  if (path_length != static_cast<uint16_t>(N)) {
    return false;
  }

  Link links[N];
  for (uint16_t i = 0; i < N; ++i) {
    const UrdfLinkDesc* child_link_desc =
        find_urdf_link_by_name(workspace->links, link_count,
                               workspace->joints[workspace->path_indices[i]]
                                   .child_link);
    links[i] = make_link_from_urdf_joint(
        workspace->joints[workspace->path_indices[i]], child_link_desc);
  }
  *chain_out = parser::make_serial_chain(links);
  return true;
}

template <int N>
inline bool load_chain_from_urdf(const char* urdf_xml,
                                 const char* root_link,
                                 const char* tip_link,
                                 SerialChain<N>* chain_out) {
  UrdfParserWorkspace workspace;
  return load_chain_from_urdf(urdf_xml, root_link, tip_link, chain_out,
                              &workspace);
}

template <int N>
inline bool load_movable_chain_from_urdf(const char* urdf_xml,
                                         const char* root_link,
                                         const char* tip_link,
                                         SerialChain<N>* chain_out,
                                         UrdfParserWorkspace* workspace) {
  if (urdf_xml == nullptr || root_link == nullptr || tip_link == nullptr ||
      chain_out == nullptr || workspace == nullptr) {
    return false;
  }

  uint16_t joint_count = 0U;
  uint16_t link_count = 0U;
  if (!parse_urdf_joints(urdf_xml, workspace->joints,
                         ARM_LIB_URDF_MAX_JOINTS, &joint_count, workspace)) {
    return false;
  }
  (void)parse_urdf_links(urdf_xml, workspace->links,
                         ARM_LIB_URDF_MAX_JOINTS + 1U, &link_count, workspace);

  uint16_t path_length = 0U;
  if (!find_urdf_joint_path(workspace->joints, joint_count, root_link, tip_link,
                            workspace->path_indices,
                            ARM_LIB_URDF_MAX_JOINTS, &path_length, workspace)) {
    return false;
  }
  if (count_movable_urdf_joints_on_path(
          workspace->joints, workspace->path_indices, path_length) !=
      static_cast<uint16_t>(N)) {
    return false;
  }

  Link links[N];
  Transform base_frame = toolbox_adapter::identity_transform();
  Transform accumulated_fixed = toolbox_adapter::identity_transform();
  uint16_t movable_index = 0U;

  for (uint16_t i = 0; i < path_length; ++i) {
    const UrdfJointDesc& joint = workspace->joints[workspace->path_indices[i]];
    if (joint.joint.type == ChainJointType::kFixed) {
      accumulated_fixed = accumulated_fixed * joint.origin;
      continue;
    }

    if (movable_index == 0U) {
      base_frame = accumulated_fixed;
      links[movable_index] = make_link_from_urdf_joint(
          joint, find_urdf_link_by_name(workspace->links, link_count,
                                        joint.child_link));
    } else {
      links[movable_index] = make_link_from_urdf_joint(
          joint, find_urdf_link_by_name(workspace->links, link_count,
                                        joint.child_link));
      links[movable_index].set_joint_origin_transform(accumulated_fixed *
                                                      joint.origin);
    }

    accumulated_fixed = toolbox_adapter::identity_transform();
    ++movable_index;
  }

  *chain_out =
      parser::make_serial_chain(links, base_frame, ToolFrame(accumulated_fixed));
  return true;
}

template <int N>
inline bool load_movable_chain_from_urdf(const char* urdf_xml,
                                         const char* root_link,
                                         const char* tip_link,
                                         SerialChain<N>* chain_out) {
  UrdfParserWorkspace workspace;
  return load_movable_chain_from_urdf(urdf_xml, root_link, tip_link, chain_out,
                                      &workspace);
}

}  // namespace parser
}  // namespace mr::robotics::arm

#endif
