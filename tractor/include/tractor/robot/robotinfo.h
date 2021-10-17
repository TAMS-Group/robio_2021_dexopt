// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <unordered_map>
#include <vector>

#include <tractor/core/eigen.h>

namespace moveit {
namespace core {
class RobotModel;
class JointModel;
class LinkModel;
} // namespace core
} // namespace moveit

namespace tractor {

enum class JointType {
  Fixed,
  Revolute,
  Prismatic,
  Planar,
  Floating,
};

class RobotIndexMap {
  std::vector<std::string> _names;
  std::unordered_map<std::string, size_t> _indices;

public:
  RobotIndexMap(const std::vector<std::string> &names) {
    _names = names;
    for (size_t i = 0; i < _names.size(); i++) {
      _indices[names[i]] = i;
    }
  }
  const std::string &name(size_t i) const { return _names.at(i); }
  size_t index(const std::string &name) const {
    auto it = _indices.find(name);
    if (it != _indices.end()) {
      return it->second;
    } else {
      throw std::runtime_error("entry not found: " + name);
    }
  }
  size_t size() const { return _names.size(); }
  size_t count() const { return _names.size(); }
  auto &names() const { return _names; }
  bool contains(const std::string &name) const {
    return _indices.find(name) != _indices.end();
  }
};

class RobotJointInfo {
  size_t _index = 0;
  std::string _name;
  bool _is_mimic = false;
  size_t _mimic_index = 0;
  double _mimic_factor = 0;
  double _mimic_offset = 0;
  size_t _first_variable_index = 0;
  Eigen::Transform<double, 3, Eigen::Isometry, Eigen::DontAlign> _origin;
  bool _has_bounds = false;
  double _lower_bound = 0;
  double _upper_bound = 0;
  JointType _type = JointType::Fixed;
  Eigen::Vector3d _axis = Eigen::Vector3d::Zero();
  ssize_t _parent_link_index = -1;
  size_t _child_link_index = -1;

public:
  RobotJointInfo(const moveit::core::JointModel &moveit_joint);
  bool isMimicJoint() const { return _is_mimic; }
  size_t mimicIndex() const { return _mimic_index; }
  double mimicFactor() const { return _mimic_factor; }
  double mimicOffset() const { return _mimic_offset; }
  size_t firstVariableIndex() const { return _first_variable_index; }
  auto &origin() const { return _origin; }
  bool hasBounds() const { return _has_bounds; }
  double lowerBound() const { return _lower_bound; }
  double upperBound() const { return _upper_bound; }
  JointType type() const { return _type; }
  auto &axis() const { return _axis; }
  auto childLinkIndex() const { return _child_link_index; }
  auto parentLinkIndex() const { return _parent_link_index; }
  auto &name() const { return _name; }
  bool hasParentLink() const { return (_parent_link_index >= 0); }
};

class RobotJointMap : public RobotIndexMap {
  std::vector<RobotJointInfo> _joint_infos;
  std::vector<double> _default_positions;

public:
  RobotJointMap(const moveit::core::RobotModel &robot_model);
  auto &info(size_t i) const { return _joint_infos[i]; }
  auto &info(const std::string &name) const {
    return _joint_infos.at(index(name));
  }
  auto &defaultPositions() const { return _default_positions; }
};

class RobotLinkMap : public RobotIndexMap {
public:
  RobotLinkMap(const std::vector<std::string> &names) : RobotIndexMap(names) {}
};

struct RobotInfo {
  size_t _variable_count = 0;
  RobotJointMap _joints;
  RobotLinkMap _links;

public:
  RobotInfo(const moveit::core::RobotModel &robot_model);
  const RobotJointMap &joints() const { return _joints; }
  const RobotLinkMap &links() const { return _links; }
  double variableCount() const { return _variable_count; }
};

} // namespace tractor
