// (c) 2020-2021 Philipp Ruppel

#include <tractor/robot/robotinfo.h>

#include <moveit/robot_model/robot_model.h>

namespace tractor {

RobotJointInfo::RobotJointInfo(const moveit::core::JointModel &moveit_joint)
    : _index(moveit_joint.getJointIndex()), _name(moveit_joint.getName()),
      _first_variable_index(moveit_joint.getFirstVariableIndex()),
      _origin(moveit_joint.getChildLinkModel()
                  ->getJointOriginTransform()
                  .matrix()) {

  if (auto *moveit_parent = moveit_joint.getParentLinkModel()) {
    _parent_link_index = moveit_parent->getLinkIndex();
  }

  _child_link_index = moveit_joint.getChildLinkModel()->getLinkIndex();

  if (auto *mimic = moveit_joint.getMimic()) {
    _is_mimic = true;
    _mimic_index = mimic->getJointIndex();
    _mimic_factor = mimic->getMimicFactor();
    _mimic_offset = mimic->getMimicOffset();
  }

  auto &bounds = moveit_joint.getVariableBounds();
  if (!bounds.empty()) {
    _has_bounds = bounds.front().position_bounded_;
    _lower_bound = bounds.front().min_position_;
    _upper_bound = bounds.front().max_position_;
  }

  switch (moveit_joint.getType()) {
  case moveit::core::JointModel::FIXED:
    _type = JointType::Fixed;
    break;
  case moveit::core::JointModel::REVOLUTE: {
    _axis = dynamic_cast<const moveit::core::RevoluteJointModel &>(moveit_joint)
                .getAxis();
    _type = JointType::Revolute;
    break;
  }
  case moveit::core::JointModel::PRISMATIC: {
    _axis =
        dynamic_cast<const moveit::core::PrismaticJointModel &>(moveit_joint)
            .getAxis();
    _type = JointType::Prismatic;
    break;
  }
  case moveit::core::JointModel::PLANAR:
    _type = JointType::Planar;
    break;
  case moveit::core::JointModel::FLOATING:
    _type = JointType::Floating;
    break;
  default:
    throw std::runtime_error("unsupported joint type");
  }
}

RobotJointMap::RobotJointMap(const moveit::core::RobotModel &robot_model)
    : RobotIndexMap(robot_model.getJointModelNames()) {
  for (auto &joint : robot_model.getJointModels()) {
    _joint_infos.emplace_back(*joint);
  }
  robot_model.getVariableDefaultPositions(_default_positions);
}

RobotInfo::RobotInfo(const moveit::core::RobotModel &robot_model)
    : _joints(robot_model), _links(robot_model.getLinkModelNames()),
      _variable_count(robot_model.getVariableCount()) {}

} // namespace tractor
