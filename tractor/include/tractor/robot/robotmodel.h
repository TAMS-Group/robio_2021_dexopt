// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "jointtypes.h"
#include "robotinfo.h"

#include <deque>
#include <memory>

namespace tractor {

template <class Geometry> class JointState;
template <class Geometry> class LinkState;

template <class Geometry> class RobotModel {
  std::shared_ptr<const RobotInfo> _robot_info;
  AlignedStdVector<JointVariant<JointModelBase<Geometry>>> _joint_models;
  AlignedStdVector<JointVariant<JointStateBase<Geometry>>>
      _default_joint_states;
  AlignedStdVector<typename Geometry::Scalar> _default_positions;

  struct JointInfo {
    ssize_t parent_link_index = -1;
    ssize_t child_link_index = -1;
  };
  std::vector<JointInfo> _joint_infos;

  template <class Model, class State>
  void addJoint(const RobotJointInfo &robot_joint_info,
                const Model &joint_model_in, const State &joint_state_in) {

    Model joint_model = joint_model_in;
    State joint_state = joint_state_in;

    joint_model.origin() = Geometry::import(robot_joint_info.origin());
    _joint_models.emplace_back(joint_model);

    joint_state.deserializePositions(_default_positions.data() +
                                     robot_joint_info.firstVariableIndex());
    _default_joint_states.emplace_back(joint_state);

    JointInfo joint_info;
    joint_info.parent_link_index = robot_joint_info.parentLinkIndex();
    joint_info.child_link_index = robot_joint_info.childLinkIndex();
    _joint_infos.emplace_back(joint_info);
  }

public:
  RobotModel() {}

  RobotModel(const moveit::core::RobotModel &moveit_robot)
      : _robot_info(std::make_shared<RobotInfo>(moveit_robot)) {

    _default_positions.clear();
    for (auto &p : _robot_info->joints().defaultPositions()) {
      _default_positions.push_back(typename Geometry::Value(p));
    }

    for (size_t joint_index = 0; joint_index < _robot_info->joints().size();
         joint_index++) {
      auto &joint = _robot_info->joints().info(joint_index);

      switch (joint.type()) {

      case JointType::Fixed: {
        FixedJointModel<Geometry> joint_model;
        FixedJointState<Geometry> joint_state;
        addJoint(joint, joint_model, joint_state);
        break;
      }

      case JointType::Revolute: {
        RevoluteJointModel<Geometry> joint_model;
        joint_model.axis() = Geometry::import(joint.axis());
        if (joint.hasBounds()) {
          joint_model.limits() = JointLimits<Geometry>(
              typename Geometry::Value(joint.lowerBound()),
              typename Geometry::Value(joint.upperBound()));
        }
        RevoluteJointState<Geometry> joint_state;
        addJoint(joint, joint_model, joint_state);
        break;
      }

      case JointType::Prismatic: {
        PrismaticJointModel<Geometry> joint_model;
        joint_model.axis() = Geometry::import(joint.axis());
        if (joint.hasBounds()) {
          joint_model.limits() = JointLimits<Geometry>(
              typename Geometry::Value(joint.lowerBound()),
              typename Geometry::Value(joint.upperBound()));
        }
        PrismaticJointState<Geometry> joint_state;
        addJoint(joint, joint_model, joint_state);
        break;
      }

      case JointType::Planar: {
        PlanarJointModel<Geometry> joint_model;
        PlanarJointState<Geometry> joint_state;
        addJoint(joint, joint_model, joint_state);
        break;
      }

      case JointType::Floating: {
        FloatingJointModel<Geometry> joint_model;
        FloatingJointState<Geometry> joint_state;
        addJoint(joint, joint_model, joint_state);
        break;
      }

      default:
        throw std::runtime_error("joint type not yet implemented");
      }
    }
  }

  auto &joint(size_t i) const { return *_joint_models.at(i); }
  auto &joint(size_t i) { return *_joint_models.at(i); }
  auto &joint(const std::string &name) const {
    return *_joint_models.at(_robot_info->joints().index(name));
  }
  auto &joint(const std::string &name) {
    return *_joint_models.at(_robot_info->joints().index(name));
  }

  void computeFK(const JointState<Geometry> &joint_state,
                 LinkState<Geometry> &link_state) const {

    for (size_t joint_index = 0; joint_index < _joint_infos.size();
         joint_index++) {
      auto &joint_info = _joint_infos[joint_index];

      if (joint_info.parent_link_index >= 0) {
        link_state.pose(joint_info.child_link_index) =
            joint_state.joint(joint_index)
                .compute(*_joint_models.at(joint_index),
                         link_state.pose(joint_info.parent_link_index) *
                             _joint_models.at(joint_index)->origin());
      } else {
        link_state.pose(joint_info.child_link_index) =
            joint_state.joint(joint_index)
                .compute(*_joint_models.at(joint_index),
                         _joint_models.at(joint_index)->origin());
      }
    }
  }

  const AlignedStdVector<JointVariant<JointStateBase<Geometry>>> &
  defaultJointStates() const {
    return _default_joint_states;
  }

  const std::shared_ptr<const RobotInfo> &info() const { return _robot_info; }
};

} // namespace tractor
