// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "jointtypes.h"
#include "robotinfo.h"

#include <tractor/core/array.h>
#include <tractor/core/eigen.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace tractor {

template <class Geometry> class RobotModel;

template <class Geometry> class JointState {
  std::shared_ptr<const RobotInfo> _robot_info;
  AlignedStdVector<JointVariant<JointStateBase<Geometry>>> _joint_states;

public:
  void init(const RobotModel<Geometry> &robot_model) {
    _robot_info = robot_model.info();
    _joint_states = robot_model.defaultJointStates();
  }

  JointState() {}
  JointState(const RobotModel<Geometry> &robot_model) { init(robot_model); }

  size_t size() const { return _joint_states.size(); }

  auto begin() const { return _joint_states.begin(); }
  auto end() const { return _joint_states.end(); }

  auto begin() { return _joint_states.begin(); }
  auto end() { return _joint_states.end(); }

  const JointStateBase<Geometry> &joint(size_t joint_index) const {
    return *_joint_states.at(joint_index);
  }
  JointStateBase<Geometry> &joint(size_t joint_index) {
    return *_joint_states.at(joint_index);
  }

  const JointStateBase<Geometry> &joint(const std::string &joint_name) const {
    return *_joint_states.at(_robot_info->joints().index(joint_name));
  }
  JointStateBase<Geometry> &joint(const std::string &joint_name) {
    return *_joint_states.at(_robot_info->joints().index(joint_name));
  }

  void serializePositions(
      AlignedStdVector<typename Geometry::Scalar> &joint_positions) const {
    joint_positions.resize(_robot_info->variableCount(),
                           typename Geometry::Value(0.0));
    for (size_t i = 0; i < _robot_info->joints().count(); i++) {
      auto &joint_info = _robot_info->joints().info(i);
      auto *joint_state = _joint_states.at(i).get();
      joint_state->serializePositions(joint_positions.data() +
                                      joint_info.firstVariableIndex());
    }
  }

  void deserializePositions(
      const AlignedStdVector<typename Geometry::Scalar> &joint_positions) {
    if (joint_positions.size() < _robot_info->variableCount()) {
      throw std::runtime_error("wrong array size");
    }
    for (size_t i = 0; i < _robot_info->joints().count(); i++) {
      auto &joint_info = _robot_info->joints().info(i);
      auto *joint_state = _joint_states.at(i).get();
      joint_state->deserializePositions(joint_positions.data() +
                                        joint_info.firstVariableIndex());
    }
  }

  template <class RobotState> void toMoveIt(RobotState &moveit_state) const {
    AlignedStdVector<typename Geometry::Scalar> pp;
    serializePositions(pp);
    for (size_t i = 0; i < pp.size(); i++) {
      moveit_state.setVariablePosition(i, value(pp[i]));
    }
  }

  template <class RobotState> void fromMoveIt(RobotState &moveit_state) {
    AlignedStdVector<typename Geometry::Scalar> pp(_robot_info->variableCount(),
                                                   typename Geometry::Scalar());
    for (size_t i = 0; i < pp.size(); i++) {
      // std::cout << i << " " << pp[i] << std::endl;
      pp[i] = typename Geometry::Value(moveit_state.getVariablePosition(i));
      // std::cout << i << " " << pp[i] << std::endl;
    }
    deserializePositions(pp);
  }
};

} // namespace tractor
