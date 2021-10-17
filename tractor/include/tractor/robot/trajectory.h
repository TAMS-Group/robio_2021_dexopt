// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "robotstate.h"

namespace tractor {

template <class Geometry> class RobotTrajectory {
  std::deque<RobotState<Geometry>, AlignedStdAlloc<RobotState<Geometry>>>
      _states;

public:
  void init(const RobotModel<Geometry> &robot_model, size_t frame_count) {
    _states.clear();
    for (size_t i = 0; i < frame_count; i++) {
      _states.emplace_back(robot_model);
    }
  }
  RobotTrajectory() {}
  RobotTrajectory(const RobotModel<Geometry> &robot_model, size_t frame_count) {
    init(robot_model, frame_count);
  }
  size_t size() const { return _states.size(); }
  auto &state(size_t i) const { return _states.at(i); }
  auto &state(size_t i) { return _states.at(i); }
  void shift(int steps = 1) {
    AlignedStdVector<typename Geometry::Scalar> pp;
    if (steps > 0) {
      for (size_t i = 0; i < _states.size(); i++) {
        _states[std::min(_states.size() - 1, i + 1)]
            .joints()
            .serializePositions(pp);
        _states[i].joints().deserializePositions(pp);
      }
    } else {
      for (ssize_t i = _states.size() - 1; i >= 0; i--) {
        _states[std::max(ssize_t(0), i - 1)].joints().serializePositions(pp);
        _states[i].joints().deserializePositions(pp);
      }
    }
  }
};

} // namespace tractor
