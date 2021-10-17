// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "jointtypes.h"
#include "robotinfo.h"

#include <memory>
#include <vector>

namespace tractor {

template <class Geometry> class RobotModel;

template <class Geometry> class LinkState {
  std::shared_ptr<const RobotInfo> _info;
  AlignedStdVector<typename Geometry::Pose> _poses;

public:
  void init(const RobotModel<Geometry> &robot_model) {
    _info = robot_model.info();
    _poses.resize(robot_model.info()->links().size());
  }

  LinkState() {}
  LinkState(const RobotModel<Geometry> &robot_model) { init(robot_model); }

  size_t size() const { return _poses.size(); }

  const typename Geometry::Pose &pose(size_t link_index) const {
    return _poses.at(link_index);
  }
  typename Geometry::Pose &pose(size_t link_index) {
    return _poses.at(link_index);
  }

  const typename Geometry::Pose &pose(const std::string &link_name) const {
    return _poses.at(_info->links().index(link_name));
  }
  typename Geometry::Pose &pose(const std::string &link_name) {
    return _poses.at(_info->links().index(link_name));
  }
};

} // namespace tractor
