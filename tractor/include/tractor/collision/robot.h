// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/collision/link.h>

#include <unordered_map>

namespace moveit {
namespace core {
class RobotModel;
class LinkModel;
class RobotState;
} // namespace core
} // namespace moveit

namespace tractor {

class CollisionRobotBase {
protected:
  void _load(const moveit::core::RobotModel &robot_model,
             bool merge_fixed_links);

public:
  virtual std::shared_ptr<CollisionLinkBase>
  createLink(const std::string &name) = 0;
};

template <class Scalar> class CollisionRobot : public CollisionRobotBase {
  std::vector<std::shared_ptr<const CollisionLink<Scalar>>> _links;
  std::unordered_map<std::string, std::shared_ptr<CollisionLink<Scalar>>>
      _link_map;

public:
  CollisionRobot() {}
  CollisionRobot(const moveit::core::RobotModel &robot_model,
                 bool merge_fixed_links = true) {
    _load(robot_model, merge_fixed_links);
  }
  const auto &links() const { return _links; }
  const auto &link(const std::string &name) {
    auto it = _link_map.find(name);
    if (it == _link_map.end()) {
      std::string message =
          "Link \"" + name +
          "\" not found in collision model. Known collision links:";
      for (auto &pair : _link_map) {
        message += " " + pair.first;
      }
      throw std::runtime_error(message);
    }
    return it->second;
  }
  const auto &link(size_t i) { return _links.at(i); }
  /*
  bool hasLink(const std::string &name) const {
    return _link_map.find(name) != _link_map.end();
  }
  */
  virtual std::shared_ptr<CollisionLinkBase>
  createLink(const std::string &name) override {
    if (_link_map.find(name) != _link_map.end()) {
      throw std::runtime_error(
          "collision link with the same name already exists");
    }
    auto link = std::make_shared<CollisionLink<Scalar>>(name);
    _links.push_back(link);
    _link_map[name] = link;
    return link;
  }
};

} // namespace tractor
