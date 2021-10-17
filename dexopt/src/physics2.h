// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "collision.h"
#include <tractor/collision/shape.h>

#include <random>

namespace tractor {

template <class Geometry> class PhysicsSimulator {
  std::vector<uint8_t> _is_robot_part, _is_object_part, _is_moving_part,
      _is_collider;
  std::shared_ptr<tractor::CollisionRobot<typename Geometry::Value>>
      _collision_robot;
  std::vector<typename Geometry::Twist> _link_accelerations;
  std::vector<typename Geometry::Twist> _link_velocities;
  std::vector<typename Geometry::Pose> _link_poses;
  std::vector<typename Geometry::Pose> _previous_link_poses;
  std::deque<ShapeCollisionPair<typename Geometry::Value>>
      _shape_collision_pairs;
  tractor::RobotState<Geometry> _robot_state;
  std::shared_ptr<tractor::RobotModel<Geometry>> _robot_model;
  size_t _link_count = 0;

  void _findDynamicJoints() {
    _is_moving_part.resize(_robot_model->info()->links().size(), 0);
    _is_collider.resize(_robot_model->info()->links().size(), 0);
    _is_robot_part.resize(_robot_model->info()->links().size(), 0);
    _is_object_part.resize(_robot_model->info()->links().size(), 0);
    for (size_t i = 0; i < _robot_model->info()->joints().size(); i++) {
      auto &joint_info = _robot_model->info()->joints().info(i);
      auto child_link = joint_info.childLinkIndex();
      auto parent_link = joint_info.parentLinkIndex();
      if (parent_link >= 0) {
        _is_robot_part.at(child_link) = _is_robot_part.at(parent_link);
        _is_object_part.at(child_link) = _is_object_part.at(parent_link);
      }
      if (joint_info.type() == JointType::Floating) {
        _is_object_part.at(child_link) = true;
        _is_robot_part.at(child_link) = false;
      } else if (joint_info.type() != JointType::Fixed) {
        _is_object_part.at(child_link) = false;
        _is_robot_part.at(child_link) = true;
      }
      _is_moving_part.at(child_link) =
          (_is_object_part.at(child_link) || _is_robot_part.at(child_link));
      _is_collider.at(i) =
          !_collision_robot->link(child_link)->shapes().empty();
    }
  }

public:
  PhysicsSimulator(
      const std::shared_ptr<tractor::RobotModel<Geometry>> &robot_model,
      const std::shared_ptr<tractor::CollisionRobot<typename Geometry::Value>>
          &collision_robot)
      : _robot_model(robot_model), _collision_robot(collision_robot),
        _robot_state(*robot_model),
        _link_count(robot_model->info()->joints().size()) {
    _findDynamicJoints();
    init();
  }

  PhysicsSimulator(const moveit::core::RobotModel &robot_model)
      : PhysicsSimulator(
            std::make_shared<RobotModel<Geometry>>(robot_model),
            std::make_shared<CollisionRobot<typename Geometry::Value>>(
                robot_model)) {}

  bool isObjectLink(size_t i) const { return _is_object_part.at(i); }
  bool isObjectLink(const std::string &name) const {
    return _is_object_part.at(_robot_model->info()->links().index(name));
  }

  auto &state() const { return _robot_state; }
  auto &state() { return _robot_state; }

  auto &model() const { return _robot_model; }

  auto &linkPose(const std::string &name) const {
    return _robot_state.links().pose(name);
  }
  auto &linkPose(size_t i) const { return _robot_state.links().pose(i); }

  auto &linkVelocity(size_t i) const { return _link_velocities.at(i); }
  auto &linkVelocity(const std::string &name) const {
    return linkVelocity(_robot_model->info()->links().index(name));
  }

  void applyForce(size_t link, const typename Geometry::Vector3 &point,
                  const typename Geometry::Vector3 &force) {
    auto &pose = _link_poses.at(link);
    _link_accelerations.at(link) +=
        Geometry::twist(force, cross(point - Geometry::translation(pose),
                                     force * typename Geometry::Value(100)));
  }

  typename Geometry::Vector3
  pointVelocity(size_t link, const typename Geometry::Vector3 &point) {
    return point -
           _previous_link_poses.at(link) *
               (Geometry::inverse(Geometry::orientation(_link_poses.at(link))) *
                (point - Geometry::translation(_link_poses.at(link))));
  }

  void makeContact(const typename Geometry::Vector3 &normal, size_t link_a,
                   const typename Geometry::Vector3 &point_a, size_t link_b,
                   const typename Geometry::Vector3 &point_b,
                   const typename Geometry::Scalar &distance,
                   const typename Geometry::Scalar &weight,
                   const typename Geometry::Scalar &smoothness) {

    typename Geometry::Value stiffness = 1;
    typename Geometry::Value friction = 100;

    auto barrier = log(1 + exp(distance * (-1 / smoothness))) *
                   (smoothness * weight * stiffness);

    auto force_vector = normal * barrier;
    applyForce(link_a, point_a, force_vector);
    applyForce(link_b, point_b, -force_vector);

    // auto friction_point = (point_a + point_b) * 0.5;
    auto friction_force =
        (pointVelocity(link_a, point_a) - pointVelocity(link_b, point_b)) *
        (barrier * friction);
    // friction_force -= normal * dot(normal, friction_force);
    applyForce(link_a, point_a, -friction_force);
    applyForce(link_b, point_b, friction_force);
  }

  void init() {
    _link_accelerations.resize(_link_count);
    _link_velocities.resize(_link_count);
    _link_poses.resize(_link_count);
    for (size_t link_i = 0; link_i < _link_count; link_i++) {
      _link_poses.at(link_i) = _robot_state.links().pose(link_i);
      _link_velocities.at(link_i) =
          Geometry::import(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
      _link_accelerations.at(link_i) =
          Geometry::import(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
    }
    _previous_link_poses = _link_poses;
  }

  void init(const RobotState<Geometry> &state) {
    _robot_state = state;
    init();
  }

  void step() {

    // init accelerations
    for (size_t link_i = 0; link_i < _link_count; link_i++) {
      _link_accelerations.at(link_i) =
          Geometry::import(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
    }

    // apply external forces
    for (size_t link_i = 0; link_i < _link_count; link_i++) {
      if (_is_object_part.at(link_i)) {
        _link_accelerations.at(link_i) += Geometry::import(
            Eigen::Vector3d(0.0, 0.0, -0.002), Eigen::Vector3d(0, 0, 0));
      }
    }

    // generate and apply contact forces
    for (size_t link_a = 0; link_a < _link_count; link_a++) {
      for (size_t link_b = 0; link_b < _link_count; link_b++) {

        if (((_is_moving_part.at(link_a) && _is_moving_part.at(link_b) &&
              link_a > link_b) ||
             (!_is_moving_part.at(link_a) && _is_moving_part.at(link_b))) &&
            _is_collider.at(link_a) && _is_collider.at(link_b)) {

          for (auto &shape_a : _collision_robot->link(link_a)->shapes()) {
            for (auto &shape_b : _collision_robot->link(link_b)->shapes()) {

              auto &pose_a = _link_poses.at(link_a);
              auto &pose_b = _link_poses.at(link_b);

              // axis = Geometry::import(Eigen::Vector3d(0, 0, 1));

              if (_is_moving_part.at(link_a)) {
                // if (1) {

#if 1
                typename Geometry::Vector3 point_a, point_b, axis, local_a,
                    local_b;
                _shape_collision_pairs.emplace_back(shape_a, shape_b);
                collision_axes(pose_a, pose_b,
                               uint64_t(&_shape_collision_pairs.back()),
                               point_a, point_b, axis, local_a, local_b);

                point_a = pose_a * local_a;
                point_b = pose_b * local_b;

                auto distance = dot(axis, point_a - point_b);

                // point_b = point_a = (point_a + point_b) * 0.5;
                std::swap(point_a, point_b);

                makeContact(axis, link_a, point_a, link_b, point_b, distance,
                            0.05, 0.02);
#endif

              } else {

#if 1
                if (auto *polyhedron_a =
                        dynamic_cast<const ConvexPolyhedralCollisionShape<
                            typename Geometry::Value> *>(shape_a.get())) {

                  if (auto *polyhedron_b =
                          dynamic_cast<const ConvexPolyhedralCollisionShape<
                              typename Geometry::Value> *>(shape_b.get())) {

                    // a = fixed

                    // typename Geometry::Value weight =
                    //    2.0 / (1.0 + polyhedron_b->points().size());

                    // auto &pose_a = _previous_link_poses.at(link_a);
                    // auto &pose_b = _previous_link_poses.at(link_b);

                    for (auto &vertex_b : polyhedron_b->points()) {

                      typename Geometry::Vector3 point = pose_b * vertex_b;

                      typename Geometry::Vector3 axis;
                      typename Geometry::Scalar distance;
                      /*
                      collision_project(
                          Geometry::inverse(Geometry::orientation(pose_a)) *
                              (point - Geometry::translation(pose_a)),
                          uint64_t(polyhedron_a), axis, distance);
                      */
                      /*
                      collision_project(
                          (Geometry::inverse(Geometry::orientation(pose_a)) *
                           point) -
                              Geometry::translation(pose_a),
                          uint64_t(polyhedron_a), axis, distance);
                          */
                      collision_project(Geometry::inverse(pose_a) * point,
                                        uint64_t(polyhedron_a), axis, distance);

                      axis = Geometry::orientation(pose_a) * axis;

                      makeContact(axis, link_a, point, link_b, point, distance,
                                  0.5, 0.001);
                    }
                  }
                }
#endif
              }
            }
          }
        }
      }
    }

    // integrate accelerations -> velocities
    typename Geometry::Scalar friction = 0.1;
    for (size_t link_i = 0; link_i < _link_count; link_i++) {
      if (_is_object_part.at(link_i)) {
        _link_velocities.at(link_i) *= 1 - friction;
        _link_velocities.at(link_i) += _link_accelerations.at(link_i);
        //_link_velocities[link_name] = _link_accelerations[link_name];
      }
    }

    // store previous link poses
    _previous_link_poses = _link_poses;

    // integrate velocities -> positions
    for (size_t link_i = 0; link_i < _link_count; link_i++) {
      if (_is_object_part.at(link_i)) {
        auto previous_pose = _link_poses.at(link_i);
        auto new_pose =
            Geometry::translationPose(Geometry::translation(previous_pose)) *
            (Geometry::identityPose() + _link_velocities.at(link_i)) *
            Geometry::orientationPose(Geometry::orientation(previous_pose));
        _link_poses.at(link_i) = new_pose;
      }
    }
  }

  void simulate(RobotTrajectory<Geometry> &trajectory) {

    init(trajectory.state(0));

    for (size_t frame_index = 1; frame_index < trajectory.size();
         frame_index++) {

      for (auto &link_name : model()->info()->links().names()) {
        if (!isObjectLink(link_name)) {
          _link_poses.at(model()->info()->links().index(link_name)) =
              trajectory.state(frame_index - 1).links().pose(link_name);
        }
      }

      step();

      for (auto &link_name : model()->info()->links().names()) {
        if (isObjectLink(link_name)) {
          trajectory.state(frame_index).links().pose(link_name) =
              _link_poses.at(model()->info()->links().index(link_name));
        }
      }

      for (auto &joint_name : model()->info()->joints().names()) {
        auto &joint_info = model()->info()->joints().info(joint_name);
        if (isObjectLink(joint_info.childLinkIndex())) {
          if (auto *joint_state =
                  dynamic_cast<tractor::FloatingJointState<Geometry> *>(
                      &trajectory.state(frame_index)
                           .joints()
                           .joint(joint_name))) {
            auto pose =
                Geometry::inverse(_link_poses.at(joint_info.parentLinkIndex()) *
                                  Geometry::import(joint_info.origin())) *
                _link_poses.at(joint_info.childLinkIndex());
            joint_state->pose(pose);
          }
        }
      }
    }
  }
};

template <class Geometry> struct NewPhysicsGoal : public MotionGoal<Geometry> {

  PhysicsSimulator<Geometry> _simulator;

  NewPhysicsGoal(const moveit::core::RobotModel &robot_model)
      : _simulator(robot_model) {}

  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();

    _simulator.init(trajectory.state(0));

    for (size_t frame_index = 1; frame_index < trajectory.size();
         frame_index++) {

      //_simulator.state() = trajectory.state(frame_index - 1);
      for (auto &link_name : _simulator.model()->info()->links().names()) {
        if (!_simulator.isObjectLink(link_name)) {
          _simulator.state().links().pose(link_name) =
              trajectory.state(frame_index).links().pose(link_name);
        }
      }

      _simulator.step();

      // goals
      if (_simulator.model()->info()->links().contains("object")) {
        goal(_simulator.linkVelocity("object") -
             Geometry::import(Eigen::Vector3d(0.0, 0.0, 0.0),
                              Eigen::Vector3d(0.0, 0.0, -0.1)) *
                 5);
        goal(dot(Geometry::translation(_simulator.linkPose("object")),
                 Geometry::import(Eigen::Vector3d(1, 0, 0))));
        goal(dot(Geometry::translation(_simulator.linkPose("object")),
                 Geometry::import(Eigen::Vector3d(0, 1, 0))));
      }

      // apply positions
      for (auto &link_name : _simulator.model()->info()->links().names()) {
        if (_simulator.isObjectLink(link_name)) {
          tractor::goal(Geometry::residual(
              _simulator.state().links().pose(link_name),
              trajectory.state(frame_index).links().pose(link_name)));
        }
      }
    }
  }
};

} // namespace tractor
