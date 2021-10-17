// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "collision.h"
#include <tractor/collision/shape.h>

#include <deque>
#include <random>

#include <eigen_conversions/eigen_kdl.h>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/rotationalinertia.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/collision_detection/collision_matrix.h>

namespace kdl_parser {
KDL::RigidBodyInertia toKdl(urdf::InertialSharedPtr i);
}

namespace tractor {

template <class Geometry> struct PhysicsSimulator {
  RobotState<Geometry> _robot_state, _previous_robot_state;
  std::shared_ptr<tractor::RobotModel<Geometry>> _robot_model;
  std::shared_ptr<tractor::CollisionRobot<
      typename BatchScalar<typename Geometry::Value>::Type>>
      _collision_model;
  std::deque<
      ShapeCollisionPair<typename BatchScalar<typename Geometry::Value>::Type>>
      _shape_collision_pairs;
  typename Geometry::Scalar _time_step = typename Geometry::Value(0.01);
  size_t _iterations = 1;

  typename Geometry::Vector3 _gravity =
      Geometry::pack(typename Geometry::Value(0), typename Geometry::Value(0),
                     typename Geometry::Value(-9.81));

  std::function<void(PhysicsSimulator<Geometry> &)> _user_constraints;

  std::vector<uint8_t> _is_link_dynamic;
  std::vector<size_t> _link_to_parent_joint;
  std::set<std::pair<size_t, size_t>> _collision_link_pairs;

  typename Geometry::Value _contact_stiffness = typename Geometry::Value(50);
  typename Geometry::Value _contact_damping = typename Geometry::Value(1);
  typename Geometry::Scalar _controller_stiffness =
      typename Geometry::Value(20);
  typename Geometry::Scalar _friction_coefficient =
      typename Geometry::Value(0.5);
  typename Geometry::Scalar _friction_gain = typename Geometry::Value(0.1);
  typename Geometry::Scalar _contact_smoothness =
      typename Geometry::Value(0.005);
  typename Geometry::Scalar _joint_admittance = typename Geometry::Value(1000);

  AlignedStdVector<typename Geometry::Vector3> _friction_forces;

  enum class ControlMode {
    None,
    Velocity,
    Position,
  };
  struct Controller {
    ControlMode mode = ControlMode::None;
    typename Geometry::Scalar command = typename Geometry::Value(0);
  };
  AlignedStdVector<Controller> _controllers;

public:
  struct Body {
    std::string name;
    ssize_t joint = -1;
    ssize_t link = -1;
    ssize_t parent_link = -1;
    bool dynamic = false;
    typename Geometry::Vector3 center = Geometry::Vector3Zero();
    typename Geometry::Vector3 center_global = Geometry::Vector3Zero();
    typename Geometry::Vector3 position = Geometry::Vector3Zero();
    typename Geometry::Orientation orientation =
        Geometry::OrientationIdentity();
    typename Geometry::Vector3 linear_momentum = Geometry::Vector3Zero();
    typename Geometry::Vector3 angular_momentum = Geometry::Vector3Zero();
    typename Geometry::Vector3 linear_velocity = Geometry::Vector3Zero();
    typename Geometry::Vector3 angular_velocity = Geometry::Vector3Zero();
    typename Geometry::Scalar mass = Geometry::ScalarZero();
    typename Geometry::Scalar inverse_mass = Geometry::ScalarZero();
    typename Geometry::Matrix3 inverse_inertia = Geometry::Matrix3Zero();
    typename Geometry::Pose inverse_anchor = Geometry::PoseIdentity();
    std::vector<size_t> links;
  };

private:
  std::deque<Body, AlignedStdAlloc<Body>> _bodies;

  struct Contact {
    size_t body_a = 0;
    size_t body_b = 0;
    size_t link_a = 0;
    size_t link_b = 0;
    typename Geometry::Vector3 point = Geometry::Vector3Zero();
    typename Geometry::Scalar distance = typename Geometry::Value(0);
    typename Geometry::Vector3 axis = Geometry::Vector3Zero();
  };
  std::deque<Contact, AlignedStdAlloc<Body>> _contacts;

  typename Geometry::Vector3
  _velocity(size_t link, const typename Geometry::Vector3 &point) {
    return (point -
            _previous_robot_state.links().pose(link) *
                (Geometry::inverse(_robot_state.links().pose(link)) * point)) *
           (typename Geometry::Value(1) / _time_step);
  }

  void _applyImpulse(size_t body, const typename Geometry::Vector3 &point,
                     const typename Geometry::Vector3 &impulse) {
    if (_bodies[body].dynamic) {
      _bodies[body].linear_momentum += impulse;
      _bodies[body].angular_momentum +=
          cross(point - _bodies[body].center_global, impulse);
    }
  }

  void _applyImpulse(size_t body, size_t link,
                     const typename Geometry::Vector3 &point,
                     const typename Geometry::Vector3 &impulse) {

    _applyImpulse(body, point, impulse);

    while (true) {
      size_t joint = _link_to_parent_joint[link];
      auto &link_pose = _robot_state.links().pose(link);
      auto &joint_state = _robot_state.joints().joint(joint);
      auto &joint_model = _robot_model->joint(joint);
      if (auto *revolute_joint_model =
              dynamic_cast<const RevoluteJointModel<Geometry> *>(
                  &joint_model)) {
        if (auto *revolute_joint_state =
                dynamic_cast<RevoluteJointState<Geometry> *>(&joint_state)) {
          auto joint_axis =
              Geometry::orientation(link_pose) * revolute_joint_model->axis();
          auto joint_anchor = Geometry::translation(link_pose);
          auto joint_impulse =
              dot(cross(joint_axis, point - joint_anchor), impulse);
          revolute_joint_state->position() += joint_impulse * _joint_admittance;
          // std::cout << "joint impulse " << joint_impulse << std::endl;
        }
      }
      auto &joint_info = _robot_model->info()->joints().info(joint);
      if (joint_info.hasParentLink()) {
        link = joint_info.parentLinkIndex();
      } else {
        break;
      }
    }
  }

  void _applyForce(size_t body, size_t link,
                   const typename Geometry::Vector3 &point,
                   const typename Geometry::Vector3 &force) {
    _applyImpulse(body, link, point, force * _time_step);
  }

  void _updateBodyPoses() {
    for (auto &body : _bodies) {
      body.center_global = body.position + body.orientation * body.center;
    }
  }

  void _applyGravity() {
    for (auto &body : _bodies) {
      if (body.dynamic) {
        body.linear_momentum += _gravity * (body.mass * _time_step);
      }
    }
  }

  void _applyDamping() {
    typename Geometry::Scalar linear_decay = typename Geometry::Value(0.01);
    typename Geometry::Scalar angular_decay = typename Geometry::Value(0.01);
    // typename Geometry::Scalar linear_decay_x = pow(linear_decay, _time_step);
    // typename Geometry::Scalar angular_decay_x = pow(angular_decay,
    // _time_step);
    typename Geometry::Scalar linear_decay_x =
        exp(log(linear_decay) * _time_step);
    typename Geometry::Scalar angular_decay_x =
        exp(log(angular_decay) * _time_step);
    for (auto &body : _bodies) {
      if (body.dynamic) {
        body.linear_momentum *= linear_decay_x;
        body.angular_momentum *= angular_decay_x;
      }
    }
  }

  void _detectContacts() {
    _contacts.clear();
    size_t collision_pair_counter = 0;
    for (size_t i_body_a = 0; i_body_a < _bodies.size(); i_body_a++) {
      for (size_t i_body_b = 0; i_body_b < _bodies.size(); i_body_b++) {
        auto &body_a = _bodies[i_body_a];
        auto &body_b = _bodies[i_body_b];
        for (size_t i_link_a : body_a.links) {
          for (size_t i_link_b : body_b.links) {
            if (_collision_link_pairs.find(std::make_pair(
                    i_link_a, i_link_b)) == _collision_link_pairs.end()) {
              continue;
            }

            if ((_is_link_dynamic[i_link_a] && _is_link_dynamic[i_link_b] &&
                 i_body_a < i_body_b) ||
                (!_is_link_dynamic[i_link_a] && _is_link_dynamic[i_link_b] &&
                 i_body_a != i_body_b)) {

              auto &pose_a = _robot_state.links().pose(i_link_a);
              auto &pose_b = _robot_state.links().pose(i_link_b);

              for (auto &shape_a : _collision_model->link(i_link_a)->shapes()) {
                for (auto &shape_b :
                     _collision_model->link(i_link_b)->shapes()) {
                  if (_is_link_dynamic[i_link_a]) {
#if 1
                    typename Geometry::Vector3 point_a, point_b, axis, local_a,
                        local_b;
                    if (_shape_collision_pairs.size() <=
                        collision_pair_counter) {
                      _shape_collision_pairs.emplace_back(shape_a, shape_b);
                    }
                    collision_axes(
                        pose_a, pose_b,
                        uint64_t(
                            &_shape_collision_pairs[collision_pair_counter]),
                        point_a, point_b, axis, local_a, local_b);
                    collision_pair_counter++;
                    point_a = pose_a * local_a;
                    point_b = pose_b * local_b;
                    auto distance = dot(axis, point_a - point_b);
                    _contacts.emplace_back();
                    auto &contact = _contacts.back();
                    contact.body_a = i_body_a;
                    contact.body_b = i_body_b;
                    contact.point =
                        (point_a + point_b) * typename Geometry::Value(0.5);
                    contact.distance = distance;
                    contact.axis = axis;
                    contact.link_a = i_link_a;
                    contact.link_b = i_link_b;
#endif
                  } else {
#if 1
                    if (auto *polyhedron_b =
                            dynamic_cast<const ConvexPolyhedralCollisionShape<
                                typename Geometry::Value> *>(shape_b.get())) {
                      for (auto &vertex_b : polyhedron_b->points()) {
                        typename Geometry::Vector3 point = pose_b * vertex_b;
                        typename Geometry::Vector3 axis;
                        typename Geometry::Scalar distance;
                        collision_project(
                            Geometry::inverse(pose_a) * point,
                            uint64_t(
                                (CollisionShape<typename Geometry::Value> *)
                                    shape_a.get()),
                            axis, distance);
                        axis = Geometry::orientation(pose_a) * axis;
                        _contacts.emplace_back();
                        auto &contact = _contacts.back();
                        contact.body_a = i_body_a;
                        contact.body_b = i_body_b;
                        contact.point = point;
                        contact.distance = distance;
                        contact.axis = axis;
                        contact.link_a = i_link_a;
                        contact.link_b = i_link_b;
                      }
                    }
#endif
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  void _resolveContacts() {

    if (_friction_forces.empty()) {
      _friction_forces.resize(_contacts.size(), Geometry::Vector3Zero());
    }

    for (size_t contact_index = 0; contact_index < _contacts.size();
         contact_index++) {
      auto &contact = _contacts.at(contact_index);

      auto normal = normalized(contact.axis);

      auto normal_force =
          _barrier(contact.distance, _contact_smoothness) * _contact_stiffness;

      auto slip_vector = _velocity(contact.link_b, contact.point) -
                         _velocity(contact.link_a, contact.point);

      auto damping_force = slip_vector * (normal_force * _contact_damping);

      auto &friction_force = _friction_forces.at(contact_index);
      friction_force += slip_vector * (normal_force * _friction_gain);
      friction_force -= normal * dot(normal, friction_force);

      auto friction_limit = normal_force * _friction_coefficient;
      auto friction_force_norm = norm(friction_force);

      friction_force =
          friction_force *
          ((friction_limit - relu(friction_limit - friction_force_norm)) /
           (friction_force_norm + typename Geometry::Value(1e-6)));

      auto force_vector =
          contact.axis * normal_force + damping_force + friction_force;

      _applyForce(contact.body_a, contact.link_a, contact.point, force_vector);
      _applyForce(contact.body_b, contact.link_b, contact.point, -force_vector);
    }
  }

  typename Geometry::Scalar
  _barrier(const typename Geometry::Scalar &distance,
           const typename Geometry::Scalar &smoothness) {
    return log(typename Geometry::Value(1) +
               exp(distance * (typename Geometry::Value(-1) / smoothness))) *
           smoothness;
  }

  void _integrate() {
    for (auto &body : _bodies) {
      // body.linear_momentum *= 0.9;
      // body.angular_momentum *= 0.9;

      auto local_angular_momentum =
          Geometry::inverse(body.orientation) * body.angular_momentum;

      body.linear_velocity = body.linear_momentum * body.inverse_mass;
      body.position += body.linear_velocity * _time_step;

      body.angular_velocity =
          body.orientation * (body.inverse_inertia * local_angular_momentum);

      typename Geometry::Vector3 rotation = body.angular_velocity * _time_step;
      body.position -= cross(rotation, body.orientation * body.center);

      body.orientation += rotation;

      body.angular_momentum = body.orientation * local_angular_momentum;

      // body.linear_momentum = Geometry::Vector3Zero();
      // body.angular_momentum = Geometry::Vector3Zero();
    }
  }

  void _updateRobotState() {
    for (auto &body : _bodies) {
      if (auto *joint_state =
              dynamic_cast<tractor::FloatingJointState<Geometry> *>(
                  &_robot_state.joints().joint(body.joint))) {
        auto &joint_info = _robot_model->info()->joints().info(body.joint);
        joint_state->pose(body.inverse_anchor *
                          Geometry::translationPose(body.position) *
                          Geometry::orientationPose(body.orientation));
      }
    }
  }

  void _applyControllers() {
    for (size_t i = 0; i < _robot_model->info()->joints().size(); i++) {
      if (auto *revolute_joint_state =
              dynamic_cast<tractor::RevoluteJointState<Geometry> *>(
                  &_robot_state.joints().joint(i))) {
        auto &pos = revolute_joint_state->position();
        auto &controller = _controllers[i];
        switch (controller.mode) {
        case ControlMode::Velocity:
          pos += controller.command * _time_step;
          break;
        case ControlMode::Position:
          pos +=
              (controller.command - pos) * _time_step * _controller_stiffness;
          break;
        }
      }
    }
  }

  void _loadLink(Body &body, KDL::RigidBodyInertia &inertia,
                 const Eigen::Isometry3d &transform,
                 const moveit::core::RobotModel &robot_model,
                 const moveit::core::LinkModel *link_model,
                 const Eigen::Isometry3d &anchor, bool dynamic) {

    _is_link_dynamic[link_model->getLinkIndex()] = dynamic;

    body.links.push_back(link_model->getLinkIndex());

#if 0
    std::cout << "body " << body.name << " link " << link_model->getName()
              << std::endl;
#endif

    if (auto urdf_link =
            robot_model.getURDF()->getLink(link_model->getName())) {
      if (auto urdf_inertial = urdf_link->inertial) {
        KDL::Frame kdl_frame;
        tf::transformEigenToKDL(transform, kdl_frame);
        auto kdl_inertia = kdl_parser::toKdl(urdf_inertial);
        inertia = inertia + kdl_frame * kdl_inertia;
      }
    }

    for (auto *child_joint : link_model->getChildJointModels()) {
      auto *child_link = child_joint->getChildLinkModel();
      if (child_joint->getType() != moveit::core::JointModel::FLOATING) {
        _loadLink(
            body, inertia,
            Eigen::Isometry3d(
                (transform * child_link->getJointOriginTransform()).matrix()),
            robot_model, child_link,
            Eigen::Isometry3d(
                (anchor * child_link->getJointOriginTransform()).matrix()),
            (child_joint->getType() != moveit::core::JointModel::FIXED));
      } else {
        _loadBody(robot_model, child_joint, anchor, true);
      }
    }
  }

  void _loadBody(const moveit::core::RobotModel &robot_model,
                 const moveit::core::JointModel *joint_model,
                 const Eigen::Isometry3d &parent_anchor, bool dynamic) {

    Eigen::Isometry3d anchor = Eigen::Isometry3d(
        (parent_anchor *
         joint_model->getChildLinkModel()->getJointOriginTransform())
            .matrix());

    _bodies.emplace_back();
    auto &body = _bodies.back();
    body.joint = joint_model->getJointIndex();
    body.name = joint_model->getChildLinkModel()->getName();
    body.link = joint_model->getChildLinkModel()->getLinkIndex();
    body.inverse_anchor = Geometry::import(Eigen::Isometry3d(anchor.inverse()));
    if (auto *parent_link = joint_model->getParentLinkModel()) {
      body.parent_link = parent_link->getLinkIndex();
    } else {
      body.parent_link = -1;
    }

    KDL::RigidBodyInertia inertia = KDL::RigidBodyInertia::Zero();
    _loadLink(body, inertia, Eigen::Isometry3d::Identity(), robot_model,
              joint_model->getChildLinkModel(), anchor, dynamic);

    if ((inertia.getMass() > 0) &&
        (joint_model->getType() == moveit::core::JointModel::FLOATING)) {
      body.dynamic = true;
      body.center = Geometry::import(Eigen::Vector3d(
          inertia.getCOG().x(), inertia.getCOG().y(), inertia.getCOG().z()));
      body.mass = typename Geometry::Value(inertia.getMass());
      body.inverse_mass = typename Geometry::Value(1.0 / inertia.getMass());
      body.inverse_inertia = Geometry::import(Eigen::Matrix3d(
          Eigen::Map<const Eigen::Matrix3d>(inertia.getRotationalInertia().data)
              .inverse()));
    } else {
      body.dynamic = false;
      body.center = Geometry::Vector3Zero();
      body.mass = typename Geometry::Value(0);
      body.inverse_mass = typename Geometry::Value(0);
      body.inverse_inertia = Geometry::Matrix3Zero();
    }

#if 0
      std::cout << "body " << body.name << " " << body.center << " "
                << body.inverse_mass << " " << body.inverse_inertia
                << std::endl;
#endif
  }

  typedef CollisionRobot<typename BatchScalar<typename Geometry::Value>::Type>
      CollisionRobotType;

public:
  PhysicsSimulator(const moveit::core::RobotModel &robot_model,
                   const collision_detection::AllowedCollisionMatrix &acm =
                       collision_detection::AllowedCollisionMatrix())
      : _robot_model(std::make_shared<RobotModel<Geometry>>(robot_model)),

        /*
          _collision_model(
              std::make_shared<CollisionRobot<
                  typename BatchScalar<typename Geometry::Value>::Type>>(
                  robot_model, false))
                  */

        _collision_model(
            std::allocate_shared<CollisionRobotType
                                 // AlignedStdAlloc<CollisionRobotType>,

                                 >(AlignedStdAlloc<CollisionRobotType>(),
                                   robot_model, false))

  // std::allocate_shared<T, AlignedStdAlloc<T>, const T &>(
  //    AlignedStdAlloc<T>(), *std::dynamic_pointer_cast<T>(instance));

  {

    _is_link_dynamic.resize(robot_model.getLinkModelCount(), false);

    _controllers.resize(robot_model.getJointModelCount());

    _loadBody(robot_model, robot_model.getRootJoint(),
              Eigen::Isometry3d::Identity(), false);

    _link_to_parent_joint.resize(robot_model.getLinkModelCount(), 0);
    for (size_t i = 0; i < _link_to_parent_joint.size(); i++) {
      _link_to_parent_joint[i] =
          robot_model.getLinkModel(i)->getParentJointModel()->getJointIndex();
    }

    for (size_t i_link_a = 0; i_link_a < robot_model.getLinkModelCount();
         i_link_a++) {
      for (size_t i_link_b = 0; i_link_b < robot_model.getLinkModelCount();
           i_link_b++) {
        if (i_link_a != i_link_b) {
          auto &name_a = robot_model.getLinkModelNames()[i_link_a];
          auto &name_b = robot_model.getLinkModelNames()[i_link_b];
          collision_detection::AllowedCollision::Type allowed =
              collision_detection::AllowedCollision::NEVER;
          if (!acm.getEntry(name_a, name_b, allowed)) {
            allowed = collision_detection::AllowedCollision::NEVER;
          }
          if (allowed == collision_detection::AllowedCollision::CONDITIONAL) {
            throw std::runtime_error("conditional collisions not supported");
          }
          if (allowed == collision_detection::AllowedCollision::NEVER) {
            _collision_link_pairs.insert(std::make_pair(i_link_a, i_link_b));
          }
        }
      }
    }
  }

  auto &state() const { return _robot_state; }
  auto &state() { return _robot_state; }

  void init(const RobotState<Geometry> &state) {
    _robot_state = state;
    _robot_model->computeFK(_robot_state.joints(), _robot_state.links());
    _previous_robot_state = _robot_state;
    for (auto &body : _bodies) {
      body.linear_momentum = Geometry::Vector3Zero();
      body.angular_momentum = Geometry::Vector3Zero();
      body.position =
          Geometry::translation(_robot_state.links().pose(body.link));
      body.orientation =
          Geometry::orientation(_robot_state.links().pose(body.link));
      body.linear_velocity = Geometry::Vector3Zero();
      body.angular_velocity = Geometry::Vector3Zero();
    }
    for (auto &controller : _controllers) {
      controller.mode = ControlMode::None;
    }
    _friction_forces.clear();
  }

  void step() {
    for (size_t i = 0; i < _iterations; i++) {
      //_previous_robot_state = _robot_state;
      _updateBodyPoses();
      _applyGravity();
      _applyDamping();
      _detectContacts();
      _resolveContacts();
      _previous_robot_state = _robot_state;
      if (_user_constraints) {
        _user_constraints(*this);
      }
      _integrate();
      _updateRobotState();
      _applyControllers();
      _robot_model->computeFK(_robot_state.joints(), _robot_state.links());
    }
  }

  void setGravity(const typename Geometry::Vector3 &gravity) {
    _gravity = gravity;
  }

  void moveBody(const std::string &name,
                const typename Geometry::Vector3 &offset) {
    for (auto &body : _bodies) {
      if (body.name == name) {
        body.position += offset;
        return;
      }
    }
    throw std::runtime_error("body not found: " + name);
  }

  void rotateBody(const std::string &name,
                  const typename Geometry::Orientation &offset) {
    for (auto &body : _bodies) {
      if (body.name == name) {
        body.orientation = offset * body.orientation;
        return;
      }
    }
    throw std::runtime_error("body not found: " + name);
  }

  void setBodyPose(const std::string &name,
                   const typename Geometry::Pose &pose) {
    for (auto &body : _bodies) {
      if (body.name == name) {
        body.position = Geometry::translation(pose);
        body.orientation = Geometry::orientation(pose);
        return;
      }
    }
    throw std::runtime_error("body not found: " + name);
  }

  auto &model() const { return _robot_model; }

  void controlJointVelocity(size_t joint_index,
                            const typename Geometry::Scalar &velocity) {
    auto &controller = _controllers[joint_index];
    controller.mode = ControlMode::Velocity;
    controller.command = velocity;
  }
  void controlJointVelocity(const std::string &joint_name,
                            const typename Geometry::Scalar &velocity) {
    controlJointVelocity(_robot_model->info()->joints().index(joint_name),
                         velocity);
  }

  void controlJointPosition(size_t joint_index,
                            const typename Geometry::Scalar &position) {
    auto &controller = _controllers[joint_index];
    controller.mode = ControlMode::Position;
    controller.command = position;
  }
  void controlJointPosition(const std::string &joint_name,
                            const typename Geometry::Scalar &position) {
    controlJointPosition(_robot_model->info()->joints().index(joint_name),
                         position);
  }

  auto &body(const std::string &name) {
    for (auto &body : _bodies) {
      if (body.name == name) {
        return body;
      }
    }
    throw std::runtime_error("body not found: " + name);
  }

  void setUserConstraints(
      const std::function<void(PhysicsSimulator<Geometry> &)> &f) {
    _user_constraints = f;
  }

  void setJointPosition(const std::string &joint_name,
                        const typename Geometry::Scalar &position) {
    for (size_t i = 0; i < _robot_model->info()->joints().size(); i++) {
      if (_robot_model->info()->joints().info(i).name() == joint_name) {
        if (auto *revolute_joint_state =
                dynamic_cast<tractor::RevoluteJointState<Geometry> *>(
                    &_robot_state.joints().joint(i))) {
          auto &pos = revolute_joint_state->position();
          pos = position;
        }
      }
    }
  }

  void applyForce(size_t body, const typename Geometry::Vector3 &point,
                  const typename Geometry::Vector3 &force) {
    _applyImpulse(body, point, force * _time_step);
  }

  typename Geometry::Vector3 velocity(const std::string &link,
                                      const typename Geometry::Vector3 &point) {
    return _velocity(_robot_model->info()->links().index(link), point);
  }
};

} // namespace tractor
