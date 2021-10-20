// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "dexenv.h"

namespace tractor {

template <class ValueSingle, class ValueBatch> class DexLearn {

  typedef tractor::Var<ValueSingle> ScalarSingle;
  typedef tractor::GeometryFast<ScalarSingle> GeometrySingle;

  typedef tractor::Var<ValueBatch> ScalarBatch;
  typedef tractor::GeometryFast<ScalarBatch> GeometryBatch;

  size_t _contact_dimensions = 9;
  std::shared_ptr<tractor::Engine> _engine;
  tractor::CollisionRobot<ValueSingle> _collision_robot;
  robot_model::RobotModelConstPtr _robot_model;
  std::shared_ptr<tractor::PhysicsSimulator<GeometryBatch>> _simulator;
  std::vector<std::string> _end_effectors;
  std::vector<std::string> _joint_names;
  std::string _group_robot;
  robot_state::RobotState _robot_state;
  std::map<std::pair<std::string, std::string>,
           std::shared_ptr<tractor::ShapeCollisionPair<ValueSingle>>>
      _collision_pairs;
  tractor::JointVariableOptions<GeometryBatch> _joint_variable_options;
  std::shared_ptr<tractor::Solver> _solver;
  RobotTrajectory<GeometryBatch> _test_trajectory;
  std::vector<std::shared_ptr<MotionGoal<GeometryBatch>>> _goals;
  std::vector<const robot_model::JointModel *> _joints;
  std::function<void(tractor::PhysicsSimulator<GeometryBatch> &)> _initializer;
  std::shared_ptr<DexEnv<ValueSingle, ValueBatch>> _env;

  std::deque<ScalarBatch, tractor::AlignedStdAlloc<ScalarBatch>> _zero_buffer;
  auto makeZero() {
    if (_zero_buffer.empty()) {
      _zero_buffer.emplace_back(ValueBatch(0));
      tractor::parameter(_zero_buffer.back());
    }
    return _zero_buffer.back();
  }
  auto makeRandomGaussian() { return add_random_normal(makeZero(), 1.0); }

  DexViz _viz;

  size_t _frames = 1;
  size_t _start = 1;
  size_t _outer_batch_size = 1;

  tractor::NeuralNetwork<ScalarBatch> _policy_net;

  collision_detection::AllowedCollisionMatrix _allowed_collision_matrix;

  static std::pair<std::string, std::string> _make_sorted_pair(std::string a,
                                                               std::string b) {
    if (b < a) {
      return std::make_pair(b, a);
    } else {
      return std::make_pair(a, b);
    }
  }

  void _applyCollisionPenalties() {
    for (auto &link_a : _robot_model->getLinkModelNames()) {
      for (auto &link_b : _robot_model->getLinkModelNames()) {
        if (link_a < link_b) {
          collision_detection::AllowedCollision::Type allowed =
              collision_detection::AllowedCollision::NEVER;
          if (!_allowed_collision_matrix.getEntry(link_a, link_b, allowed)) {
            allowed = collision_detection::AllowedCollision::NEVER;
          }
          if (allowed == collision_detection::AllowedCollision::CONDITIONAL) {
            throw std::runtime_error("conditional collisions not supported");
          }
          if (allowed == collision_detection::AllowedCollision::NEVER) {
            auto &pose_a = _simulator->state().links().pose(link_a);
            auto &pose_b = _simulator->state().links().pose(link_b);
            for (auto &shape_a : _collision_robot.link(link_a)->shapes()) {
              for (auto &shape_b : _collision_robot.link(link_b)->shapes()) {

                typename GeometryBatch::Vector3 point_a, point_b, axis, local_a,
                    local_b;
                auto &collision_pair =
                    _collision_pairs[_make_sorted_pair(link_a, link_b)];
                if (!collision_pair) {
                  collision_pair = std::make_shared<
                      tractor::ShapeCollisionPair<ValueSingle>>(shape_a,
                                                                shape_b);
                }
                collision_axes(pose_a, pose_b, uint64_t(collision_pair.get()),
                               point_a, point_b, axis, local_a, local_b);
                point_a = pose_a * local_a;
                point_b = pose_b * local_b;

                auto distance = dot(axis, point_a - point_b);

                if (_env->info().collision_penalty) {
                  TRACTOR_GOAL(relu(-distance) *
                               ValueBatch(_env->info().collision_penalty));
                }

                if (_env->info().collision_avoidance_weight) {
                  TRACTOR_GOAL(
                      relu(-distance +
                           ValueBatch(
                               _env->info().collision_avoidance_distance)) *
                      ValueBatch(_env->info().collision_avoidance_weight));
                }

                if (_env->info().slip_avoidance_weight) {
                  auto p_center = (point_a + point_b) * ValueBatch(0.5);

                  auto v1 = _simulator->velocity(link_a, p_center);
                  auto v2 = _simulator->velocity(link_b, p_center);

                  TRACTOR_GOAL(
                      (v2 - v1) *
                      (relu(-distance +
                            ValueBatch(_env->info().slip_avoidance_distance)) *
                       ValueBatch(_env->info().slip_avoidance_weight)));
                }
              }
            }
          }
        }
      }
    }
  }

  auto _runPolicyNetwork(const LayerMode &mode, size_t frame) {
    auto input = _env->makePolicyInput(*this, frame);
    auto output = _policy_net.predict(input, mode);
    return output;
  }

  void _applyJointLimitPenalties() {
    for (size_t i = 0; i < _joint_names.size(); i++) {
      auto *joint = _joints[i];
      auto &bounds = joint->getVariableBounds().at(0);

      ValueBatch joint_limit_penalty =
          ValueBatch(_env->info().joint_limit_penalty);
      auto &joint_state = _simulator->state().joints().joint(_joint_names[i]);
      if (auto *revolute_joint_state =
              dynamic_cast<tractor::RevoluteJointState<GeometryBatch> *>(
                  &joint_state)) {
        auto pos = revolute_joint_state->position();
        TRACTOR_GOAL(relu(pos - ValueBatch(bounds.max_position_)) *
                     joint_limit_penalty);
        TRACTOR_GOAL(relu(ValueBatch(bounds.min_position_) - pos) *
                     joint_limit_penalty);
      }
    }
  }

  void
  _applyShapePenalty(const std::string &object_name,
                     const typename GeometryBatch::Vector3 &contact_point) {
    auto object_pose = _simulator->state().links().pose(object_name);
    auto p = GeometryBatch::inverse(object_pose) * contact_point;
    auto &shapes = _collision_robot.link(object_name)->shapes();
    if (shapes.size() != 1) {
      throw std::runtime_error("invalid number of collision shapes");
    }

    auto *shape = shapes.at(0).get();

    auto f = ValueBatch(_env->info().shape_penalty);

    if (auto *cylinder =
            dynamic_cast<const CollisionCylinderShape<ValueSingle> *>(shape)) {

      ScalarBatch px, py, pz;
      GeometryBatch::unpack(p, px, py, pz);

      goal(f * (relu(pz - ValueBatch(cylinder->length() * 0.5)) +
                relu(-pz - ValueBatch(cylinder->length() * 0.5))));

      // goal(f * relu(sqrt(px * px + py * py) -
      // ValueBatch(cylinder->radius())));

      size_t n = 8;
      for (size_t i = 0; i < n; i++) {
        double angle = i * M_PI / n;
        auto v =
            px * ValueBatch(std::sin(angle)) + py * ValueBatch(std::cos(angle));
        goal(f * (relu(v - ValueBatch(cylinder->radius())) +
                  relu(-v - ValueBatch(cylinder->length()))));
      }

      return;
    }

    if (auto *poly =
            dynamic_cast<const ConvexPolyhedralCollisionShape<ValueSingle> *>(
                shape)) {

      // ROS_INFO_STREAM("plane count " << poly->planes().size());
      for (auto &plane : poly->planes()) {
        auto plane_normal = GeometryBatch::pack(ValueBatch(plane.normal().x()),
                                                ValueBatch(plane.normal().y()),
                                                ValueBatch(plane.normal().z()));
        auto plane_offset = ValueBatch(plane.offset());
        auto dist = dot(plane_normal, p) + plane_offset;
        goal(f * relu(dist));
      }

      return;
    }

    throw std::runtime_error(
        std::string() +
        "collision shape not yet supported: " + typeid(*shape).name());
  }

  void _applyFrictionConePenalties(
      const typename GeometryBatch::Vector3 &contact_force,
      const typename GeometryBatch::Vector3 &contact_normal,
      const ScalarBatch &weight) {
    for (int sign : {-1, +1}) {
      for (auto v :
           {GeometryBatch::pack(ValueBatch(1), ValueBatch(0), ValueBatch(0)),
            GeometryBatch::pack(ValueBatch(0), ValueBatch(1), ValueBatch(0)),
            GeometryBatch::pack(ValueBatch(0), ValueBatch(0), ValueBatch(1))}) {
        auto n = contact_normal;
        n += cross(n, v * ValueBatch(sign * 0.5));
        TRACTOR_GOAL(relu(-dot(n, contact_force)) * weight);
      }
    }
  }

  void _evaluateContact(const std::string &end_effector_name,
                        const std::string &object_name,
                        const tractor::Tensor<ScalarBatch> &contact_parameters,
                        size_t end_effector_index) {

    auto object_pose = _simulator->state().links().pose(object_name);
    auto object_orientation = GeometryBatch::orientation(object_pose);
    auto object_position = GeometryBatch::translation(object_pose);

    auto end_effector_pose =
        _simulator->state().links().pose(end_effector_name);
    auto end_effector_orientation =
        GeometryBatch::orientation(end_effector_pose);
    auto end_effector_position = GeometryBatch::translation(end_effector_pose);

    auto contact_point_1 =
        (GeometryBatch::pack(contact_parameters[0], contact_parameters[1],
                             contact_parameters[2]) *
         ValueBatch(0.1));

    if (_env->info().contact_point_regularization) {
      tractor::goal(contact_point_1 *
                    ValueBatch(_env->info().contact_point_regularization));
    }

    // contact_point_1 = object_position + contact_point_1;

    {
      auto &shapes = _collision_robot.link(object_name)->shapes();
      if (shapes.size() != 1) {
        throw std::runtime_error("invalid number of collision shapes");
      }
      auto *shape = shapes.at(0).get();
      contact_point_1 =
          object_pose * GeometryBatch::pack(ValueBatch(shape->center().x()),
                                            ValueBatch(shape->center().y()),
                                            ValueBatch(shape->center().z())) +
          contact_point_1;
    }

    auto contact_point_2 =
        (GeometryBatch::pack(contact_parameters[3], contact_parameters[4],
                             contact_parameters[5]) *
         ValueBatch(0.1));

    if (_env->info().contact_point_regularization) {
      tractor::goal(contact_point_2 *
                    ValueBatch(_env->info().contact_point_regularization));
    }

    // contact_point_2 = end_effector_position + contact_point_2;

    {
      auto &shapes = _collision_robot.link(end_effector_name)->shapes();
      if (shapes.size() != 1) {
        throw std::runtime_error("invalid number of collision shapes");
      }
      auto *shape = shapes.at(0).get();
      contact_point_2 =
          end_effector_pose *
              GeometryBatch::pack(ValueBatch(shape->center().x()),
                                  ValueBatch(shape->center().y()),
                                  ValueBatch(shape->center().z())) +
          contact_point_2;
    }

    auto fx = contact_parameters[6];
    auto fy = contact_parameters[7];
    auto fz = contact_parameters[8];

    {
      ScalarBatch force_scale = ValueBatch(0.1);
      fx *= force_scale;
      fy *= force_scale;
      fz *= force_scale;
    }

    auto contact_force = GeometryBatch::pack(fx, fy, fz);

    if (_env->info().contact_force_regularization) {
      tractor::goal(contact_force *
                    ValueBatch(_env->info().contact_force_regularization));
    }

    _applyShapePenalty(object_name, contact_point_1);
    _applyShapePenalty(end_effector_name, contact_point_2);

    // friction cone penalty
    {
      typename GeometryBatch::Vector3 contact_normal;
      auto *shape = _collision_robot.link(object_name)->shapes().at(0).get();
      auto pos_relative = GeometryBatch::inverse(object_pose) * contact_point_2;
      typename GeometryBatch::Scalar contact_distance;
      collision_project_2(pos_relative, uint64_t(shape), contact_normal,
                          contact_distance);
      contact_normal = object_orientation * contact_normal;

      if (_env->info().friction_cone_penalty) {
        _applyFrictionConePenalties(
            contact_force, contact_normal,
            ValueBatch(_env->info().friction_cone_penalty));
      }

      //   _viz.visualizeContact(indexBatch(value(contact_point_1), 0),
      //                         indexBatch(value(contact_normal), 0),
      //                         indexBatch(value(contact_force), 0), 0);
    }

    // friction cone penalty
    {
      typename GeometryBatch::Vector3 contact_normal;
      auto *shape =
          _collision_robot.link(end_effector_name)->shapes().at(0).get();
      auto pos_relative =
          GeometryBatch::inverse(end_effector_pose) * contact_point_1;
      typename GeometryBatch::Scalar contact_distance;
      collision_project_2(pos_relative, uint64_t(shape), contact_normal,
                          contact_distance);
      contact_normal = end_effector_orientation * contact_normal;

      if (_env->info().friction_cone_penalty) {
        _applyFrictionConePenalties(
            -contact_force, contact_normal,
            ValueBatch(_env->info().friction_cone_penalty));
      }

      //   _viz.visualizeContact(indexBatch(value(contact_point_2), 0),
      //                         indexBatch(value(contact_normal), 0),
      //                         indexBatch(value(contact_force), 0), 1);
    }

    _viz.visualizeContact(
        indexBatch(value(contact_point_1) + value(contact_point_2), 0) * 0.5,
        value(GeometrySingle::Vector3Zero()),
        indexBatch(value(contact_force), 0), end_effector_index);

    if (_env->info().contact_distance_penalty) {
      auto d = (contact_point_2 - contact_point_1) *
               ValueBatch(_env->info().contact_distance_penalty);
      TRACTOR_GOAL(d * fx);
      TRACTOR_GOAL(d * fy);
      TRACTOR_GOAL(d * fz);
    }

    if (_env->info().contact_slip_penalty) {
      auto p = (contact_point_1 + contact_point_2) * ValueBatch(0.5);
      auto v1 = _simulator->velocity(object_name, p);
      auto v2 = _simulator->velocity(end_effector_name, p);
      auto dv = (v2 - v1) * ValueBatch(_env->info().contact_slip_penalty);
      TRACTOR_GOAL(dv * fx);
      TRACTOR_GOAL(dv * fy);
      TRACTOR_GOAL(dv * fz);
    }

    // apply force
    _simulator->applyForce(1, contact_point_1, contact_force);
  }

  auto _runBatch(tractor::RobotTrajectory<GeometryBatch> *trajectory,
                 const LayerMode &mode) {
    _viz.clear();
    trajectory->state(0).joints().init(*_simulator->model());
    _simulator->model()->computeFK(trajectory->state(0).joints(),
                                   trajectory->state(0).links());
    _simulator->init(trajectory->state(0));
    _env->init(*this);
    if (_initializer) {
      _initializer(*_simulator);
    }
    for (size_t frame_index = 0; frame_index < trajectory->size();
         frame_index++) {
      _simulator->step();
      trajectory->state(frame_index) = _simulator->state();
      _applyJointLimitPenalties();
      auto policy_output = _runPolicyNetwork(mode, frame_index);
      _env->controlRobot(*this, policy_output);
      for (size_t end_effector_index = 0;
           end_effector_index < _end_effectors.size(); end_effector_index++) {
        auto &end_effector_name = _end_effectors[end_effector_index];
        tractor::Tensor<ScalarBatch> contact_parameters = policy_output.range(
            _joint_names.size() + end_effector_index * _contact_dimensions,
            _contact_dimensions);
        _evaluateContact(end_effector_name, "object", contact_parameters,
                         end_effector_index);
      }
      _applyCollisionPenalties();
    }
  }

  auto _makeSimulator() {
    planning_scene::PlanningScene planning_scene(_robot_model);
    auto acm1 = planning_scene.getAllowedCollisionMatrix();
    {
      for (auto &a : _robot_model->getLinkModelNames()) {
        for (auto &b : _robot_model->getLinkModelNames()) {
          acm1.setEntry(a, b, true);
        }
      }
    }

    _simulator = std::allocate_shared<tractor::PhysicsSimulator<GeometryBatch>>(
        tractor::AlignedStdAlloc<tractor::PhysicsSimulator<GeometryBatch>>(),
        *_robot_model, acm1);
  }

  auto _runTraining() {
    for (size_t i = 0; i < _outer_batch_size; i++) {
      auto trajectory_optimization =
          std::make_shared<tractor::TrajectoryOptimization<GeometryBatch>>();
      trajectory_optimization->makeTrajectory(
          *_robot_model, _group_robot, _frames, 1, _joint_variable_options,
          &_robot_state, false);
      _runBatch(&trajectory_optimization->trajectory(), LayerMode());
      for (auto &goal : _goals) {
        trajectory_optimization->add(goal);
      }
      trajectory_optimization->applyGoals();
    }
  }

public:
  DexLearn(const std::shared_ptr<Solver> &solver,
           const std::shared_ptr<tractor::Engine> &engine,
           const robot_model::RobotModelConstPtr &robot_model,
           collision_detection::AllowedCollisionMatrix allowed_collision_matrix,
           std::string group_robot,
           const std::shared_ptr<DexEnv<ValueSingle, ValueBatch>> &env,
           size_t outer_batch_size)
      : _solver(solver), _engine(engine), _robot_model(robot_model), _env(env),
        _collision_robot(*robot_model, false),
        _allowed_collision_matrix(allowed_collision_matrix),
        _end_effectors(env->info().end_effectors),
        _frames(env->info().frame_count), _group_robot(group_robot),
        _joint_names(
            _robot_model->getJointModelGroup(group_robot)->getVariableNames()),
        _robot_state(robot_model),
        _test_trajectory(*robot_model, env->info().frame_count),
        _outer_batch_size(outer_batch_size),
        _joints(
            _robot_model->getJointModelGroup(_group_robot)->getJointModels()) {

    _policy_net = env->makePolicyNetwork(*this);
  }

  auto &robotJointGroup() const { return _group_robot; }

  auto &robotModel() const { return _robot_model; }

  auto &policyNetwork() const { return _policy_net; }
  auto &policyNetwork() { return _policy_net; }

  auto &endEffectors() const { return _end_effectors; }

  auto frameCount() const { return _frames; }

  auto &jointNames() const { return _joint_names; }

  auto &joints() const { return _joints; }

  void setInitializer(
      const std::function<void(tractor::PhysicsSimulator<GeometryBatch> &)>
          &initializer) {
    _initializer = initializer;
  }

  auto visualization() const { return _viz.finish(); }

  auto &trajectory() const { return _test_trajectory; }

  auto &simulator() { return _simulator; }
  auto &simulator() const { return _simulator; }

  void addGoal(const std::shared_ptr<MotionGoal<GeometryBatch>> &goal) {
    _goals.push_back(goal);
  }

  void build(const std::function<void()> &goal_factory) {
    tractor::Program program([&]() {
      goal_factory();
      _makeSimulator();
      _runTraining();
    });
    _solver->compile(program);
  }

  size_t contactDimensions() const { return _contact_dimensions; }

  auto &solver() { return _solver; }
  auto &solver() const { return _solver; }

  void step() {
    ROS_INFO_STREAM("optimize");
    _solver->gather();
    _solver->solve();
    _solver->scatter();
  }

  void test(bool training = false) {
    ROS_INFO_STREAM("test");
    LayerMode mode;
    mode.training = training;
    _runBatch(&_test_trajectory, mode);
  }
};

} // namespace tractor
