// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "dexenv.h"
#include "dexlearn.h"

namespace tractor {

template <class ValueSingle, class ValueBatch>
struct DexEnvGrasp : tractor::DexEnv<ValueSingle, ValueBatch> {

  typedef tractor::Var<ValueSingle> ScalarSingle;
  typedef tractor::GeometryFast<ScalarSingle> GeometrySingle;

  typedef tractor::Var<ValueBatch> ScalarBatch;
  typedef tractor::GeometryFast<ScalarBatch> GeometryBatch;

  DexEnvGrasp() {

    this->_info.name = "grasp";

    this->_info.frame_count = 20;

    this->_info.collision_avoidance_distance = 0.01;
    this->_info.collision_avoidance_weight = 0.02;

    this->_info.slip_avoidance_distance = 0.01;
    this->_info.slip_avoidance_weight = 0.1;

    this->_info.friction_cone_penalty = 1;

    this->_info.contact_distance_penalty = 5;
    this->_info.contact_slip_penalty = 0.1;

    this->_info.joint_limit_penalty = 0.5;

    this->_info.shape_penalty = 1;

    this->_info.contact_point_regularization = 0.1;
    this->_info.contact_force_regularization = 0.1;

    this->_info.collision_penalty = 1;

    this->_info.end_effectors = {
        "ffdistal", "mfdistal", "thdistal", "rfdistal", "lfdistal", "floor",
    };
  }

  virtual tractor::Tensor<ScalarBatch>
  makePolicyInput(tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn,
                  size_t frame) override {

    auto &joint_names = dexlearn.jointNames();
    auto &simulator = dexlearn.simulator();

    tractor::Tensor<ScalarBatch> neural_input;

    auto object_pose = simulator->state().links().pose("object");
    auto object_orientation = GeometryBatch::orientation(object_pose);
    auto object_position = GeometryBatch::translation(object_pose);

    neural_input.resize(joint_names.size() + 3 + 6 + 3);

    for (size_t i = 0; i < joint_names.size(); i++) {
      auto *joint = dexlearn.joints()[i];
      auto &joint_state = simulator->state().joints().joint(joint_names[i]);
      if (auto *revolute_joint_state =
              dynamic_cast<tractor::RevoluteJointState<GeometryBatch> *>(
                  &joint_state)) {
        neural_input(i) = revolute_joint_state->position();
      }
    }

    {
      size_t i = joint_names.size();

      ScalarBatch pscale = ValueBatch(10);
      ScalarBatch px, py, pz;
      GeometryBatch::unpack(object_position, px, py, pz);
      neural_input(i++) = px * pscale;
      neural_input(i++) = py * pscale;
      neural_input(i++) = pz * pscale;

      ScalarBatch rxx, rxy, rxz;
      GeometryBatch::unpack(
          object_orientation *
              GeometryBatch::pack(ValueBatch(1), ValueBatch(0), ValueBatch(0)),
          rxx, rxy, rxz);
      neural_input(i++) = rxx;
      neural_input(i++) = rxy;
      neural_input(i++) = rxz;

      ScalarBatch ryx, ryy, ryz;
      GeometryBatch::unpack(
          object_orientation *
              GeometryBatch::pack(ValueBatch(0), ValueBatch(1), ValueBatch(0)),
          ryx, ryy, ryz);
      neural_input(i++) = ryx;
      neural_input(i++) = ryy;
      neural_input(i++) = ryz;

      ScalarBatch hx, hy, hz;
      GeometryBatch::unpack(
          GeometryBatch::translation(simulator->state().links().pose("palm")) -
              object_position,
          hx, hy, hz);
      neural_input(i++) = hx;
      neural_input(i++) = hy;
      neural_input(i++) = hz;
    }

    return neural_input;
  }

  virtual tractor::NeuralNetwork<ScalarBatch> makePolicyNetwork(
      tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn) override {

    tractor::SequentialNeuralNetwork<ScalarBatch> policy_net;

    auto &joint_names = dexlearn.jointNames();
    auto &end_effectors = dexlearn.endEffectors();

    double regularization = 0.01;

    // policy_net.add(tractor::DropoutLayer<ScalarBatch>(0.3));

    policy_net.add(tractor::DenseLayer<ScalarBatch>(
        64, tractor::Activation::Linear, regularization, regularization));

    policy_net.add(
        tractor::ActivityRegularizationLayer<ScalarBatch>(regularization));

    policy_net.add(tractor::DropoutLayer<ScalarBatch>(0.3));

    policy_net.add(
        tractor::ActivationLayer<ScalarBatch>(tractor::Activation::TanH));

    policy_net.add(tractor::DenseLayer<ScalarBatch>(
        joint_names.size() +
            end_effectors.size() * dexlearn.contactDimensions(),
        tractor::Activation::Linear, regularization, regularization));

    // policy_net.add(tractor::GaussianNoiseLayer<ScalarBatch>(0.001));

    return policy_net;
  }

  virtual void
  init(tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn) override {
    auto &simulator = *dexlearn.simulator();
    {
      double s = 0.02;
      ScalarBatch px = add_random_uniform(this->makeZero(), s * -0.5, s * 0.5);
      ScalarBatch py = add_random_uniform(this->makeZero(), s * -0.5, s * 0.5);
      ScalarBatch pz = ValueBatch(0);
      simulator.moveBody("object", GeometryBatch::pack(px, py, pz));
    }
    {
      auto rot = GeometryBatch::angleAxisOrientation(
          add_random_uniform(this->makeZero(), 0, M_PI * 2),
          GeometryBatch::import(Eigen::Vector3d(0, 0, 1)));
      simulator.rotateBody("object", rot);
    }
  }

  virtual void
  goals(tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn) override {

    dexlearn.addGoal(std::allocate_shared<tractor::MoveGoal<GeometryBatch>>(
        tractor::AlignedStdAlloc<tractor::MoveGoal<GeometryBatch>>(), "object",
        dexlearn.frameCount() - 1,
        GeometryBatch::pack(ValueBatch(0.0), ValueBatch(0.0), ValueBatch(0.1)),
        ValueBatch(2)));

    dexlearn.addGoal(
        std::allocate_shared<tractor::RelativeOrientationGoal<GeometryBatch>>(
            tractor::AlignedStdAlloc<
                tractor::RelativeOrientationGoal<GeometryBatch>>(),
            "object",
            GeometryBatch::pack(ValueBatch(0.0), ValueBatch(0.0),
                                ValueBatch(0.0)),
            ValueBatch(1)));
  }

  virtual void
  controlRobot(tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn,
               const tractor::Tensor<ScalarBatch> &policy_output) override {

    auto &_robot_model = dexlearn.robotModel();
    auto &_group_robot = dexlearn.robotJointGroup();
    auto &joint_names = dexlearn.jointNames();

    auto velocities = policy_output;
    for (size_t i = 0; i < velocities.size(); i++) {
      goal(velocities[i] * ValueBatch(0.01));
      velocities[i] = tanh(velocities[i]);
    }

    std::vector<std::pair<std::string, std::string>> couplings;
    couplings.emplace_back("FFJ1", "FFJ2");
    couplings.emplace_back("RFJ1", "RFJ2");
    couplings.emplace_back("LFJ1", "LFJ2");
    couplings.emplace_back("MFJ1", "MFJ2");
    auto *group = _robot_model->getJointModelGroup(_group_robot);
    for (auto &pair : couplings) {
      int i = group->getVariableGroupIndex(pair.first);
      int j = group->getVariableGroupIndex(pair.second);
      velocities[i] = velocities[j];
    }

    for (size_t i = 0; i < joint_names.size(); i++) {
      auto vel = velocities[i];
      if (joint_names[i].substr(0, 4) == "arm_") {
        vel *= ValueBatch(3);
      } else {
        vel *= ValueBatch(20);
      }
      dexlearn.simulator()->controlJointVelocity(joint_names[i], vel);
    }
  }
};

} // namespace tractor
