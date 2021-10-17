// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "dexenv.h"
#include "dexlearn.h"

namespace tractor {

template <class ValueSingle, class ValueBatch>
struct DexEnvTurn : tractor::DexEnv<ValueSingle, ValueBatch> {

  typedef tractor::Var<ValueSingle> ScalarSingle;
  typedef tractor::GeometryFast<ScalarSingle> GeometrySingle;

  typedef tractor::Var<ValueBatch> ScalarBatch;
  typedef tractor::GeometryFast<ScalarBatch> GeometryBatch;

  DexEnvTurn() {

    this->_info.name = "turn";
    this->_info.frame_count = 20;

    this->_info.collision_avoidance_distance = 0.01;
    this->_info.collision_avoidance_weight = 0;

    this->_info.slip_avoidance_distance = 0.01;
    this->_info.slip_avoidance_weight = 100;

    this->_info.friction_cone_penalty = 10;

    this->_info.contact_distance_penalty = 1000;
    this->_info.contact_slip_penalty = 0;

    this->_info.joint_limit_penalty = 1;

    this->_info.shape_penalty = 100;

    this->_info.contact_point_regularization = 1;
    this->_info.contact_force_regularization = 1;

    this->_info.collision_penalty = 100;

    this->_info.end_effectors = {
        "ffdistal", "mfdistal", "thdistal", "rfdistal", "lfdistal",
    };
  }

  virtual tractor::Tensor<ScalarBatch>
  makePolicyInput(tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn,
                  size_t frame) override {

    auto &simulator = dexlearn.simulator();

    auto object_pose = simulator->state().links().pose("object");
    auto object_orientation = GeometryBatch::orientation(object_pose);
    auto object_position = GeometryBatch::translation(object_pose);

    size_t frequencies = 1;

    tractor::Tensor<ScalarBatch> neural_input;
    // neural_input.resize(3 * 3 + frequencies * 2);
    // neural_input.resize(frequencies * 2 + 2 + 3);
    neural_input.resize(frequencies * 2 + 2);
    // neural_input.resize(2);

    double t = frame * 1.0 / dexlearn.frameCount();

    {
      size_t i = 0;

      for (size_t j = 1; j <= frequencies; j++) {
        neural_input[i++] = ValueBatch(sin(t * j * M_PI * 2));
        neural_input[i++] = ValueBatch(cos(t * j * M_PI * 2));
      }

      ScalarBatch pscale = ValueBatch(10);
      ScalarBatch px, py, pz;
      GeometryBatch::unpack(object_position, px, py, pz);
      neural_input(i++) = px * pscale;
      neural_input(i++) = py * pscale;
    }

    return neural_input;
  }

  virtual tractor::NeuralNetwork<ScalarBatch> makePolicyNetwork(
      tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn) override {

    auto &joint_names = dexlearn.jointNames();
    auto &end_effectors = dexlearn.endEffectors();

    double regularization = 0.1;

    tractor::SequentialNeuralNetwork<ScalarBatch> policy_net;

    // policy_net.add(tractor::DenseLayer<ScalarBatch>(
    //    64, tractor::Activation::TanH, 0, regularization, 0));

    // policy_net.add(tractor::DropoutLayer<ScalarBatch>(0.3));

    policy_net.add(tractor::DenseLayer<ScalarBatch>(
        joint_names.size() +
            end_effectors.size() * dexlearn.contactDimensions(),
        tractor::Activation::Linear, 0, regularization, 0));

    return policy_net;
  }

  virtual void
  init(tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn) override {
    auto &simulator = *dexlearn.simulator();

    simulator.setGravity(GeometryBatch::Vector3Zero());

    simulator.setUserConstraints(
        [](PhysicsSimulator<GeometryBatch> &simulator) {

          auto &body = simulator.body("object");

          {
            tractor::goal(body.linear_momentum * ValueBatch(100));
            body.linear_momentum = GeometryBatch::Vector3Zero();
          }

          {
            ScalarBatch x, y, z;
            GeometryBatch::unpack(body.angular_momentum, x, y, z);
            tractor::goal(x * ValueBatch(100));
            tractor::goal(y * ValueBatch(100));
            x = ValueBatch(0);
            y = ValueBatch(0);
            // z = ValueBatch(0);
            body.angular_momentum = GeometryBatch::pack(x, y, z);
          }

          body.linear_velocity = GeometryBatch::Vector3Zero();
          body.angular_velocity = GeometryBatch::Vector3Zero();

        });

    {
      double s = 0.005;
      double sz = 0.01;
      ScalarBatch px = add_random_uniform(this->makeZero(), s * -0.5, s * 0.5);
      ScalarBatch py = add_random_uniform(this->makeZero(), s * -0.5, s * 0.5);
      ScalarBatch pz =
          add_random_uniform(this->makeZero(), sz * -0.5, sz * 0.5);
      simulator.moveBody("object", GeometryBatch::pack(px, py, pz));
    }

    {
      auto rot = GeometryBatch::angleAxisOrientation(
          add_random_uniform(this->makeZero(), 0, M_PI * 2),
          GeometryBatch::import(Eigen::Vector3d(0, 0, 1)));
      simulator.rotateBody("object", rot);
    }

    {

      auto &_robot_model = dexlearn.robotModel();
      auto &_group_robot = dexlearn.robotJointGroup();
      auto &joint_names = dexlearn.jointNames();

      for (size_t i = 0; i < joint_names.size(); i++) {
        auto *joint = dexlearn.joints()[i];
        if (joint_names[i] != joint->getName()) {
          throw std::runtime_error("");
        }
        auto &bounds = joint->getVariableBounds().at(0);
        auto pos =
            ValueBatch((bounds.max_position_ + bounds.min_position_) * 0.5);
        dexlearn.simulator()->setJointPosition(joint_names[i], pos);
      }
    }
  }

  virtual void
  goals(tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn) override {

    dexlearn.addGoal(std::allocate_shared<tractor::RotationGoal<GeometryBatch>>(
        tractor::AlignedStdAlloc<tractor::RotationGoal<GeometryBatch>>(),
        "object",
        GeometryBatch::pack(ValueBatch(0.0), ValueBatch(0.0), ValueBatch(M_PI)),
        ValueBatch(3)));

    dexlearn.addGoal(
        std::allocate_shared<
            tractor::CyclicJointAccelerationRegularizer<GeometryBatch>>(
            tractor::AlignedStdAlloc<
                tractor::CyclicJointAccelerationRegularizer<GeometryBatch>>(),
            ValueBatch(5)));
  }

  virtual void
  controlRobot(tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn,
               const tractor::Tensor<ScalarBatch> &policy_output) override {

    auto &_robot_model = dexlearn.robotModel();
    auto &_group_robot = dexlearn.robotJointGroup();
    auto &joint_names = dexlearn.jointNames();

    auto positions = policy_output;

    for (size_t i = 0; i < joint_names.size(); i++) {

      positions[i] = positions[i] + ValueBatch(0.5);

      auto *joint = dexlearn.joints()[i];

      if (joint->getName() != joint_names[i]) {
        throw std::runtime_error("");
      }

      auto &bounds = joint->getVariableBounds().at(0);

      positions[i] = positions[i] * ValueBatch(bounds.max_position_ -
                                               bounds.min_position_) +
                     ValueBatch(bounds.min_position_);
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
      positions[i] = positions[j];
    }

    for (size_t i = 0; i < joint_names.size(); i++) {
      dexlearn.simulator()->setJointPosition(joint_names[i], positions[i]);
    }
  }
};

} // namespace tractor
