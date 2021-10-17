// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <interactive_markers/interactive_marker_server.h>
#include <kdl/frames.hpp>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <mutex>
#include <pluginlib/class_loader.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>

#include <tractor/tractor.h>

#include "common.h"
#include "dexviz.h"
#include "goals.h"
#include "neural.h"
#include "physics5.h"

#define TRACTOR_GOAL_STRINGIFY(n) #n
// #define TRACTOR_GOAL(var) tractor::goal(var, 0, TRACTOR_GOAL_STRINGIFY(var))

#define TRACTOR_GOAL(var) tractor::goal(var)

namespace tractor {

template <class ValueSingle, class ValueBatch> class DexLearn;

template <class ValueSingle, class ValueBatch> struct DexEnv {

  typedef tractor::Var<ValueSingle> ScalarSingle;
  typedef tractor::GeometryFast<ScalarSingle> GeometrySingle;

  typedef tractor::Var<ValueBatch> ScalarBatch;
  typedef tractor::GeometryFast<ScalarBatch> GeometryBatch;

  static ScalarBatch makeZero() {
    static std::deque<ScalarBatch, tractor::AlignedStdAlloc<ScalarBatch>>
        _zero_buffer;
    if (_zero_buffer.empty()) {
      _zero_buffer.emplace_back(ValueBatch(0));
      tractor::parameter(_zero_buffer.back());
    }
    return _zero_buffer.back();
  };

  struct Info {
    std::string name;
    size_t frame_count = 20;
    double collision_avoidance_distance = 0;
    double collision_avoidance_weight = 0;
    double friction_cone_penalty = 0;
    double contact_distance_penalty = 0;
    double contact_slip_penalty = 0;
    double joint_limit_penalty = 0.5;
    double shape_penalty = 20;
    double contact_point_regularization = 1;
    double contact_force_regularization = 0.2;
    double slip_avoidance_distance = 0;
    double slip_avoidance_weight = 0;
    double collision_penalty = 1;
    std::vector<std::string> end_effectors;
  };

protected:
  Info _info;

public:
  const Info &info() const { return _info; }

  virtual void init(tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn) = 0;

  virtual void goals(tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn) = 0;

  virtual tractor::Tensor<ScalarBatch>
  makePolicyInput(tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn,
                  size_t frame) = 0;

  virtual tractor::NeuralNetwork<ScalarBatch>
  makePolicyNetwork(tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn) = 0;

  virtual void
  controlRobot(tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn,
               const tractor::Tensor<ScalarBatch> &policy_output) = 0;
};

} // namespace tractor
