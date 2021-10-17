// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "common.h"

#include <tractor/tractor.h>

#include <geometric_shapes/mesh_operations.h>
#include <moveit/collision_detection/collision_matrix.h>

#include "ops.h"

namespace tractor {

template <class Geometry> class MotionGoal {

public:
  MotionGoal() {}
  MotionGoal(const MotionGoal &) = delete;
  MotionGoal &operator=(const MotionGoal &) = delete;
  virtual ~MotionGoal() {}
  virtual void apply(TrajectoryOptimization<Geometry> &trajectory_opt) {}
  virtual void visualize(const TrajectoryOptimization<Geometry> &trajectory_opt,
                         visualization_msgs::MarkerArray &marker_array) {}
};

template <class Geometry> struct RotationGoal : public MotionGoal<Geometry> {
  std::string link;
  typename Geometry::Vector3 rotation;
  typename Geometry::Scalar weight;
  RotationGoal(const std::string &link,
               const typename Geometry::Vector3 &rotation,
               const typename Geometry::Scalar &weight = 1)
      : link(link), rotation(rotation), weight(weight) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    auto rot = Geometry::import(Eigen::Vector3d(0, 0, 0));
    for (size_t i = 1; i < trajectory.size(); i++) {
      auto &pa = trajectory.state(i - 1).links().pose(link);
      auto &pb = trajectory.state(i).links().pose(link);
      rot += Geometry::residual(Geometry::orientation(pa),
                                Geometry::orientation(pb));
    }
    goal((rot - rotation) * weight);
  }
};

template <class Geometry> struct RotationGoal2 : public MotionGoal<Geometry> {
  std::string link;
  typename Geometry::Vector3 rotation;
  typename Geometry::Scalar weight;
  RotationGoal2(const std::string &link,
                const typename Geometry::Vector3 &rotation,
                const typename Geometry::Scalar &weight = 1)
      : link(link), rotation(rotation), weight(weight) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    for (size_t i = 1; i < trajectory.size(); i++) {
      auto &pa = trajectory.state(i - 1).links().pose(link);
      auto &pb = trajectory.state(i).links().pose(link);
      goal((rotation - Geometry::residual(Geometry::orientation(pa),
                                          Geometry::orientation(pb))) *
           weight);
    }
  }
};

template <class Geometry>
struct RelativeOrientationGoal : public MotionGoal<Geometry> {
  std::string link;
  typename Geometry::Vector3 rotation;
  typename Geometry::Scalar weight;
  RelativeOrientationGoal(const std::string &link,
                          const typename Geometry::Vector3 &rotation,
                          const typename Geometry::Scalar &weight = 1)
      : link(link), rotation(rotation), weight(weight) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    auto &pa = trajectory.state(0).links().pose(link);
    auto &pb = trajectory.state(trajectory.size() - 1).links().pose(link);
    goal((rotation - Geometry::residual(Geometry::orientation(pa),
                                        Geometry::orientation(pb))) *
         weight);
  }
};

template <class Geometry>
struct RotationRegularizer : public MotionGoal<Geometry> {
  std::string link;
  typename Geometry::Scalar weight;
  RotationRegularizer(const std::string &link,
                      const typename Geometry::Scalar &weight = 1)
      : link(link), weight(weight) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    for (size_t i = 1; i < trajectory.size(); i++) {
      auto &pa = trajectory.state(i - 1).links().pose(link);
      auto &pb = trajectory.state(i).links().pose(link);
      goal(Geometry::residual(Geometry::orientation(pa),
                              Geometry::orientation(pb)) *
           weight);
    }
  }
};

template <class Geometry> struct PositionGoal : public MotionGoal<Geometry> {
  std::string link;
  size_t frame = 0;
  typename Geometry::Vector3 position;
  typename Geometry::Scalar weight;
  PositionGoal(
      const std::string &link, size_t frame,
      const typename Geometry::Vector3 &position = typename Geometry::Vector3(),
      typename Geometry::Scalar weight = 1)
      : link(link), frame(frame), position(position), weight(weight) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    goal((Geometry::translation(trajectory.state(frame).links().pose(link)) -
          position) *
         weight);
  }
  virtual void
  visualize(const TrajectoryOptimization<Geometry> &trajectory_opt,
            visualization_msgs::MarkerArray &marker_array) override {
    auto &trajectory = trajectory_opt.trajectory();
    double visualization_marker_size = 0.1;
    typename Geometry::Scalar x, y, z;
    Geometry::unpack(position, x, y, z);
    marker_array.markers.emplace_back();
    visualization_msgs::Marker &marker = marker_array.markers.back();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = visualization_marker_size;
    marker.scale.y = visualization_marker_size;
    marker.scale.z = visualization_marker_size;
    marker.pose.position.x = firstBatchElement(value(x));
    marker.pose.position.y = firstBatchElement(value(y));
    marker.pose.position.z = firstBatchElement(value(z));
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
  }
};

template <class Geometry> struct AxisGoal : public MotionGoal<Geometry> {
  std::string link;
  size_t frame = 0;
  typename Geometry::Vector3 axis;
  typename Geometry::Vector3 direction;
  typename Geometry::Scalar weight;
  AxisGoal(const std::string &link, size_t frame,
           const typename Geometry::Vector3 &axis,
           const typename Geometry::Vector3 &direction,
           typename Geometry::Scalar weight = 1)
      : link(link), frame(frame), axis(axis), direction(direction),
        weight(weight) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    goal((Geometry::orientation(trajectory.state(frame).links().pose(link)) *
              axis -
          direction) *
         weight);
  }
};

template <class Geometry> struct MoveGoal : public MotionGoal<Geometry> {
  std::string link;
  size_t frame = 0;
  typename Geometry::Vector3 position;
  typename Geometry::Scalar weight;
  MoveGoal(
      const std::string &link, size_t frame,
      const typename Geometry::Vector3 &position = typename Geometry::Vector3(),
      typename Geometry::Scalar weight = 1)
      : link(link), frame(frame), position(position), weight(weight) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    goal((Geometry::translation(trajectory.state(frame).links().pose(link)) -
          Geometry::translation(trajectory.state(0).links().pose(link)) -
          position) *
         weight);
  }
};

template <class Geometry> struct OrientationGoal : public MotionGoal<Geometry> {
  std::string link;
  size_t frame = 0;
  typename Geometry::Orientation orientation;
  typename Geometry::Scalar weight;
  OrientationGoal(const std::string &link, size_t frame,
                  const typename Geometry::Orientation &orientation,
                  typename Geometry::Scalar weight = 1)
      : link(link), frame(frame), orientation(orientation), weight(weight) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    goal(Geometry::residual(orientation,
                            Geometry::orientation(
                                trajectory.state(frame).links().pose(link))) *
         weight);
  }
};

template <class Geometry> struct PoseGoal : public MotionGoal<Geometry> {
  std::string link;
  size_t frame = 0;
  typename Geometry::Pose pose;
  PoseGoal(const std::string &link, size_t frame,
           const typename Geometry::Pose &pose)
      : link(link), frame(frame), pose(pose) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    goal(Geometry::residual(pose, trajectory.state(frame).links().pose(link)));
  }
};

template <class Geometry>
struct RelativePoseGoal : public MotionGoal<Geometry> {
  std::string link;
  size_t frame = 0;
  typename Geometry::Pose pose;
  typename Geometry::Scalar weight;
  RelativePoseGoal(
      const std::string &link, size_t frame,
      const typename Geometry::Pose &pose,
      const typename Geometry::Scalar &weight = typename Geometry::Value(1))
      : link(link), frame(frame), pose(pose), weight(weight) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    goal(Geometry::residual(trajectory.state(0).links().pose(link) * pose,
                            trajectory.state(frame).links().pose(link)) *
         weight);
  }
};

template <class Geometry> struct StartSlowGoal : public MotionGoal<Geometry> {
  typename Geometry::Scalar weight;
  StartSlowGoal(const typename Geometry::Scalar &weight = 1) : weight(weight) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    auto &sa = trajectory.state(1).joints();
    auto &sb = trajectory.state(2).joints();
    for (size_t j : trajectory_opt.jointIndices()) {
      auto *ja = dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
          &sa.joint(j));
      auto *jb = dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
          &sb.joint(j));
      if (ja) {
        goal((ja->position() - jb->position()) * weight);
      }
    }
  }
};

template <class Geometry> struct StopConstraint : public MotionGoal<Geometry> {
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    auto &sa = trajectory.state(trajectory.size() - 2).joints();
    auto &sb = trajectory.state(trajectory.size() - 1).joints();
    for (size_t j : trajectory_opt.jointIndices()) {
      auto *ja = dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
          &sa.joint(j));
      auto *jb = dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
          &sb.joint(j));
      if (ja) {
        goal(ja->position() - jb->position(), 1);
      }
    }
  }
};

template <class Geometry>
struct JointTestConstraint : public MotionGoal<Geometry> {
  JointTestConstraint() {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    for (size_t i = trajectory_opt.fixedFrames() + 1; i < trajectory.size();
         i++) {
      auto &sa = trajectory.state(i - 1).joints();
      auto &sb = trajectory.state(i).joints();
      for (size_t j : trajectory_opt.jointIndices()) {
        auto *ja =
            dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                &sa.joint(j));
        auto *jb =
            dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                &sb.joint(j));
        if (ja) {
          goal(ja->position() - jb->position() + 0.05, 1);
        }
      }
    }
  }
  virtual void
  visualize(const TrajectoryOptimization<Geometry> &trajectory_opt,
            visualization_msgs::MarkerArray &marker_array) override {
    auto &trajectory = trajectory_opt.trajectory();
  }
};

template <class Geometry>
struct JointVelocityConstraint : public MotionGoal<Geometry> {
  typename Geometry::Value velocity_limit = 1.0;
  std::deque<typename Geometry::Scalar> slack_variables;
  JointVelocityConstraint(const typename Geometry::Value &velocity_limit)
      : velocity_limit(velocity_limit) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    slack_variables.clear();
    for (size_t i = 1; i < trajectory.size(); i++) {
      auto &sa = trajectory.state(i - 1).joints();
      auto &sb = trajectory.state(i).joints();
      for (size_t j : trajectory_opt.jointIndices()) {
        auto *ja =
            dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                &sa.joint(j));
        auto *jb =
            dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                &sb.joint(j));
        if (ja) {
          slack_variables.emplace_back();
          slackVariable(slack_variables.back(), -velocity_limit,
                        +velocity_limit);
          goal(ja->position() - jb->position() + slack_variables.back(), 1);
        }
      }
    }
  }
  virtual void
  visualize(const TrajectoryOptimization<Geometry> &trajectory_opt,
            visualization_msgs::MarkerArray &marker_array) override {
    auto &trajectory = trajectory_opt.trajectory();
    // ROS_INFO_STREAM("velocity_limit " << velocity_limit);
    for (auto &v : slack_variables) {
      if (abs(firstBatchElement(value(v))) > velocity_limit) {
        ROS_ERROR_STREAM("v " << firstBatchElement(value(v)));
      } else {
        // ROS_INFO_STREAM("v " << value(v));
      }
    }
  }
};

template <class Geometry>
struct JointAccelerationConstraint : public MotionGoal<Geometry> {
  typename Geometry::Value acceleration_limit = 1.0;
  std::deque<typename Geometry::Scalar> slack_variables;
  std::unordered_set<std::string> joint_names;
  JointAccelerationConstraint(
      const typename Geometry::Value &acceleration_limit,
      const std::vector<std::string> &joint_names = {})
      : acceleration_limit(acceleration_limit),
        joint_names(joint_names.begin(), joint_names.end()) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    slack_variables.clear();
    for (size_t i = 2; i < trajectory.size(); i++) {
      auto &sa = trajectory.state(i - 2).joints();
      auto &sb = trajectory.state(i - 1).joints();
      auto &sc = trajectory.state(i).joints();
      for (size_t j : trajectory_opt.jointIndices()) {

        if (!joint_names.empty() &&
            (joint_names.find(
                 trajectory_opt.robotModel()->info()->joints().name(j)) ==
             joint_names.end())) {
          continue;
        }

        auto *ja =
            dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                &sa.joint(j));
        auto *jb =
            dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                &sb.joint(j));
        auto *jc =
            dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                &sc.joint(j));
        if (ja) {

          slack_variables.emplace_back();
          auto *var = slackVariable(slack_variables.back(), -acceleration_limit,
                                    +acceleration_limit);
          var->name() += " joint acceleration";
          goal(jb->position() + jb->position() - ja->position() -
                   jc->position() + slack_variables.back(),
               1);
        }
      }
    }
  }
  virtual void
  visualize(const TrajectoryOptimization<Geometry> &trajectory_opt,
            visualization_msgs::MarkerArray &marker_array) override {
    auto &trajectory = trajectory_opt.trajectory();
    // ROS_INFO_STREAM("acceleration " << acceleration_limit);
    for (auto &v : slack_variables) {
      if (abs(value(v)) > acceleration_limit) {
        ROS_ERROR_STREAM("a " << value(v));
      } else {
        // ROS_INFO_STREAM("a " << value(v));
      }
    }
  }
};

template <class Geometry>
struct JointVelocityRegularizer : public MotionGoal<Geometry> {
  typename Geometry::Scalar weight = 1.0;
  JointVelocityRegularizer(const typename Geometry::Scalar &weight)
      : weight(weight) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    for (size_t i = 1; i < trajectory.size(); i++) {
      auto &sa = trajectory.state(i - 1).joints();
      auto &sb = trajectory.state(i).joints();
      for (size_t j : trajectory_opt.jointIndices()) {
        auto *ja =
            dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                &sa.joint(j));
        auto *jb =
            dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                &sb.joint(j));
        if (ja) {
          goal((ja->position() - jb->position()) * weight);
        }
      }
    }
  }
};

template <class Geometry>
struct JointAccelerationRegularizer : public MotionGoal<Geometry> {

  typename Geometry::Scalar weight = 1.0;
  std::unordered_set<std::string> joint_names;

  JointAccelerationRegularizer(const typename Geometry::Scalar &weight)
      : weight(weight) {}

  JointAccelerationRegularizer(const typename Geometry::Scalar &weight,
                               const std::vector<std::string> &joint_names)
      : weight(weight), joint_names(joint_names.begin(), joint_names.end()) {}

  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();

    for (size_t i = 2; i < trajectory.size(); i++) {
      auto &sa = trajectory.state(i - 2).joints();
      auto &sb = trajectory.state(i - 1).joints();
      auto &sc = trajectory.state(i).joints();

      for (size_t j : trajectory_opt.jointIndices()) {

        if (!joint_names.empty() &&
            (joint_names.find(
                 trajectory_opt.robotModel()->info()->joints().name(j)) ==
             joint_names.end())) {
          continue;
        }

        auto *ja =
            dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                &sa.joint(j));
        auto *jb =
            dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                &sb.joint(j));
        auto *jc =
            dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                &sc.joint(j));

        if (ja) {
          goal((jb->position() + jb->position() - ja->position() -
                jc->position()) *
               weight);
        }
      }
    }
  }
};

template <class Geometry>
struct CyclicJointAccelerationRegularizer : public MotionGoal<Geometry> {

  typename Geometry::Scalar weight = 1.0;
  std::unordered_set<std::string> joint_names;

  CyclicJointAccelerationRegularizer(const typename Geometry::Scalar &weight)
      : weight(weight) {}

  CyclicJointAccelerationRegularizer(
      const typename Geometry::Scalar &weight,
      const std::vector<std::string> &joint_names)
      : weight(weight), joint_names(joint_names.begin(), joint_names.end()) {}

  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();

    for (size_t i = 0; i < trajectory.size(); i++) {
      auto &sa = trajectory.state((i + 0) % trajectory.size()).joints();
      auto &sb = trajectory.state((i + 1) % trajectory.size()).joints();
      auto &sc = trajectory.state((i + 2) % trajectory.size()).joints();

      for (size_t j : trajectory_opt.jointIndices()) {

        if (!joint_names.empty() &&
            (joint_names.find(
                 trajectory_opt.robotModel()->info()->joints().name(j)) ==
             joint_names.end())) {
          continue;
        }

        auto *ja =
            dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                &sa.joint(j));
        auto *jb =
            dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                &sb.joint(j));
        auto *jc =
            dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                &sc.joint(j));

        if (ja) {
          goal((jb->position() + jb->position() - ja->position() -
                jc->position()) *
               weight);
        }
      }
    }
  }
};

template <class Geometry>
struct CenterJointsGoal : public MotionGoal<Geometry> {
  typename Geometry::Scalar weight = 1.0;
  std::unordered_set<std::string> joint_names;
  CenterJointsGoal(const typename Geometry::Scalar &weight,
                   const std::vector<std::string> &joint_names = {})
      : weight(weight), joint_names(joint_names.begin(), joint_names.end()) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    for (size_t i = trajectory_opt.fixedFrames(); i < trajectory.size(); i++) {
      auto &sa = trajectory.state(i).joints();
      for (size_t j : trajectory_opt.jointIndices()) {
        if (!joint_names.empty() &&
            (joint_names.find(
                 trajectory_opt.robotModel()->info()->joints().name(j)) ==
             joint_names.end())) {
          continue;
        }
        if (auto *ja =
                dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                    &sa.joint(j))) {
          goal(ja->position() * weight);
        }
      }
    }
  }
};

template <class Geometry>
struct JointLimitPenalty : public MotionGoal<Geometry> {
  typename Geometry::Scalar weight = 1.0;
  std::unordered_set<std::string> joint_names;
  JointLimitPenalty(const typename Geometry::Scalar &weight,
                    const std::vector<std::string> &joint_names = {})
      : weight(weight), joint_names(joint_names.begin(), joint_names.end()) {}
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();
    for (size_t i = trajectory_opt.fixedFrames(); i < trajectory.size(); i++) {
      auto &sa = trajectory.state(i).joints();
      for (size_t j : trajectory_opt.jointIndices()) {
        if (!joint_names.empty() &&
            (joint_names.find(
                 trajectory_opt.robotModel()->info()->joints().name(j)) ==
             joint_names.end())) {
          continue;
        }
        if (auto *joint_state =
                dynamic_cast<const tractor::ScalarJointStateBase<Geometry> *>(
                    &sa.joint(j))) {
          if (auto *joint_model =
                  dynamic_cast<const tractor::ScalarJointModelBase<Geometry> *>(
                      &trajectory_opt.robotModel()->joint(j))) {
            goal(relu(joint_model->limits().lower() - joint_state->position()));
            goal(relu(joint_state->position() - joint_model->limits().upper()));
          }
        }
      }
    }
  }
};

} // namespace tractor
