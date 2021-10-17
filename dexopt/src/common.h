// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/core/eigen.h>
#include <tractor/tractor.h>

#include <interactive_markers/interactive_marker_server.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>

#include <moveit_msgs/DisplayTrajectory.h>

class RobotStatePublisher {
  ros::NodeHandle node_handle;
  ros::Publisher joint_state_pub =
      node_handle.advertise<sensor_msgs::JointState>("/ik_test/joint_state",
                                                     10);
  ros::Publisher robot_state_pub =
      node_handle.advertise<moveit_msgs::DisplayRobotState>(
          "/ik_test/display_robot_state", 10);

public:
  template <class Geometry>
  void publish(const moveit::core::RobotModelConstPtr &robot_model,
               const tractor::RobotState<Geometry> &state) {
    robot_state::RobotState robot_state(robot_model);
    state.toMoveIt(robot_state);
    robot_state.update();
    {
      sensor_msgs::JointState msg;
      moveit::core::robotStateToJointStateMsg(robot_state, msg);
      joint_state_pub.publish(msg);
    }
    {
      moveit_msgs::DisplayRobotState msg;
      moveit::core::robotStateToRobotStateMsg(robot_state, msg.state);
      robot_state_pub.publish(msg);
    }
  }
};

class RobotTrajectoryPublisher {
  ros::NodeHandle node_handle;
  ros::Publisher trajectory_pub =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>(
          "/move_group/display_planned_path", 10);

public:
  template <class Geometry>
  void publish(const moveit::core::RobotModelConstPtr &robot_model,
               const std::string &group,
               const tractor::RobotTrajectory<Geometry> &trajectory) {
    robot_state::RobotState robot_state(robot_model);
    moveit_msgs::DisplayTrajectory msg;
    robot_trajectory::RobotTrajectory traj(robot_model, group);
    for (size_t i = 0; i < trajectory.size(); i++) {

      // trajectory.state(i).toMoveIt(robot_state);

      tractor::AlignedStdVector<typename Geometry::Scalar> pp;
      trajectory.state(i).joints().serializePositions(pp);
      for (size_t i = 0; i < pp.size(); i++) {
        robot_state.setVariablePosition(i, firstBatchElement(value(pp[i])));
      }

      traj.addSuffixWayPoint(robot_state, 0.1);
      if (i == 0) {
        moveit::core::robotStateToRobotStateMsg(robot_state,
                                                msg.trajectory_start);
      }
    }
    msg.trajectory.emplace_back();
    traj.getRobotTrajectoryMsg(msg.trajectory.back());
    trajectory_pub.publish(msg);
  }
};

class InteractivePoseMarker {
  struct Data {
    mutable std::mutex _mutex;
    Eigen::Affine3d _pose = Eigen::Affine3d::Identity();
    bool _flag = true;
  };
  std::shared_ptr<Data> _data = std::make_shared<Data>();
  std::string _name;
  Eigen::Affine3d _start_pose = Eigen::Affine3d::Identity();

public:
  InteractivePoseMarker(const InteractivePoseMarker &) = delete;
  InteractivePoseMarker &operator=(const InteractivePoseMarker &) = delete;

  InteractivePoseMarker(
      interactive_markers::InteractiveMarkerServer &interactive_marker_server,
      const std::string &name, const moveit::core::RobotState &robot_state,
      double scale = 0.2)
      : InteractivePoseMarker(
            interactive_marker_server,
            robot_state.getRobotModel()->getRootLink()->getName(),
            robot_state.getGlobalLinkTransform(name), name, scale) {}

  InteractivePoseMarker(
      interactive_markers::InteractiveMarkerServer &interactive_marker_server,
      const std::string &root, const Eigen::Affine3d &pose,
      const std::string &name, double scale = 0.2)
      : _name(name) {

    _data->_pose = pose;
    _start_pose = pose;

    visualization_msgs::InteractiveMarker interactive_marker;
    interactive_marker.header.frame_id = root;
    interactive_marker.header.stamp = ros::Time::now();
    interactive_marker.name = name;
    interactive_marker.scale = scale;

    robot_interaction::add6DOFControl(interactive_marker);

    tf::poseEigenToMsg(pose, interactive_marker.pose);

    std_msgs::ColorRGBA marker_color;
    marker_color.r = 1;
    marker_color.g = 1;
    marker_color.b = 0;
    marker_color.a = 1;
    robot_interaction::addViewPlaneControl(interactive_marker, 0.33 * scale,
                                           marker_color, true, false);

    auto data = _data;
    interactive_marker_server.insert(
        interactive_marker,
        [data, name](const visualization_msgs::InteractiveMarkerFeedbackConstPtr
                         &feedback) {
          if (feedback->marker_name == name &&
              feedback->event_type ==
                  visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
            // ROS_INFO_STREAM("goal pose update " << feedback->marker_name);
            Eigen::Affine3d pose;
            tf::poseMsgToEigen(feedback->pose, pose);
            std::lock_guard<std::mutex> lock(data->_mutex);
            if (!data->_pose.isApprox(pose)) {
              data->_pose = pose;
              data->_flag = true;
            }
          }
        });

    interactive_marker_server.applyChanges();
  }

  bool poll() const {
    std::lock_guard<std::mutex> lock(_data->_mutex);
    bool ret = _data->_flag;
    _data->_flag = false;
    return ret;
  }

  Eigen::Affine3d pose() const {
    std::lock_guard<std::mutex> lock(_data->_mutex);
    return _data->_pose;
  }

  const std::string &name() const { return _name; }

  const Eigen::Affine3d &initialPose() const { return _start_pose; }
};

class InteractivePositionMarker {
  struct Data {
    mutable std::mutex _mutex;
    Eigen::Vector3d _position = Eigen::Vector3d::Zero();
  };
  std::shared_ptr<Data> _data = std::make_shared<Data>();
  std::string _name;
  Eigen::Vector3d _initial_position = Eigen::Vector3d::Zero();

public:
  InteractivePositionMarker(const InteractivePoseMarker &) = delete;
  InteractivePositionMarker &operator=(const InteractivePoseMarker &) = delete;
  InteractivePositionMarker(
      interactive_markers::InteractiveMarkerServer &interactive_marker_server,
      const std::string &name, const moveit::core::RobotState &robot_state)
      : InteractivePositionMarker(
            interactive_marker_server,
            robot_state.getRobotModel()->getRootLink()->getName(),
            robot_state.getGlobalLinkTransform(name).translation(), name) {}
  InteractivePositionMarker(
      interactive_markers::InteractiveMarkerServer &interactive_marker_server,
      const std::string &root, const Eigen::Vector3d &position,
      const std::string &name)
      : _name(name) {

    _data->_position = position;
    _initial_position = position;

    visualization_msgs::InteractiveMarker interactive_marker;
    interactive_marker.header.frame_id = root;
    interactive_marker.header.stamp = ros::Time::now();
    interactive_marker.name = name;
    interactive_marker.scale = 0.2;

    robot_interaction::addPositionControl(interactive_marker);

    tf::pointEigenToMsg(position, interactive_marker.pose.position);
    interactive_marker.pose.orientation.w = 1;

    std_msgs::ColorRGBA marker_color;
    marker_color.r = 1;
    marker_color.g = 1;
    marker_color.b = 0;
    marker_color.a = 1;
    robot_interaction::addViewPlaneControl(interactive_marker, 0.066,
                                           marker_color, true, false);

    auto data = _data;
    interactive_marker_server.insert(
        interactive_marker,
        [data, name](const visualization_msgs::InteractiveMarkerFeedbackConstPtr
                         &feedback) {
          if (feedback->marker_name == name &&
              feedback->event_type ==
                  visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
            // ROS_INFO_STREAM("goal pose update " << feedback->marker_name);
            std::lock_guard<std::mutex> lock(data->_mutex);
            tf::pointMsgToEigen(feedback->pose.position, data->_position);
          }
        });

    interactive_marker_server.applyChanges();
  }

  Eigen::Vector3d position() const {
    std::lock_guard<std::mutex> lock(_data->_mutex);
    return _data->_position;
  }

  const std::string &name() const { return _name; }

  const Eigen::Vector3d &initialPosition() const { return _initial_position; }
};

namespace tractor {

template <class Geometry> class MotionGoal;

template <class Geometry> class TrajectoryOptimization {
  std::shared_ptr<tractor::RobotModel<Geometry>> _robot_model;
  tractor::RobotTrajectory<Geometry> _trajectory;
  tractor::AlignedStdVector<std::shared_ptr<MotionGoal<Geometry>>> _goals;
  // typename Geometry::Scalar _gate = typename Geometry::Scalar(1);
  std::vector<size_t> _joint_indices;
  size_t _fixed_frames = 0;

public:
  // auto &gate() const { return _gate; }
  // auto &gate() { return _gate; }
  auto &robotModel() const { return _robot_model; }
  auto &fixedFrames() const { return _fixed_frames; }
  auto &jointIndices() const { return _joint_indices; }
  auto &trajectory() const { return _trajectory; }
  auto &trajectory() { return _trajectory; }
  auto &state(size_t i) const { return _trajectory.state(i); }
  auto &state(size_t i) { return _trajectory.state(i); }
  void add(const std::shared_ptr<MotionGoal<Geometry>> &goal) {
    _goals.push_back(goal);
  }

  void
  makeTrajectory(const moveit::core::RobotModel &moveit_robot,
                 const std::string &group, size_t frames, size_t fixed,
                 const JointVariableOptions<Geometry> &joint_variable_options =
                     JointVariableOptions<Geometry>(),
                 const moveit::core::RobotState *start_state = nullptr,
                 bool make_variables = true) {

    _fixed_frames = fixed;

    _robot_model =
        std::make_shared<tractor::RobotModel<Geometry>>(moveit_robot);
    auto *joint_model_group = moveit_robot.getJointModelGroup(group);
    _trajectory.init(*_robot_model, frames);

    _joint_indices.clear();
    for (auto &joint : joint_model_group->getJointModels()) {
      _joint_indices.push_back(
          _robot_model->info()->joints().index(joint->getName()));
    }

    if (start_state) {
      for (size_t i = 0; i < _trajectory.size(); i++) {
        _trajectory.state(i).fromMoveIt(*start_state);
      }
    }

    if (make_variables) {
      for (size_t i = 0; i < _trajectory.size(); i++) {
        for (auto &joint : joint_model_group->getJointModels()) {
          if (i < fixed) {
            _trajectory.state(i)
                .joints()
                .joint(joint->getName())
                .makeParameters(_robot_model->joint(joint->getName()));
          } else {
            _trajectory.state(i)
                .joints()
                .joint(joint->getName())
                .makeVariables(_robot_model->joint(joint->getName()),
                               joint_variable_options);
          }
        }
      }
    }
  }

  void computeFK() {
    for (size_t i = 0; i < _trajectory.size(); i++) {
      _robot_model->computeFK(_trajectory.state(i).joints(),
                              _trajectory.state(i).links());
    }
  }

  void applyGoals() {
    for (auto &goal : _goals) {
      goal->apply(*this);
    }
  }

  void build(const moveit::core::RobotModel &moveit_robot,
             const std::string &group, size_t frames, size_t fixed,
             const JointVariableOptions<Geometry> &joint_variable_options =
                 JointVariableOptions<Geometry>(),
             const moveit::core::RobotState *start_state = nullptr) {
    makeTrajectory(moveit_robot, group, frames, fixed, joint_variable_options,
                   start_state);
    computeFK();
    applyGoals();
  }

  void visualize(visualization_msgs::MarkerArray &marker_array) {
    for (size_t i = 0; i < _trajectory.size(); i++) {
      _robot_model->computeFK(_trajectory.state(i).joints(),
                              _trajectory.state(i).links());
    }
    for (auto &goal : _goals) {
      size_t start_index = marker_array.markers.size();
      goal->visualize(*this, marker_array);
      for (size_t i = start_index; i < marker_array.markers.size(); i++) {
        auto &m = marker_array.markers[i];
        if (m.ns.empty()) {
          m.ns = boost::core::demangle(typeid(*goal).name());
        }
      }
    }
    size_t id_counter = 1000000000;
    for (auto &marker : marker_array.markers) {
      if (marker.header.frame_id.empty()) {
        marker.header.frame_id = "world";
      }
      if (marker.id <= 0) {
        marker.id = id_counter++;
      }
    }
  }

  void visualize() {
    static ros::NodeHandle node;
    static ros::Publisher visualization_publisher =
        node.advertise<visualization_msgs::MarkerArray>(
            "/tractor/visualization", 10, true);
    visualization_msgs::MarkerArray marker_array;
    {
      marker_array.markers.emplace_back();
      auto &m2 = marker_array.markers.back();
      m2.action = 3; // visualization_msgs::Marker::DELETEALL;
      m2.pose.orientation.w = 1;
      m2.header.frame_id = "world";
      m2.ns = "DeleteAll";
    }
    visualize(marker_array);
    visualization_publisher.publish(marker_array);
  }
};

} // namespace tractor

class LogTimer {
  const char *_label = nullptr;
  ros::WallTime _t0;

public:
  LogTimer(const char *label) : _label(label), _t0(ros::WallTime::now()) {}
  ~LogTimer() {
    ROS_INFO_STREAM("timer " << _label << " "
                             << (ros::WallTime::now() - _t0).toSec());
  }
};
