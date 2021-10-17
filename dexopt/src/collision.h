// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "goals.h"
#include "ops.h"

namespace tractor {

template <class Geometry> class TouchGoal : public MotionGoal<Geometry> {
  size_t _frame_index = 0;
  std::string _link_a, _link_b;
  tractor::CollisionRobot<typename Geometry::Value> _collision_robot;
  typename Geometry::Vector3 _slack_a, _slack_b;

public:
  TouchGoal(size_t frame, const moveit::core::RobotModel &robot_model,
            const std::string &link_a, const std::string &link_b)
      : _frame_index(frame), _collision_robot(robot_model), _link_a(link_a),
        _link_b(link_b) {}
  virtual void apply(TrajectoryOptimization<Geometry> &trajectory_opt) {

    auto pose_a =
        trajectory_opt.trajectory().state(_frame_index).links().pose(_link_a);
    auto pose_b =
        trajectory_opt.trajectory().state(_frame_index).links().pose(_link_b);

    auto shape_a = _collision_robot.link(_link_a)->shapes().front();
    auto shape_b = _collision_robot.link(_link_b)->shapes().front();

    _slack_a.value() = shape_a->center();
    _slack_b.value() = shape_b->center();
    tractor::slackVariable(_slack_a);
    tractor::slackVariable(_slack_b);

    tractor::goal(pose_a * _slack_a - pose_b * _slack_b);

    tractor::goal(capture_constraint(_slack_a, (uintptr_t)shape_a.get()));
    tractor::goal(capture_constraint(_slack_b, (uintptr_t)shape_b.get()));
  }
  virtual void visualize(const TrajectoryOptimization<Geometry> &trajectory_opt,
                         visualization_msgs::MarkerArray &marker_array) {
    auto &trajectory = trajectory_opt.trajectory();

    auto &pose_a = value(trajectory.state(_frame_index).links().pose(_link_a));
    auto &pose_b = value(trajectory.state(_frame_index).links().pose(_link_b));

    std::cout << "touch " << pose_a * value(_slack_a) << " "
              << pose_b * value(_slack_b) << std::endl;

    auto addPoint = [&](Vector3<typename Geometry::Value> p) {
      auto &marker = marker_array.markers.back();
      marker.points.emplace_back();
      marker.points.back().x = p.x();
      marker.points.back().y = p.y();
      marker.points.back().z = p.z();
    };

    {
      marker_array.markers.emplace_back();
      auto &marker = marker_array.markers.back();
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.scale.x = 0.02;
      marker.color.r = 1;
      marker.color.g = 1;
      marker.color.b = 0;
      marker.color.a = 1;
      addPoint(pose_a * value(_slack_a));
      addPoint(pose_b * value(_slack_b));
    }
  }
};

template <class Geometry>
class CollisionPairConstraint : public MotionGoal<Geometry> {
  size_t _frame_index = 0;
  tractor::CollisionRobot<typename Geometry::Value> _collision_robot;
  tractor::LinkCollisionPair<typename Geometry::Value> _link_pair;
  // typename Geometry::Vector3 _slack;
  typename Geometry::Pose _slack;
  uint64_t link_pair_ptr = uintptr_t(&_link_pair);

public:
  CollisionPairConstraint(size_t frame,
                          const moveit::core::RobotModel &robot_model,
                          const std::string &a, const std::string &b)
      : _frame_index(frame), _collision_robot(robot_model),
        _link_pair(_collision_robot.link(a), _collision_robot.link(b)) {}
  virtual void apply(TrajectoryOptimization<Geometry> &trajectory_opt) {

    auto pose_a = trajectory_opt.trajectory()
                      .state(_frame_index)
                      .links()
                      .pose(_link_pair.linkA()->name());
    auto pose_b = trajectory_opt.trajectory()
                      .state(_frame_index)
                      .links()
                      .pose(_link_pair.linkB()->name());

    _slack.value() = pose_a.value().inverse() * pose_b.value();
    tractor::slackVariable(_slack);

    tractor::goal(Geometry::residual(pose_a * _slack, pose_b), 1);

    tractor::goal(collision_constraint(_slack, link_pair_ptr));
  }
  virtual void visualize(const TrajectoryOptimization<Geometry> &trajectory_opt,
                         visualization_msgs::MarkerArray &marker_array) {
    auto &trajectory = trajectory_opt.trajectory();

    auto &pose_a = value(trajectory.state(_frame_index)
                             .links()
                             .pose(_link_pair.linkA()->name()));
    auto &pose_b = value(trajectory.state(_frame_index)
                             .links()
                             .pose(_link_pair.linkB()->name()));

    std::cout << "pose_a " << pose_a << std::endl;
    std::cout << "pose_b " << pose_b << std::endl;

    for (auto &shape_pair : _link_pair.elements()) {
      std::cout << "distance " << shape_pair.distance() << std::endl;
    }

    auto addPoint = [&](Vector3<typename Geometry::Value> p) {
      p = pose_a * p;
      auto &marker = marker_array.markers.back();
      marker.points.emplace_back();
      marker.points.back().x = p.x();
      marker.points.back().y = p.y();
      marker.points.back().z = p.z();
    };

    {
      marker_array.markers.emplace_back();
      auto &marker = marker_array.markers.back();
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.scale.x = 0.01;
      marker.color.r = 1;
      marker.color.b = 1;
      marker.color.a = 1;
      for (auto &shape_pair : _link_pair.elements()) {
        addPoint(shape_pair.pointA());
        addPoint(shape_pair.pointB());
      }
    }

    {
      marker_array.markers.emplace_back();
      auto &marker = marker_array.markers.back();
      marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 1;
      marker.color.r = 1;
      marker.color.g = 1;
      marker.color.b = 0;
      marker.color.a = 0.5;
      typename Geometry::Value size = 0.25;
      for (auto &shape_pair : _link_pair.elements()) {
        auto plane = shape_pair.barrier();
        ROS_INFO_STREAM("plane " << plane);
        auto normal = plane.normal();
        auto point = plane.point();
        auto tangent1 = normalized(
            cross(normal, Vector3<typename Geometry::Value>(1, 2, 3)));
        auto tangent2 = normalized(cross(normal, tangent1));
        auto c = (pose_a.inverse() * pose_b).translation() *
                 typename Geometry::Value(0.5);
        point += tangent1 * dot(tangent1, c - point);
        point += tangent2 * dot(tangent2, c - point);
        addPoint(point - tangent1 * size - tangent2 * size);
        addPoint(point + tangent1 * size - tangent2 * size);
        addPoint(point + tangent1 * size + tangent2 * size);
        addPoint(point - tangent1 * size - tangent2 * size);
        addPoint(point + tangent1 * size + tangent2 * size);
        addPoint(point - tangent1 * size + tangent2 * size);
      }
    }
  }
};

template <class Geometry>
class CollisionShapeVisualizer : public MotionGoal<Geometry> {
  tractor::CollisionRobot<typename Geometry::Value> _collision_robot;

public:
  CollisionShapeVisualizer(const moveit::core::RobotModel &robot_model)
      : _collision_robot(robot_model) {}
  virtual void apply(TrajectoryOptimization<Geometry> &trajectory_opt) {}
  virtual void visualize(const TrajectoryOptimization<Geometry> &trajectory_opt,
                         visualization_msgs::MarkerArray &marker_array) {
    auto &trajectory = trajectory_opt.trajectory();
    marker_array.markers.emplace_back();
    auto &marker = marker_array.markers.back();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.02;
    random_numbers::RandomNumberGenerator rng(0);
    for (auto &link : _collision_robot.links()) {
      double r = rng.uniform01();
      double g = rng.uniform01();
      double b = rng.uniform01();
      auto pose = value(trajectory.state(0).links().pose(link->name()));
      for (auto &shape : link->shapes()) {
        {
          auto p = pose * shape->center();
          marker.points.emplace_back();
          marker.points.back().x = p.x();
          marker.points.back().y = p.y();
          marker.points.back().z = p.z();
          marker.colors.emplace_back();
          marker.colors.back().r = r;
          marker.colors.back().g = g;
          marker.colors.back().b = b;
          marker.colors.back().a = 1;
        }
        if (auto *polyhedron =
                dynamic_cast<const ConvexPolyhedralCollisionShape<
                    typename Geometry::Value> *>(shape.get())) {
          for (auto &point : polyhedron->points()) {
            auto p = pose * point;
            marker.points.emplace_back();
            marker.points.back().x = p.x();
            marker.points.back().y = p.y();
            marker.points.back().z = p.z();
            marker.colors.emplace_back();
            marker.colors.back().r = r;
            marker.colors.back().g = g;
            marker.colors.back().b = b;
            marker.colors.back().a = 1;
          }
        }
      }
    }
  }
};

template <class Geometry>
class SphereCollisionConstraint : public MotionGoal<Geometry> {
  size_t _frame_index = 0;
  std::string _link_a;
  std::string _link_b;
  typename Geometry::Vector3 _slack;
  typename Geometry::Scalar _distance;

public:
  SphereCollisionConstraint(size_t frame_index, const std::string &link_a,
                            const std::string &link_b,
                            const typename Geometry::Scalar &distance)
      : _frame_index(frame_index), _link_a(link_a), _link_b(link_b),
        _distance(distance) {}
  virtual void apply(TrajectoryOptimization<Geometry> &trajectory_opt) {
    typename Geometry::Vector3 diff =
        Geometry::translation(trajectory_opt.trajectory()
                                  .state(_frame_index)
                                  .links()
                                  .pose(_link_a)) -
        Geometry::translation(trajectory_opt.trajectory()
                                  .state(_frame_index)
                                  .links()
                                  .pose(_link_b));
    _slack = diff;
    tractor::slackVariable(_slack);
    tractor::goal(diff - _slack, 1);
    tractor::goal(sphere_collision_constraint(_slack, _distance));
  }
  virtual void visualize(const TrajectoryOptimization<Geometry> &trajectory_opt,
                         visualization_msgs::MarkerArray &marker_array) {
    auto &trajectory = trajectory_opt.trajectory();
    ROS_INFO_STREAM(_frame_index
                    << " " << _link_a << " " << _link_b << " "
                    << _distance.value() << " " << _slack.value().x() << " "
                    << _slack.value().y() << " " << _slack.value().z());
  }
};

template <class Geometry>
class CollisionConstraint : public MotionGoal<Geometry> {

protected:
  struct CollisionShape {
    std::vector<typename Geometry::Vector3> points;
    typename Geometry::Scalar padding = typename Geometry::Scalar(0);
    CollisionShape(shapes::ShapeConstPtr shape, const Eigen::Affine3d &origin) {
      if (shape->type == shapes::SPHERE) {
        ROS_INFO_STREAM("collision shape sphere");
        points.push_back(Geometry::import(origin.translation().eval()));
        // Eigen::Matrix3d rotation;
        // Eigen::Vector3d scaling;
        // origin.computeRotationScaling(&rotation, &scaling);
        auto &sphere = dynamic_cast<const shapes::Sphere &>(*shape);
        // padding = sphere.radius * scaling.x() * 2;
        padding = sphere.radius;
        return;
      }
      if (shape->type != shapes::MESH) {
        shape.reset(shapes::createMeshFromShape(shape.get()));
      }
      ROS_INFO_STREAM("collision shape mesh");
      auto &mesh = dynamic_cast<const shapes::Mesh &>(*shape);
      for (size_t i = 0; i < mesh.vertex_count; i++) {
        Eigen::Vector3d pos =
            origin * Eigen::Vector3d(mesh.vertices[i * 3 + 0],
                                     mesh.vertices[i * 3 + 1],
                                     mesh.vertices[i * 3 + 2]);
        ROS_INFO_STREAM("p " << pos);
        points.push_back(Geometry::import(pos));
      }
    }
  };
  struct CollisionLink {
    std::deque<CollisionShape> shapes;
    CollisionLink(const moveit::core::LinkModel *link) {
      ROS_INFO_STREAM("collision link " << link->getName());
      auto &shapes = link->getShapes();
      auto &origins = link->getCollisionOriginTransforms();
      for (size_t shape_index = 0; shape_index < shapes.size(); shape_index++) {
        auto &shape = shapes[shape_index];
        auto &origin = origins[shape_index];
        this->shapes.emplace_back(shape, origin);
      }
    }
  };
  std::vector<std::shared_ptr<CollisionLink>> _links;
  std::vector<std::pair<size_t, size_t>> _pairs;
  std::deque<typename Geometry::Scalar> _slack_variables;
  moveit::core::RobotState _start_state;
  std::vector<Eigen::Vector3d> _plane_normals;

public:
  CollisionConstraint(const planning_scene::PlanningScene &planning_scene,
                      const collision_detection::AllowedCollisionMatrix &acm,
                      const moveit::core::RobotState &robot_state)
      : _start_state(robot_state) {

    auto robot_model = planning_scene.getRobotModel();
    _links.resize(robot_model->getLinkModelCount());
    for (size_t i = 0; i < robot_model->getLinkModelCount(); i++) {
      for (size_t j = i + 1; j < robot_model->getLinkModelCount(); j++) {
        collision_detection::AllowedCollision::Type t;
        if (acm.getEntry(robot_model->getLinkModel(i)->getName(),
                         robot_model->getLinkModel(j)->getName(), t) &&
            t != collision_detection::AllowedCollision::ALWAYS) {
          ROS_INFO_STREAM("check collisions "
                          << robot_model->getLinkModel(i)->getName() << " "
                          << robot_model->getLinkModel(j)->getName());
          _pairs.emplace_back(i, j);
          for (size_t k : {i, j}) {
            auto *link = robot_model->getLinkModel(k);
            if (!_links[k]) {
              _links[k] = std::make_shared<CollisionLink>(link);
            }
          }
        }
      }
    }
  }
  virtual void
  apply(TrajectoryOptimization<Geometry> &trajectory_opt) override {
    auto &trajectory = trajectory_opt.trajectory();

    typename Geometry::Scalar weight = 1.0;

    for (size_t iframe = trajectory_opt.fixedFrames();
         iframe < trajectory.size(); iframe++) {
      for (auto &pair : _pairs) {
        for (auto &shape_a : _links[pair.first]->shapes) {
          for (auto &shape_b : _links[pair.second]->shapes) {

            ROS_INFO_STREAM("c apply "
                            << iframe << " " << pair.first << " " << pair.second
                            << " " << &shape_a << " " << shape_a.points.size()
                            << " " << &shape_b << " " << shape_b.points.size());

            /*_slack_variables.emplace_back();
            auto &plane_position = _slack_variables.back();
            slackVariable(plane_position);*/

            Eigen::Vector3d pa =
                _start_state
                    .getGlobalLinkTransform(
                        _start_state.getRobotModel()->getLinkModel(pair.first))
                    .translation();
            Eigen::Vector3d pb =
                _start_state
                    .getGlobalLinkTransform(
                        _start_state.getRobotModel()->getLinkModel(pair.second))
                    .translation();

            Eigen::Vector3d plane_normal_1 = (pa - pb).normalized().eval();
            _plane_normals.push_back(plane_normal_1);
            auto plane_normal = Geometry::import(plane_normal_1);

            typename Geometry::Scalar plane_position =
                plane_normal_1.dot((pa + pb) * 0.5);

            auto &pose_a = trajectory.state(iframe).links().pose(pair.first);
            auto &pose_b = trajectory.state(iframe).links().pose(pair.second);

            for (auto &p : shape_a.points) {
              _slack_variables.emplace_back();
              auto &slack = _slack_variables.back();
              slackVariable(slack, 0.0, 10.0);
              goal((dot(plane_normal, pose_a * p) - plane_position - slack) *
                       weight,
                   1);
            }

            for (auto &p : shape_b.points) {
              _slack_variables.emplace_back();
              auto &slack = _slack_variables.back();
              slackVariable(slack, 0.0, 10.0);
              goal((dot(plane_normal, pose_b * p) - plane_position + slack) *
                       weight,
                   1);
            }

            /*for (auto &p : shape_a.points) {
              _slack_variables.emplace_back();
              auto &slack = _slack_variables.back();
              slackVariable(slack, 0.0, 10.0);
              goal((plane_position + dot(plane_normal, pose_a * p) - slack) *
                       weight,
                   1);
            }

            for (auto &p : shape_b.points) {
              _slack_variables.emplace_back();
              auto &slack = _slack_variables.back();
              slackVariable(slack, 0.0, 10.0);
              goal((plane_position + dot(plane_normal, pose_b * p) + slack) *
                       weight,
                   1);
            }*/
          }
        }
      }
    }
  }
  virtual void
  visualize(const TrajectoryOptimization<Geometry> &trajectory_opt,
            visualization_msgs::MarkerArray &marker_array) override {
    auto &trajectory = trajectory_opt.trajectory();
    ROS_INFO_STREAM("vis");
    for (auto &p : _slack_variables) {
      ROS_INFO_STREAM("s " << value(p));
    }
    for (auto &n : _plane_normals) {
      ROS_INFO_STREAM("n " << n.x() << " " << n.y() << " " << n.z());
    }
    for (size_t ilink = 0; ilink < _links.size(); ilink++) {
      if (auto &link = _links[ilink]) {
        for (auto &shape : link->shapes) {
          marker_array.markers.emplace_back();
          visualization_msgs::Marker &marker = marker_array.markers.back();
          marker.type = visualization_msgs::Marker::POINTS;
          marker.scale.x = 0.01 + value(shape.padding);
          marker.color.r = 0;
          marker.color.g = 1;
          marker.color.b = 0;
          marker.color.a = 1;
          for (size_t iframe = 0; iframe < trajectory.size(); iframe++) {
            auto &state = trajectory.state(iframe);
            auto &pose = state.links().pose(ilink);
            for (auto p : shape.points) {
              p = pose * p;
              typename Geometry::Scalar x, y, z;
              Geometry::unpack(p, x, y, z);
              marker.points.emplace_back();
              marker.points.back().x = value(x);
              marker.points.back().y = value(y);
              marker.points.back().z = value(z);
            }
          }
        }
      }
    }
  }
};

} // namespace tractor
