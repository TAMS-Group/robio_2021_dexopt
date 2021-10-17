// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/core/eigen.h>
#include <tractor/core/profiler.h>
#include <tractor/geometry/types.h>

#include <stdexcept>
#include <vector>

namespace tractor {

template <class Scalar> class CollisionShape {
protected:
  Vector3<Scalar> _center = Vector3<Scalar>::Zero();

  static inline void _capturePlane(const Vector3<Scalar> &point,
                                   const Plane<Scalar> &plane,
                                   Eigen::Matrix<Scalar, 3, 1> &gradient,
                                   Eigen::Matrix<Scalar, 3, 3> &hessian) {
    Vector3<Scalar> gradient_vector =
        plane.normal() *
        (Scalar(1) / std::max(Scalar(0), -plane.signedDistance(point)));
    gradient += Eigen::Matrix<Scalar, 3, 1>(
        gradient_vector.x(), gradient_vector.y(), gradient_vector.z());
    hessian += gradient * gradient.transpose();
  }

public:
  inline auto &center() const { return _center; }
  virtual void support(const Vector3<Scalar> &direction,
                       Vector3<Scalar> &point) const = 0;
  virtual Vector3<Scalar> support(const Vector3<Scalar> &direction) const {
    Vector3<Scalar> point;
    support(direction, point);
    return point;
  }
  virtual void barrier(const Plane<Scalar> &plane, const Twist<Scalar> &twist,
                       const Pose<Scalar> &pose,
                       Eigen::Matrix<Scalar, 6, 1> &gradient,
                       Eigen::Matrix<Scalar, 6, 6> &hessian) const = 0;
  virtual void capture(const Vector3<Scalar> &point,
                       Eigen::Matrix<Scalar, 3, 1> &gradient,
                       Eigen::Matrix<Scalar, 3, 3> &hessian) const = 0;
  virtual void project(const Vector3<Scalar> &point, Vector3<Scalar> &normal,
                       Scalar &distance);
};

template <class Scalar>
class CollisionSphereShape : public CollisionShape<Scalar> {
  Scalar _radius = 0;

public:
  CollisionSphereShape(const Vector3<Scalar> &center, const Scalar &radius) {
    this->_center = center;
    _radius = radius;
  }

  virtual void support(const Vector3<Scalar> &direction,
                       Vector3<Scalar> &point) const override {
    point = this->_center + normalized(direction) * _radius;
  }

  virtual void
  barrier(const Plane<Scalar> &plane, const Twist<Scalar> &twist,
          const Pose<Scalar> &pose, Eigen::Matrix<Scalar, 6, 1> &out_gradient,
          Eigen::Matrix<Scalar, 6, 6> &out_hessian) const override {

    TRACTOR_PROFILER("sphere barrier");

    out_gradient.setZero();
    out_hessian.setZero();

    auto &point_local = this->_center;

    Vector3<Scalar> position_global = twist * (pose * point_local);

    Scalar gradient_magnitude =
        Scalar(-1) /
        std::max(Scalar(0), plane.signedDistance(position_global) - _radius);

    Vector3<Scalar> gradient_vector = plane.normal() * -gradient_magnitude;

    Twist<Scalar> gradient_twist = Twist<Scalar>::Zero();
    gradient_twist.translation() = gradient_vector;
    gradient_twist.rotation() = cross(gradient_vector, position_global);

    Eigen::Matrix<Scalar, 6, 1> gradient_twist_vector;
    gradient_twist_vector[0] = -gradient_twist.translation().x();
    gradient_twist_vector[1] = -gradient_twist.translation().y();
    gradient_twist_vector[2] = -gradient_twist.translation().z();
    gradient_twist_vector[3] = gradient_twist.rotation().x();
    gradient_twist_vector[4] = gradient_twist.rotation().y();
    gradient_twist_vector[5] = gradient_twist.rotation().z();

    out_gradient += gradient_twist_vector;
    out_hessian += gradient_twist_vector * gradient_twist_vector.transpose();
  }

  virtual void capture(const Vector3<Scalar> &point,
                       Eigen::Matrix<Scalar, 3, 1> &gradient,
                       Eigen::Matrix<Scalar, 3, 3> &hessian) const override {

    TRACTOR_PROFILER("sphere capture");

    gradient.setZero();
    hessian.setZero();

    static std::vector<Vector3<Scalar>> directions = []() {
      std::vector<Vector3<Scalar>> directions;
      for (int x = -1; x <= 1; x++) {
        for (int y = -1; y <= 1; y++) {
          for (int z = -1; z <= 1; z++) {
            if (x == 0 && y == 0 && z == 0) {
              continue;
            }
            directions.push_back(normalized(Vector3<Scalar>(x, y, z)));
          }
        }
      }
      return directions;
    }();
    for (auto &dir : directions) {
      CollisionShape<Scalar>::_capturePlane(point, Plane<Scalar>(dir, -_radius),
                                            gradient, hessian);
    }
    {
      Scalar f = Scalar(1) / Scalar(Scalar(directions.size() + 8));
      gradient.array() *= f;
      hessian.array() *= f;
    }
  }
};

template <class Scalar>
class CollisionCylinderShape : public CollisionShape<Scalar> {
  Scalar _radius = 0;
  Scalar _length = 0;

  static inline Scalar _sign(const Scalar &v) {
    if (v < 0) {
      return Scalar(-1);
    }
    if (v > 0) {
      return Scalar(1);
    }
    return Scalar(0);
  }

public:
  CollisionCylinderShape(const Scalar &radius, const Scalar &length) {
    _radius = radius;
    _length = length;
  }

  auto &radius() const { return _radius; }
  auto &length() const { return _length; }

  virtual void support(const Vector3<Scalar> &direction,

                       Vector3<Scalar> &point) const override {
    Vector3<Scalar> axis = Vector3<Scalar>(Scalar(0), Scalar(0), Scalar(1));
    Vector3<Scalar> a = direction - dot(axis, direction) * axis;
    if (norm(a) != Scalar(0)) {
      point = Scalar(_sign(direction.z())) * (_length * Scalar(0.5)) * axis +
              _radius * normalized(a);
    } else {
      point = Scalar(_sign(direction.z())) * (_length * Scalar(0.5)) * axis;
    }
  }

  virtual void
  barrier(const Plane<Scalar> &plane, const Twist<Scalar> &twist,
          const Pose<Scalar> &pose, Eigen::Matrix<Scalar, 6, 1> &out_gradient,
          Eigen::Matrix<Scalar, 6, 6> &out_hessian) const override {
    throw std::runtime_error("nyi");
  }

  virtual void capture(const Vector3<Scalar> &point,
                       Eigen::Matrix<Scalar, 3, 1> &gradient,
                       Eigen::Matrix<Scalar, 3, 3> &hessian) const override {
    throw std::runtime_error("nyi");
  }

  // virtual void project(const Vector3<Scalar> &point, Vector3<Scalar> &normal,
  //                      Scalar &distance) override {
  //   throw std::runtime_error("nyi");
  // }
};

template <class Scalar>
class ConvexPolyhedralCollisionShape : public CollisionShape<Scalar> {
  std::vector<Vector3<Scalar>> _points;
  std::vector<Plane<Scalar>> _planes;

public:
  ConvexPolyhedralCollisionShape(const std::vector<Vector3<Scalar>> &points,
                                 const std::vector<Plane<Scalar>> &planes)
      : _points(points), _planes(planes) {
    if (points.empty()) {
      throw std::invalid_argument("points");
    }
    if (planes.empty()) {
      throw std::invalid_argument("planes");
    }
    this->_center = Vector3<Scalar>::Zero();
    for (auto &p : points) {
      this->_center += p;
    }
    this->_center *= Scalar(1.0 / points.size());
  }

  virtual void support(const Vector3<Scalar> &direction,
                       Vector3<Scalar> &point) const override {
    point = _points.front();
    for (auto &p : _points) {
      if (dot(p, direction) > dot(point, direction)) {
        point = p;
      }
    }
  }

  virtual void
  barrier(const Plane<Scalar> &plane, const Twist<Scalar> &twist,
          const Pose<Scalar> &pose, Eigen::Matrix<Scalar, 6, 1> &out_gradient,
          Eigen::Matrix<Scalar, 6, 6> &out_hessian) const override {

    TRACTOR_PROFILER("mesh barrier");

    out_gradient.setZero();
    out_hessian.setZero();

    Eigen::Matrix<Scalar, 3, 3> twist_rotation_e;
    twist_rotation_e << Scalar(1), -twist.rotation().z(),
        twist.rotation().y(),                                   //
        twist.rotation().z(), Scalar(1), -twist.rotation().x(), //
        -twist.rotation().y(), twist.rotation().x(), Scalar(1);

    Eigen::Matrix<Scalar, 3, 3> pose_rotation_e =
        Eigen::Matrix<Scalar, 3, 3>(Eigen::Quaternion<Scalar>(
            pose.orientation().w(), pose.orientation().x(),
            pose.orientation().y(), pose.orientation().z()));

    Matrix3<Scalar> rotation;
    Eigen::Matrix<Scalar, 3, 3> rotation_e = twist_rotation_e * pose_rotation_e;
    for (size_t row = 0; row < 3; row++) {
      for (size_t col = 0; col < 3; col++) {
        rotation(row, col) = rotation_e(row, col);
      }
    }

    Vector3<Scalar> translation = twist * pose.translation();

    for (size_t row = 0; row < _points.size(); row++) {

      auto &position_local = _points[row];

      Vector3<Scalar> position_global_2 =
          translation + rotation * position_local;

#if 0
      Vector3<Scalar> position_global_1 = twist * (pose * position_local);
      for (size_t i = 0; i < 3; i++) {
        if (std::abs(position_global_1[i] - position_global_2[i]) > 1e-6) {
          std::cout << norm(pose.orientation()) << std::endl;
          std::cout << position_global_1 << " " << position_global_2
                    << std::endl;
          throw 0;
        }
      }
#endif

      auto &position_global = position_global_2;

      Scalar scalar_gradient =
          Scalar(-1) /
          std::max(Scalar(0), plane.signedDistance(position_global));

      Vector3<Scalar> translation_gradient = plane.normal() * -scalar_gradient;
      Vector3<Scalar> rotation_gradient =
          cross(translation_gradient, position_global);

      Eigen::Matrix<Scalar, 6, 1> gradient_twist_vector;
      gradient_twist_vector[0] = -translation_gradient.x();
      gradient_twist_vector[1] = -translation_gradient.y();
      gradient_twist_vector[2] = -translation_gradient.z();
      gradient_twist_vector[3] = rotation_gradient.x();
      gradient_twist_vector[4] = rotation_gradient.y();
      gradient_twist_vector[5] = rotation_gradient.z();

      out_gradient += gradient_twist_vector;
      out_hessian += gradient_twist_vector * gradient_twist_vector.transpose();
    }

    {
      Scalar f = Scalar(1) / Scalar(Scalar(_points.size() + 8));
      out_gradient.array() *= f;
      out_hessian.array() *= f;
    }

    // std::cout << "h " << out_hessian << std::endl;
    // std::cout << "t " << (out_gradient * out_gradient.transpose()) <<
    // std::endl;
  }

  virtual void capture(const Vector3<Scalar> &point,
                       Eigen::Matrix<Scalar, 3, 1> &gradient,
                       Eigen::Matrix<Scalar, 3, 3> &hessian) const override {
    TRACTOR_PROFILER("mesh capture");
    gradient.setZero();
    hessian.setZero();
    for (auto &plane : _planes) {
      CollisionShape<Scalar>::_capturePlane(point, plane, gradient, hessian);
    }
    {
      Scalar f = Scalar(1) / Scalar(Scalar(_planes.size() + 8));
      gradient.array() *= f;
      hessian.array() *= f;
    }
  }

  const std::vector<Vector3<Scalar>> &points() const { return _points; }

  auto &planes() const { return _planes; }

  virtual void project(const Vector3<Scalar> &point, Vector3<Scalar> &normal,
                       Scalar &distance) override {
    bool inside = true;
    for (size_t i = 0; i < _planes.size(); i++) {
      auto &plane = _planes[i];
      auto dist = plane.signedDistance(point);
      if (dist > 0) {
        inside = false;
      }
      if (i == 0 || dist > distance) {
        distance = dist;
        normal = -plane.normal();
      }
    }
    if (!inside) {
      CollisionShape<Scalar>::project(point, normal, distance);
      // normal = Vector3<Scalar>(0, 0, 1);
      // distance = 1;
    }
  }
};

} // namespace tractor
