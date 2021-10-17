// (c) 2020-2021 Philipp Ruppel

#pragma once

//#define EIGEN_NO_STATIC_ASSERT

#include <tractor/collision/link.h>
#include <tractor/core/profiler.h>

namespace tractor {

namespace internal {

class CollisionSupportInterface {
public:
  virtual void support(double dx, double dy, double dz, double &px, double &py,
                       double &pz) const = 0;
  virtual void center(double &px, double &py, double &pz) const = 0;
};
struct CollisionResult {
  double ax = 0, ay = 0, az = 0;
  double bx = 0, by = 0, bz = 0;
  double nx = 1, ny = 0, nz = 0;
  double d = 0;
};
void doCollisionQuery(const CollisionSupportInterface &a,
                      const CollisionSupportInterface &b,
                      CollisionResult &result);

template <class Scalar> struct BarrierSolver {
  template <class Matrix, class Input, class Output>
  static void solve(const Matrix &matrix, const Input &input, Output &output) {
    throw std::runtime_error("nyi");
  }
};
template <> struct BarrierSolver<double> {
  template <class Matrix, class Input, class Output>
  static void solve(const Matrix &matrix, const Input &input, Output &output) {
    output = matrix.colPivHouseholderQr().solve(input);
  }
};

template <class Scalar>
class CollisionShapeSupport
    : public tractor::internal::CollisionSupportInterface {
  Pose<Scalar> _pose;
  const CollisionShape<Scalar> *_shape = nullptr;

public:
  CollisionShapeSupport(const Pose<Scalar> &pose,
                        const CollisionShape<Scalar> *shape)
      : _pose(pose), _shape(shape) {}
  virtual void support(double dx, double dy, double dz, double &px, double &py,
                       double &pz) const override {
    if (!_shape) {
      throw std::runtime_error("shape is null");
    }
    Vector3<Scalar> d = Vector3<Scalar>(Scalar(dx), Scalar(dy), Scalar(dz));
    d = _pose.orientation().inverse() * d;
    Vector3<Scalar> p;
    _shape->support(d, p);
    p = _pose * p;
    px = double(p.x());
    py = double(p.y());
    pz = double(p.z());
  }
  virtual void center(double &px, double &py, double &pz) const override {
    auto p = _shape->center();
    p = _pose * p;
    px = p.x();
    py = p.y();
    pz = p.z();
  }
};

template <class Scalar>
class PointSupport : public tractor::internal::CollisionSupportInterface {
  Vector3<Scalar> _point;

public:
  PointSupport(const Vector3<Scalar> &point) : _point(point) {}
  virtual void support(double dx, double dy, double dz, double &px, double &py,
                       double &pz) const override {

    px = double(_point.x());
    py = double(_point.y());
    pz = double(_point.z());

    // double r = 1e-3;
    // px = double(_point.x() + dx * r);
    // py = double(_point.y() + dy * r);
    // pz = double(_point.z() + dz * r);

    /*
    auto p = _point + normalized(Vector3<Scalar>(dx, dy, dz)) * Scalar(1e-3);
    px = double(p.x());
    py = double(p.y());
    pz = double(p.z());
    */
  }
  virtual void center(double &px, double &py, double &pz) const override {
    px = double(_point.x());
    py = double(_point.y());
    pz = double(_point.z());
  }
};

} // namespace internal

template <class Scalar>
void CollisionShape<Scalar>::project(const Vector3<Scalar> &point,
                                     Vector3<Scalar> &normal,
                                     Scalar &distance) {
  auto a = tractor::internal::CollisionShapeSupport<Scalar>(
      Pose<Scalar>::Identity(), this);
  auto b = tractor::internal::PointSupport<Scalar>(point);
  tractor::internal::CollisionResult r;
  tractor::internal::doCollisionQuery(a, b, r);
  normal = Vector3<Scalar>(Scalar(r.nx), Scalar(r.ny), Scalar(r.nz));
  distance = r.d;
}

template <class Scalar> class ShapeCollisionPair {
  bool _initialized = false;
  std::shared_ptr<const CollisionShape<Scalar>> _shape_a, _shape_b;
  Vector3<Scalar> _point_a, _point_b, _normal;
  Scalar _distance;
  Plane<Scalar> _barrier;
  tractor::internal::CollisionResult _collision_result;

public:
  ShapeCollisionPair() {}
  ShapeCollisionPair(
      const std::shared_ptr<const CollisionShape<Scalar>> &shape_a,
      const std::shared_ptr<const CollisionShape<Scalar>> &shape_b)
      : _shape_a(shape_a), _shape_b(shape_b), _initialized(true) {}
  bool initialized() const { return _initialized; }
  auto &shapeA() const { return _shape_a; }
  auto &shapeB() const { return _shape_b; }
  void update(const Pose<Scalar> &pose_a, const Pose<Scalar> &pose_b) {
    auto a = tractor::internal::CollisionShapeSupport<Scalar>(pose_a,
                                                              _shape_a.get());
    auto b = tractor::internal::CollisionShapeSupport<Scalar>(pose_b,
                                                              _shape_b.get());
    auto &r = _collision_result;
    tractor::internal::doCollisionQuery(a, b, r);
    _point_a = Vector3<Scalar>(Scalar(r.ax), Scalar(r.ay), Scalar(r.az));
    _point_b = Vector3<Scalar>(Scalar(r.bx), Scalar(r.by), Scalar(r.bz));
    _normal =
        normalized(Vector3<Scalar>(Scalar(r.nx), Scalar(r.ny), Scalar(r.nz)));
    _distance = Scalar(r.d);
  }
  inline auto &pointA() const { return _point_a; }
  inline auto &pointB() const { return _point_b; }
  inline auto &normal() const { return _normal; }
  inline auto &pointA() { return _point_a; }
  inline auto &pointB() { return _point_b; }
  inline auto &normal() { return _normal; }
  inline auto &distance() const { return _distance; }
  inline auto plane() const {
    Plane<Scalar> plane;
    plane.normal() = _normal;
    plane.offset() = -dot(_normal, (_point_a + _point_b) * Scalar(0.5));
    return plane;
  }
  inline auto center() const { return (_point_a + _point_b) * Scalar(0.5); }
  void optimize(Pose<Scalar> pose_a, Pose<Scalar> pose_b) {

    TRACTOR_PROFILER("collision barrier");

    pose_a.translation() -= center();
    pose_b.translation() -= center();

    _barrier = plane();

    _barrier = Plane<Scalar>(_barrier.normal(), _barrier.point() - center());

    double regularization = 10;

    Eigen::Matrix<Scalar, 6, 1> previous_gradient =
        Eigen::Matrix<Scalar, 6, 1>::Zero();

    for (size_t j = 0; j < 100; j++) {

      // std::cout << "step " << j << " " << regularization << std::endl;

      // static size_t n = 1;
      // n = (n + 1) % 20;
      // std::cout << n << std::endl;

      Twist<Scalar> twist = Twist<Scalar>::Zero();

      {

        Eigen::Matrix<Scalar, 6, 1> gradient_a;
        Eigen::Matrix<Scalar, 6, 6> hessian_a;
        _shape_a->barrier(_barrier, twist, pose_a, gradient_a, hessian_a);

        Eigen::Matrix<Scalar, 6, 1> gradient_b;
        Eigen::Matrix<Scalar, 6, 6> hessian_b;
        _shape_b->barrier(-_barrier, twist, pose_b, gradient_b, hessian_b);

        Eigen::Matrix<Scalar, 6, 1> gradient = gradient_a + gradient_b;
        Eigen::Matrix<Scalar, 6, 6> hessian = hessian_a + hessian_b;

        if (gradient.dot(previous_gradient) > 0) {
          regularization *= 0.5;
        } else {
          regularization *= 4.0;
        }
        previous_gradient = gradient;

        hessian.diagonal() +=
            Eigen::Matrix<Scalar, 6, 1>::Constant(Scalar(regularization));

        // Eigen::Matrix<Scalar, 6, 1> result_vector =
        //    hessian.colPivHouseholderQr().solve(gradient);

        Eigen::Matrix<Scalar, 6, 1> result_vector;
        tractor::internal::BarrierSolver<Scalar>::solve(hessian, gradient,
                                                        result_vector);

        // std::cout << result_vector.norm() << std::endl;

        if (!(result_vector.norm() > 1e-6)) {
          break;
        }

        twist.translation().x() -= result_vector[0];
        twist.translation().y() -= result_vector[1];
        twist.translation().z() -= result_vector[2];
        twist.rotation().x() -= result_vector[3];
        twist.rotation().y() -= result_vector[4];
        twist.rotation().z() -= result_vector[5];
      }

      twist = twist * Scalar(-1);

      Scalar f = 1;

      while (f > 1e-2) {

        auto barrier =
            Plane<Scalar>(normalized(twist * f * _barrier.normal() -
                                     twist * f * Vector3<Scalar>::Zero()),
                          twist * f * _barrier.point());

        Eigen::Matrix<Scalar, 6, 1> gradient_a;
        Eigen::Matrix<Scalar, 6, 6> hessian_a;
        _shape_a->barrier(barrier, Twist<Scalar>::Zero(), pose_a, gradient_a,
                          hessian_a);

        Eigen::Matrix<Scalar, 6, 1> gradient_b;
        Eigen::Matrix<Scalar, 6, 6> hessian_b;
        _shape_b->barrier(-barrier, Twist<Scalar>::Zero(), pose_b, gradient_b,
                          hessian_b);

        f *= 0.5;

        if (gradient_a.allFinite() && gradient_b.allFinite()) {

          /*
          if (f < 0.4) {
            regularization *= 2;
          } else {
            regularization *= 0.5;
          }
          */

          _barrier =
              Plane<Scalar>(normalized(twist * f * _barrier.normal() -
                                       twist * f * Vector3<Scalar>::Zero()),
                            twist * f * _barrier.point());

          break;
        }
      }
    }

    _barrier = Plane<Scalar>(_barrier.normal(), _barrier.point() + center());

    if (!std::isfinite(_barrier.normal().x()) ||
        !std::isfinite(_barrier.normal().y()) ||
        !std::isfinite(_barrier.normal().z()) ||
        !std::isfinite(_barrier.offset())) {
      throw std::runtime_error("barrier not finite");
    }
  }
  auto &barrier() const { return _barrier; }
  auto &barrier() { return _barrier; }
};

template <class Scalar> class LinkCollisionPair {
  std::shared_ptr<const CollisionLink<Scalar>> _link_a, _link_b;
  AlignedStdVector<ShapeCollisionPair<Scalar>> _elements;

public:
  LinkCollisionPair() {}
  LinkCollisionPair(const std::shared_ptr<const CollisionLink<Scalar>> &link_a,
                    const std::shared_ptr<const CollisionLink<Scalar>> &link_b)
      : _link_a(link_a), _link_b(link_b) {
    for (auto &shape_a : link_a->shapes()) {
      for (auto &shape_b : link_b->shapes()) {
        _elements.emplace_back(shape_a, shape_b);
      }
    }
  }
  void update(const Pose<Scalar> &pose_a, const Pose<Scalar> &pose_b) {
    for (auto &element : _elements) {
      element.update(pose_a, pose_b);
    }
  }
  void optimize(const Pose<Scalar> &pose_a, const Pose<Scalar> &pose_b) {
    for (auto &element : _elements) {
      element.optimize(pose_a, pose_b);
    }
  }
  inline auto &elements() const { return _elements; }
  inline auto &elements() { return _elements; }
  inline auto &linkA() const { return _link_a; }
  inline auto &linkB() const { return _link_b; }
};

} // namespace tractor
