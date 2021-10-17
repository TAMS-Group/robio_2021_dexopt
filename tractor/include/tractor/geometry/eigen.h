// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/core/eigen.h>

namespace tractor {

template <class ScalarType> struct GeometryEigenMat {

  typedef ScalarType Scalar;
  typedef Eigen::Matrix<Scalar, 3, 1, Eigen::DontAlign> Vector3;
  typedef Eigen::Quaternion<Scalar, Eigen::DontAlign> Orientation;
  typedef Eigen::Transform<Scalar, 3, Eigen::Isometry, Eigen::DontAlign> Pose;

  static Pose angleAxisPose(const Scalar &angle, const Vector3 &axis) {
    return Pose(Eigen::AngleAxis<Scalar>(angle, axis));
  }

  static Pose angleAxisPose(const Pose &parent, const Scalar &angle,
                            const Vector3 &axis) {
    return parent * Pose(Eigen::AngleAxis<Scalar>(angle, axis));
  }

  static Pose pose(const Scalar &px, const Scalar &py, const Scalar &pz,
                   const Scalar &qx, const Scalar &qy, const Scalar &qz,
                   const Scalar &qw) {

    Orientation quat;
    quat.x() = qx;
    quat.y() = qy;
    quat.z() = qz;
    quat.w() = qw;

    Pose pose;
    pose.translation().x() = px;
    pose.translation().y() = py;
    pose.translation().z() = pz;
    pose.linear() = quat.toRotationMatrix();

    return pose;
  }

  static void unpack(const Pose &pose, Scalar &px, Scalar &py, Scalar &pz,
                     Scalar &qx, Scalar &qy, Scalar &qz, Scalar &qw) {

    Vector3 pos = pose.translation();
    px = pos.x();
    py = pos.y();
    pz = pos.z();

    Orientation quat(pose.linear());
    qx = quat.x();
    qy = quat.y();
    qz = quat.z();
    qw = quat.w();
  }

  static Pose identityPose() { Pose::Identity(); }

  static Pose translationPose(const Vector3 &pos) {
    Pose pose;
    pose.setIdentity();
    pose.translation() = pos;
    return pose;
  }

  static Pose translationPose(const Pose &parent, const Vector3 &pos) {
    return parent * translationPose(pos);
  }

  static Pose translationPose(const Pose &parent, const Scalar &x,
                              const Scalar &y, const Scalar &z) {
    return parent * translationPose(Vector3(x, y, z));
  }

  static Vector3 translation(const Pose &pose) { return pose.translation(); }

  static Orientation orientation(const Pose &pose) {
    // return Orientation(pose.linear());
    // return Orientation::Identity();
    auto &mat = pose.linear();
    Orientation ret;
    ret.w() = sqrt(Scalar(1) + mat(0, 0) + mat(1, 1) + mat(2, 2)) * Scalar(0.5);
    ret.x() = (mat(2, 1) - mat(1, 2)) / (Scalar(4) * ret.w());
    ret.y() = (mat(0, 2) - mat(2, 0)) / (Scalar(4) * ret.w());
    ret.z() = (mat(1, 0) - mat(0, 1)) / (Scalar(4) * ret.w());
    return ret;
  }

  static Orientation inverse(const Orientation &q) {
    return Orientation(q.w(), -q.x(), -q.y(), -q.z());
  }

  static Vector3 residual(const Orientation &a, const Orientation &b) {
    // return (a.inverse() * b).vec();
    return (inverse(a) * b).vec();
    // return Vector3::Zero();
  }

  template <class T, int Mode, int Flags>
  static Pose import(const Eigen::Transform<T, 3, Mode, Flags> &p) {
    // Eigen::Transform<typename P::Scalar, 3, Eigen::Isometry,
    // Eigen::DontAlign>(
    //    p.matrix());
    // return Pose(p.matrix().template cast<Scalar>());
    /*return Eigen::Transform<typename P::Scalar, 3, Eigen::Isometry,
                            Eigen::DontAlign>(p.matrix())
        .template cast<Scalar>();*/

    /*
    Pose ret;
    ret.matrix() = p.matrix().template cast<Scalar>();
    return ret;
    */

    // return Pose(p.matrix().template cast<Scalar>());

    /*
    Pose ret;
    for (size_t row = 0; row < 3; row++) {
      for (size_t col = 0; col < 4; col++) {
        ret.matrix()(row, col) = Scalar(p.matrix(row, col));
      }
    }
    ret(3, 0) = Scalar(0);
    ret(3, 1) = Scalar(0);
    ret(3, 2) = Scalar(0);
    ret(3, 3) = Scalar(1);
    return ret;
    */

    Pose t = translationPose(
        Vector3(p.translation().x(), p.translation().y(), p.translation().z()));

    Eigen::AngleAxisd angle_axis(p.linear());

    Pose orientation =
        angleAxisPose(angle_axis.angle(),
                      Vector3(angle_axis.axis().x(), angle_axis.axis().y(),
                              angle_axis.axis().z()));

    return t * orientation;
  }

  template <class T, int Flags>
  static Vector3 import(const Eigen::Matrix<T, 3, 1, Flags> &p) {
    return Vector3(Scalar(p.x()), Scalar(p.y()), Scalar(p.z()));
  }
};

template <class Scalar>
void parameter(
    Eigen::Transform<Scalar, 3, Eigen::Isometry, Eigen::DontAlign> &pose) {
  for (size_t row = 0; row < pose.rows(); row++) {
    for (size_t col = 0; col < pose.cols(); col++) {
      parameter(pose.matrix()(row, col));
    }
  }
}

template <class ScalarType> class PoseEigenQuat {

public:
  typedef ScalarType Scalar;
  typedef Eigen::Matrix<Scalar, 3, 1, Eigen::DontAlign> Vector3;
  typedef Eigen::Quaternion<Scalar, Eigen::DontAlign> Orientation;

  auto &translation() const { return _translation; }
  auto &translation() { return _translation; }
  auto &orientation() const { return _orientation; }
  auto &orientation() { return _orientation; }
  PoseEigenQuat operator*(const PoseEigenQuat &other) const {
    PoseEigenQuat ret;
    ret.translation() = translation() + orientation() * other.translation();
    ret.orientation() = orientation() * other.orientation();
    return ret;
  }
  static PoseEigenQuat Identity() {
    PoseEigenQuat ret;
    ret.translation().setZero();
    ret.orientation().setIdentity();
    return ret;
  }

private:
  Vector3 _translation;
  Orientation _orientation;
};

template <class ScalarType> struct GeometryEigenQuat {

  typedef ScalarType Scalar;
  typedef PoseEigenQuat<ScalarType> Pose;
  typedef typename Pose::Vector3 Vector3;
  typedef typename Pose::Orientation Orientation;

  static Pose angleAxisPose(const Scalar &angle, const Vector3 &axis) {
    Pose pose;
    pose.translation().setZero();
    Scalar s = sin(angle * Scalar(0.5));
    Scalar c = cos(angle * Scalar(0.5));
    pose.orientation().x() = axis.x() * s;
    pose.orientation().y() = axis.y() * s;
    pose.orientation().z() = axis.z() * s;
    pose.orientation().w() = c;
    return pose;
  }

  static void unpack(const Pose &pose, Scalar &px, Scalar &py, Scalar &pz,
                     Scalar &qx, Scalar &qy, Scalar &qz, Scalar &qw) {

    Vector3 pos = pose.translation();
    px = pos.x();
    py = pos.y();
    pz = pos.z();

    Orientation quat = pose.orientation();
    qx = quat.x();
    qy = quat.y();
    qz = quat.z();
    qw = quat.w();
  }

  static Pose angleAxisPose(const Pose &parent, const Scalar &angle,
                            const Vector3 &axis) {
    return parent * angleAxisPose(angle, axis);
  }

  static Pose pose(const Scalar &px, const Scalar &py, const Scalar &pz,
                   const Scalar &qx, const Scalar &qy, const Scalar &qz,
                   const Scalar &qw) {
    Pose pose;
    pose.translation().x() = px;
    pose.translation().y() = py;
    pose.translation().z() = pz;
    pose.orientation().x() = qx;
    pose.orientation().y() = qy;
    pose.orientation().z() = qz;
    pose.orientation().w() = qw;
    return pose;
  }

  static Pose identityPose() {
    Pose pose;
    pose.translation().setZero();
    pose.orientation().setIdentity();
    return pose;
  }

  static Pose translationPose(const Vector3 &pos) {
    Pose pose;
    pose.translation() = pos;
    pose.orientation().setIdentity();
    return pose;
  }

  static Pose translationPose(const Pose &parent, const Vector3 &pos) {
    return parent * translationPose(pos);
  }

  static Pose translationPose(const Pose &parent, const Scalar &x,
                              const Scalar &y, const Scalar &z) {
    return parent * translationPose(Vector3(x, y, z));
  }

  static const Vector3 &translation(const Pose &pose) {
    return pose.translation();
  }

  static const Orientation &orientation(const Pose &pose) {
    return pose.orientation();
  }

  static Orientation inverse(const Orientation &q) {
    return Orientation(q.w(), -q.x(), -q.y(), -q.z());
  }

  static Vector3 residual(const Orientation &a, const Orientation &b) {
    return (inverse(a) * b).vec();
  }

  template <class T, int Mode, int Flags>
  static Pose import(const Eigen::Transform<T, 3, Mode, Flags> &pose) {
    /*Pose ret;
    ret.orientation().w() = sqrt(Scalar(1) + pose.linear()(0, 0) +
                                 pose.linear()(1, 1) + pose.linear()(2, 2)) *
                            Scalar(0.5);
    ret.orientation().x() = (pose.linear()(2, 1) - pose.linear()(1, 2)) /
                            (Scalar(4) * ret.orientation().w());
    ret.orientation().y() = (pose.linear()(0, 2) - pose.linear()(2, 0)) /
                            (Scalar(4) * ret.orientation().w());
    ret.orientation().z() = (pose.linear()(1, 0) - pose.linear()(0, 1)) /
                            (Scalar(4) * ret.orientation().w());
    ret.translation() = pose.translation().template cast<Scalar>();
    return ret;*/

    Pose t =
        translationPose(Vector3(pose.translation().x(), pose.translation().y(),
                                pose.translation().z()));

    Eigen::AngleAxisd angle_axis(pose.linear());

    Pose orientation =
        angleAxisPose(angle_axis.angle(),
                      Vector3(angle_axis.axis().x(), angle_axis.axis().y(),
                              angle_axis.axis().z()));

    return t * orientation;
  }

  template <class T, int Flags>
  static Vector3 import(const Eigen::Matrix<T, 3, 1, Flags> &p) {
    return Vector3(Scalar(p.x()), Scalar(p.y()), Scalar(p.z()));
  }
};

template <class Scalar>
void parameter(Eigen::Matrix<Scalar, 3, 1, Eigen::DontAlign> &v) {
  parameter(v.x());
  parameter(v.y());
  parameter(v.z());
}

template <class Scalar>
void goal(const Eigen::Matrix<Scalar, 3, 1, Eigen::DontAlign> &v) {
  goal(v.x());
  goal(v.y());
  goal(v.z());
}

template <class Scalar> void parameter(PoseEigenQuat<Scalar> &pose) {
  parameter(pose.translation().x());
  parameter(pose.translation().y());
  parameter(pose.translation().z());
  parameter(pose.orientation().x());
  parameter(pose.orientation().y());
  parameter(pose.orientation().z());
  parameter(pose.orientation().w());
}

} // namespace tractor
