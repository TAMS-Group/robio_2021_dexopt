// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/core/batch.h>
#include <tractor/geometry/quaternion.h>
#include <tractor/geometry/twist.h>
#include <tractor/geometry/vector3.h>

namespace tractor {

template <class Scalar> class Pose {

private:
  Vector3<Scalar> _translation;
  Quaternion<Scalar> _orientation;

public:
  inline Pose() {}
  inline Pose(const Vector3<Scalar> &translation,
              const Quaternion<Scalar> &orientation)
      : _translation(translation), _orientation(orientation) {}
  inline auto &translation() const { return _translation; }
  inline auto &translation() { return _translation; }
  inline auto &orientation() const { return _orientation; }
  inline auto &orientation() { return _orientation; }
  inline static auto Identity() {
    Pose ret;
    ret.translation() = Vector3<Scalar>::Zero();
    ret.orientation() = Quaternion<Scalar>::Identity();
    return ret;
  }
  inline void setZero() {
    _translation.setZero();
    _orientation.setZero();
  }
  inline Pose<Scalar> inverse() const {
    Pose<Scalar> ret;
    ret.orientation() = _orientation.inverse();
    ret.translation() = ret.orientation() * -_translation;
    return ret;
  }
};

template <class T, size_t S>
inline Pose<T> indexBatch(const Pose<Batch<T, S>> &pose, size_t i) {
  return Pose<T>(indexBatch(pose.translation(), i),
                 indexBatch(pose.orientation(), i));
}

template <class T> auto &operator<<(std::ostream &stream, const Pose<T> &p) {
  return stream << "[ " << p.translation() << " " << p.orientation() << " ]";
}

template <class T> Pose<T> operator*(const Pose<T> &a, const Pose<T> &b) {
  Pose<T> x;
  x.translation() = a.translation() + a.orientation() * b.translation();
  x.orientation() = a.orientation() * b.orientation();
  return x;
}

template <class T> Vector3<T> operator*(const Pose<T> &a, const Vector3<T> &b) {
  return a.translation() + a.orientation() * b;
}

template <class T>
Pose<T> fg_angle_axis_pose(const T &angle, const Vector3<T> &axis) {
  Pose<T> ret;
  ret.translation().setZero();
  auto &quat = ret.orientation();
  T s = std::sin(angle * T(0.5));
  T c = std::cos(angle * T(0.5));
  quat.x() = axis.x() * s;
  quat.y() = axis.y() * s;
  quat.z() = axis.z() * s;
  quat.w() = c;
  return ret;
}

template <class T>
Pose<T> fg_pose_angle_axis_pose(const Pose<T> &parent, const T &angle,
                                const Vector3<T> &axis) {

  Quaternion<T> quat;
  T s = std::sin(angle * T(0.5));
  T c = std::cos(angle * T(0.5));
  quat.x() = axis.x() * s;
  quat.y() = axis.y() * s;
  quat.z() = axis.z() * s;
  quat.w() = c;

  Pose<T> ret;
  ret.translation() = parent.translation();
  ret.orientation() = parent.orientation() * quat;
  return ret;
}

template <class T> Vector3<T> fg_pose_translation(const Pose<T> &pose) {
  return pose.translation();
}

template <class T> Quaternion<T> fg_pose_orientation(const Pose<T> &pose) {
  return pose.orientation();
}

template <class T> Pose<T> fg_translation_pose(const Vector3<T> &translation) {
  Pose<T> pose;
  pose.translation() = translation;
  pose.orientation().setIdentity();
  return pose;
}

template <class T>
Twist<T> fg_translation_twist(const Vector3<T> &translation) {
  Twist<T> twist;
  twist.translation() = translation;
  twist.rotation().setZero();
  return twist;
}

template <class T>
Pose<T> fg_orientation_pose(const Quaternion<T> &orientation) {
  Pose<T> pose;
  pose.orientation() = orientation;
  pose.translation().setZero();
  return pose;
}

template <class T>
Pose<T> fg_pose_translate(const Pose<T> &parent,
                          const Vector3<T> &translation) {
  Pose<T> pose;
  pose.orientation() = parent.orientation();
  pose.translation() =
      parent.translation() + parent.orientation() * translation;
  return pose;
}

template <class T>
Twist<T> fg_pose_residual(const Pose<T> &a, const Pose<T> &b) {
  Twist<T> x;

  // x.translation() = a.translation() - b.translation();
  // x.rotation() = fg_quat_residual(a.orientation().inverse() *
  // b.orientation());

  // residual * a = b
  // (residual * a)^-1 = b^-1
  // a^-1 * residual^-1 = b^-1
  // residual  = a * b^-1
  x.translation() = b.translation() - a.translation();
  x.rotation() = -fg_quat_residual(a.orientation() * b.orientation().inverse());

  return x;
}

} // namespace tractor
