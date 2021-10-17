// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/core/eigen.h>
#include <tractor/core/recorder.h>
#include <tractor/geometry/ops.h>

namespace tractor {

template <class ScalarType> struct GeometryFastBase {
  typedef ScalarType Scalar;
  typedef ScalarType Value;
  typedef tractor::Pose<ScalarType> Pose;
  typedef tractor::Vector3<ScalarType> Vector3;
  typedef tractor::Quaternion<ScalarType> Orientation;
  typedef tractor::Twist<ScalarType> Twist;
  typedef tractor::Matrix3<ScalarType> Matrix3;
};

template <class ValueType> struct GeometryFastBase<Var<ValueType>> {
  typedef Var<ValueType> Scalar;
  typedef ValueType Value;
  typedef Var<tractor::Pose<ValueType>> Pose;
  typedef Var<tractor::Vector3<ValueType>> Vector3;
  typedef Var<tractor::Quaternion<ValueType>> Orientation;
  typedef Var<tractor::Twist<ValueType>> Twist;
  typedef Var<tractor::Matrix3<ValueType>> Matrix3;
};

template <class ScalarType> struct GeometryFast : GeometryFastBase<ScalarType> {

  typedef typename GeometryFastBase<ScalarType>::Scalar Scalar;
  typedef typename GeometryFastBase<ScalarType>::Value Value;
  typedef typename GeometryFastBase<ScalarType>::Pose Pose;
  typedef typename GeometryFastBase<ScalarType>::Vector3 Vector3;
  typedef typename GeometryFastBase<ScalarType>::Orientation Orientation;
  typedef typename GeometryFastBase<ScalarType>::Twist Twist;
  typedef typename GeometryFastBase<ScalarType>::Matrix3 Matrix3;
  // typedef typename GeometryFast<Value> Direct;

  static auto Vector3Zero() { return Vector3(tractor::Vector3<Value>::Zero()); }
  static auto TwistZero() { return Twist(tractor::Twist<Value>::Zero()); }
  static auto ScalarZero() { return Scalar(Value(0)); }
  static auto PoseIdentity() { return Pose(tractor::Pose<Value>::Identity()); }
  static auto Matrix3Zero() { return Matrix3(tractor::Matrix3<Value>::Zero()); }
  static auto OrientationIdentity() {
    return tractor::Quaternion<Value>::Identity();
  }

  static Orientation angleAxisOrientation(const Scalar &angle,
                                          const Vector3 &axis) {
    return fg_angle_axis_quat(angle, axis);
  }

  static Pose angleAxisPose(const Scalar &angle, const Vector3 &axis) {
    return fg_angle_axis_pose(angle, axis);
  }

  static Pose angleAxisPose(const Pose &parent, const Scalar &angle,
                            const Vector3 &axis) {
    // return parent * angleAxisPose(angle, axis);
    return fg_pose_angle_axis_pose(parent, angle, axis);
  }

  static Pose identityPose() { return Pose(tractor::Pose<Value>::Identity()); }

  static Pose translationPose(const Vector3 &pos) {
    return fg_translation_pose(pos);
  }

  static Pose translationPose(const Pose &parent, const Vector3 &pos) {
    // return parent * translationPose(pos);
    return fg_pose_translate(parent, pos);
  }

  static Pose translationPose(const Pose &parent, const Scalar &x,
                              const Scalar &y, const Scalar &z) {
    Vector3 translation;
    fg_vec3_pack(x, y, z, translation);
    return fg_pose_translate(parent, translation);
  }

  static Twist translationTwist(const Vector3 &translation) {
    return fg_translation_twist(translation);
  }

  static Twist twist(const Vector3 &translation, const Vector3 &rotation) {
    return fg_make_twist(translation, rotation);
  }

  static Pose orientationPose(const Orientation &orientation) {
    return fg_orientation_pose(orientation);
  }

  static Vector3 translation(const Pose &pose) {
    return fg_pose_translation(pose);
  }

  static Orientation orientation(const Pose &pose) {
    return fg_pose_orientation(pose);
  }

  static Vector3 translation(const Twist &twist) {
    return fg_twist_translation(twist);
  }

  static Vector3 rotation(const Twist &twist) {
    return fg_twist_rotation(twist);
  }

  static Vector3 pack(const Scalar &x, const Scalar &y, const Scalar &z) {
    Vector3 ret;
    fg_vec3_pack(x, y, z, ret);
    return ret;
  }

  static void unpack(const Vector3 &v, Scalar &x, Scalar &y, Scalar &z) {
    fg_vec3_unpack(v, x, y, z);
  }

  static void unpack(const Orientation &v, Scalar &x, Scalar &y, Scalar &z,
                     Scalar &w) {
    fg_quat_unpack(v, x, y, z, w);
  }

  static void unpack(const Pose &pose, Scalar &px, Scalar &py, Scalar &pz,
                     Scalar &qx, Scalar &qy, Scalar &qz, Scalar &qw) {
    unpack(translation(pose), px, py, pz);
    unpack(orientation(pose), qx, qy, qz, qw);
  }

  static Pose pose(const Scalar &px, const Scalar &py, const Scalar &pz,
                   const Scalar &qx, const Scalar &qy, const Scalar &qz,
                   const Scalar &qw) {
    Vector3 p;
    fg_vec3_pack(px, py, pz, p);
    Orientation q;
    fg_quat_pack(qx, qy, qz, qw, q);
    return translationPose(p) * orientationPose(q);
  }

  static Orientation pack(const Scalar &qx, const Scalar &qy, const Scalar &qz,
                          const Scalar &qw) {
    Orientation q;
    fg_quat_pack(qx, qy, qz, qw, q);
    return q;
  }

  static Orientation inverse(const Orientation &q) {
    return fg_quat_inverse(q);
  }

  static Pose inverse(const Pose &pose) {
    Orientation ret_orientation = inverse(orientation(pose));
    Vector3 ret_translation = ret_orientation * -translation(pose);
    return translationPose(ret_translation) * orientationPose(ret_orientation);
  }

  static Vector3 residual(const Orientation &a, const Orientation &b) {
    return fg_quat_residual(inverse(a) * b);
  }

  static Twist residual(const Pose &a, const Pose &b) {
    return fg_pose_residual(a, b);
  }

  template <class T, int Flags>
  static Vector3 import(const Eigen::Matrix<T, 3, 1, Flags> &p) {
    return Vector3(
        tractor::Vector3<Value>(Value(p.x()), Value(p.y()), Value(p.z())));
  }

  template <class T, int Flags>
  static Matrix3 import(const Eigen::Matrix<T, 3, 3, Flags> &p) {
    typename tractor::Matrix3<Value> ret;
    // Eigen::Map<Eigen::Matrix<T, 3, 3, Flags>>(&ret(0, 0)) = p;
    for (size_t row = 0; row < 3; row++) {
      for (size_t col = 0; col < 3; col++) {
        ret(row, col) = Value(p(row, col));
      }
    }
    return Matrix3(ret);
  }

  template <class T, int Flags>
  static Twist import(const Eigen::Matrix<T, 3, 1, Flags> &p,
                      const Eigen::Matrix<T, 3, 1, Flags> &r) {
    tractor::Twist<Value> ret;
    ret.translation().x() = Value(p.x());
    ret.translation().y() = Value(p.y());
    ret.translation().z() = Value(p.z());
    ret.rotation().x() = Value(r.x());
    ret.rotation().y() = Value(r.y());
    ret.rotation().z() = Value(r.z());
    return Twist(ret);
  }

  template <class T, int Mode, int Flags>
  static Pose import(const Eigen::Transform<T, 3, Mode, Flags> &pose) {
    auto q = Eigen::Quaterniond(Eigen::AngleAxisd(pose.linear()));
    return Pose(tractor::Pose<Value>(
        tractor::Vector3<Value>(Value(pose.translation().x()),
                                Value(pose.translation().y()),
                                Value(pose.translation().z())),
        tractor::Quaternion<Value>(Value(q.x()), Value(q.y()), Value(q.z()),
                                   Value(q.w()))));
  }
};

} // namespace tractor
