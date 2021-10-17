// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/core/batch.h>
#include <tractor/geometry/vector3.h>

namespace tractor {

template <class Scalar> class Quaternion {
  Scalar _x = Scalar(), _y = Scalar(), _z = Scalar(), _w = Scalar();

public:
  inline Quaternion() {}
  inline Quaternion(const Scalar &x, const Scalar &y, const Scalar &z,
                    const Scalar &w)
      : _x(x), _y(y), _z(z), _w(w) {}
  inline auto &x() const { return _x; }
  inline auto &y() const { return _y; }
  inline auto &z() const { return _z; }
  inline auto &w() const { return _w; }
  inline auto &x() { return _x; }
  inline auto &y() { return _y; }
  inline auto &z() { return _z; }
  inline auto &w() { return _w; }
  inline void setIdentity() {
    _x = Scalar(0);
    _y = Scalar(0);
    _z = Scalar(0);
    _w = Scalar(1);
  }
  inline void setZero() {
    _x = Scalar(0);
    _y = Scalar(0);
    _z = Scalar(0);
    _w = Scalar(0);
  }
  inline auto inverse() const {
    return Quaternion<Scalar>(-x(), -y(), -z(), w());
  }
  inline auto vec() const { return Vector3<Scalar>(_x, _y, _z); }
  static inline Quaternion Identity() {
    return Quaternion(Scalar(0), Scalar(0), Scalar(0), Scalar(1));
  }
};

template <class T, size_t S>
inline Quaternion<T> indexBatch(const Quaternion<Batch<T, S>> &v, size_t i) {
  return Quaternion<T>(v.x()[i], v.y()[i], v.z()[i], v.w()[i]);
}

template <class T>
auto &operator<<(std::ostream &stream, const Quaternion<T> &v) {
  return stream << "[ " << v.x() << " " << v.y() << " " << v.z() << " " << v.w()
                << " ]";
}

template <class T> T norm(const Quaternion<T> &q) {
  return std::sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z() +
                   q.w() * q.w());
}

template <class T> Quaternion<T> normalized(const Quaternion<T> &q) {
  // T f = T(1) / norm(q);
  T n = norm(q);
  // T f = ((n > T(0)) ? (T(1) / n) : T(1));
  T f = T(1) / std::max(T(1e-9), n);
  return Quaternion<T>(q.x() * f, q.y() * f, q.z() * f, q.w() * f);
}

template <class T>
inline Vector3<T> operator*(const Quaternion<T> &q, const Vector3<T> &v) {

  T v_x = v.x();
  T v_y = v.y();
  T v_z = v.z();

  T q_x = q.x();
  T q_y = q.y();
  T q_z = q.z();
  T q_w = q.w();

  T t_x = q_y * v_z - q_z * v_y;
  T t_y = q_z * v_x - q_x * v_z;
  T t_z = q_x * v_y - q_y * v_x;

  T r_x = q_w * t_x + q_y * t_z - q_z * t_y;
  T r_y = q_w * t_y + q_z * t_x - q_x * t_z;
  T r_z = q_w * t_z + q_x * t_y - q_y * t_x;

  r_x += r_x;
  r_y += r_y;
  r_z += r_z;

  r_x += v_x;
  r_y += v_y;
  r_z += v_z;

  return Vector3<T>(r_x, r_y, r_z);

  // auto r = q * Quaternion<T>(v.x(), v.y(), v.z(), 0) * q.inverse();
  // return Vector3<T>(r.x(), r.y(), r.z());
}

template <class T>
inline Quaternion<T> operator*(const Quaternion<T> &p, const Quaternion<T> &q) {

  T p_x = p.x();
  T p_y = p.y();
  T p_z = p.z();
  T p_w = p.w();

  T q_x = q.x();
  T q_y = q.y();
  T q_z = q.z();
  T q_w = q.w();

  T r_x = (p_w * q_x + p_x * q_w) + (p_y * q_z - p_z * q_y);
  T r_y = (p_w * q_y - p_x * q_z) + (p_y * q_w + p_z * q_x);
  T r_z = (p_w * q_z + p_x * q_y) - (p_y * q_x - p_z * q_w);
  T r_w = (p_w * q_w - p_x * q_x) - (p_y * q_y + p_z * q_z);

  return Quaternion<T>(r_x, r_y, r_z, r_w);
}

template <class T>
void fg_quat_unpack(const Quaternion<T> &q, T &x, T &y, T &z, T &w) {
  x = q.x();
  y = q.y();
  z = q.z();
  w = q.w();
}

template <class T>
void fg_quat_pack(const T &x, const T &y, const T &z, const T &w,
                  Quaternion<T> &vec) {
  vec = Quaternion<T>(x, y, z, w);
}

template <class T> Vector3<T> fg_quat_residual(const Quaternion<T> &a) {
  return a.vec() * T(a.w() < 0 ? -2 : 2);
}

template <class T, size_t S>
Vector3<Batch<T, S>> fg_quat_residual(const Quaternion<Batch<T, S>> &a) {
  Batch<T, S> f;
  for (size_t i = 0; i < S; i++) {
    f[i] = T(a.w()[i] < 0 ? -2 : 2);
  }
  return a.vec() * f;
}

template <class T>
Quaternion<T> fg_angle_axis_quat(const T &angle, const Vector3<T> &axis) {
  Quaternion<T> quat;
  T s = std::sin(angle * T(0.5));
  T c = std::cos(angle * T(0.5));
  quat.x() = axis.x() * s;
  quat.y() = axis.y() * s;
  quat.z() = axis.z() * s;
  quat.w() = c;
  return quat;
}

} // namespace tractor
