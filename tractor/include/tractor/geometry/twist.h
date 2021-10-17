// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/geometry/quaternion.h>
#include <tractor/geometry/vector3.h>

namespace tractor {

template <class Scalar> class Twist {
  Vector3<Scalar> _translation;
  Vector3<Scalar> _rotation;

public:
  Twist() {}
  Twist(const Vector3<Scalar> &translation, const Vector3<Scalar> &rotation)
      : _translation(translation), _rotation(rotation) {}
  inline auto &translation() const { return _translation; }
  inline auto &translation() { return _translation; }
  inline auto &rotation() const { return _rotation; }
  inline auto &rotation() { return _rotation; }
  inline void setZero() {
    _translation.setZero();
    _rotation.setZero();
  }
  inline static Twist Zero() {
    return Twist<Scalar>(Vector3<Scalar>::Zero(), Vector3<Scalar>::Zero());
  }
  inline auto &operator[](size_t i) const { return (&_translation.x())[i]; }
  inline auto &operator[](size_t i) { return (&_translation.x())[i]; }
};

template <class T, size_t S>
inline Twist<T> indexBatch(const Twist<Batch<T, S>> &pose, size_t i) {
  return Twist<T>(indexBatch(pose.translation(), i),
                  indexBatch(pose.rotation(), i));
}

template <class T> auto &operator<<(std::ostream &stream, const Twist<T> &p) {
  return stream << "[ " << p.translation() << " " << p.rotation() << " ]";
}

template <class T> inline Twist<T> operator-(const Twist<T> &v) {
  return Twist<T>(-v.translation(), -v.rotation());
}
template <class T> inline Twist<T> operator+(const Twist<T> &v) { return v; }

template <class T>
inline Twist<T> operator+(const Twist<T> &a, const Twist<T> &b) {
  Twist<T> x;
  x.translation() = a.translation() + b.translation();
  x.rotation() = a.rotation() + b.rotation();
  return x;
}

template <class T>
inline Twist<T> operator-(const Twist<T> &a, const Twist<T> &b) {
  Twist<T> x;
  x.translation() = a.translation() - b.translation();
  x.rotation() = a.rotation() - b.rotation();
  return x;
}

template <class T> inline Twist<T> &operator+=(Twist<T> &a, const Twist<T> &b) {
  a.translation() += b.translation();
  a.rotation() += b.rotation();
  return a;
}

template <class T> inline Twist<T> &operator-=(Twist<T> &a, const Twist<T> &b) {
  a.translation() -= b.translation();
  a.rotation() -= b.rotation();
  return a;
}

template <class T> inline Twist<T> operator*(const Twist<T> &a, const T &b) {
  return Twist<T>(a.translation() * b, a.rotation() * b);
}

template <class T> inline Twist<T> &operator*=(Twist<T> &a, const T &b) {
  a.translation() *= b;
  a.rotation() *= b;
  return a;
}

template <class T>
Vector3<T> operator*(const Twist<T> &a, const Vector3<T> &p) {
  return p + a.translation() + cross(a.rotation(), p);
}

template <class T>
inline Twist<T> fg_make_twist(const Vector3<T> &translation,
                              const Vector3<T> &rotation) {
  Twist<T> twist;
  twist.translation() = translation;
  twist.rotation() = rotation;
  return twist;
}

template <class T>
inline Vector3<T> fg_twist_translation(const Twist<T> &twist) {
  return twist.translation();
}

template <class T> inline Vector3<T> fg_twist_rotation(const Twist<T> &twist) {
  return twist.rotation();
}

} // namespace tractor
