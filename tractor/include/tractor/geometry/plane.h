// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/geometry/vector3.h>

namespace tractor {

template <class Scalar> class Plane {
  Vector3<Scalar> _normal;
  Scalar _offset = Scalar(0);

public:
  Plane() {}
  Plane(const Vector3<Scalar> &normal, const Scalar &offset)
      : _normal(normal), _offset(offset) {}
  Plane(const Vector3<Scalar> &normal, const Vector3<Scalar> &point)
      : _normal(normal), _offset(-dot(normal, point)) {}
  inline auto &normal() const { return _normal; }
  inline auto &normal() { return _normal; }
  inline auto &offset() const { return _offset; }
  inline auto &offset() { return _offset; }
  inline Scalar signedDistance(const Vector3<Scalar> &p) const {
    return dot(_normal, p) + _offset;
  }
  inline Vector3<Scalar> point() const { return _normal * -_offset; }
};

template <class T> auto operator-(const Plane<T> &p) {
  return Plane<T>(-p.normal(), -p.offset());
}

template <class T> auto &operator<<(std::ostream &stream, const Plane<T> &p) {
  return stream << "[ p:" << p.point() << " n:" << p.normal()
                << " o:" << p.offset() << " ]";
}

} // namespace tractor
