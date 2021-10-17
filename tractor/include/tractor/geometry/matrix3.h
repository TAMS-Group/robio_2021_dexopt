// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <cstddef>

#include <tractor/geometry/vector3.h>

namespace tractor {

template <class Scalar> class Matrix3 {
  Scalar _data[9];

public:
  inline auto &operator()(size_t row, size_t col) const {
    return _data[row * 3 + col];
  }
  inline auto &operator()(size_t row, size_t col) {
    return _data[row * 3 + col];
  }
  inline void setZero() {
    for (size_t i = 0; i < 9; i++) {
      _data[i] = Scalar(0);
    }
  }
  static Matrix3 Zero() {
    Matrix3 ret;
    ret.setZero();
    return ret;
  }
  // inline const Scalar *data() const { return _data; }
  // inline Scalar *data() { return _data; }
};

template <class T> auto &operator<<(std::ostream &stream, const Matrix3<T> &v) {
  stream << "[ ";
  for (size_t row = 0; row < 3; row++) {
    stream << "[ ";
    for (size_t col = 0; col < 3; col++) {
      stream << v(row, col) << " ";
    }
    stream << "] ";
  }
  stream << "]";
  return stream;
}

template <class T>
Vector3<T> operator*(const Matrix3<T> &a, const Vector3<T> &b) {
  Vector3<T> ret = Vector3<T>::Zero();
  for (size_t row = 0; row < 3; row++) {
    for (size_t col = 0; col < 3; col++) {
      ret[row] += a(row, col) * b[col];
    }
  }
  return ret;
}

template <class T>
Matrix3<T> operator+(const Matrix3<T> &a, const Matrix3<T> &b) {
  Matrix3<T> ret;
  for (size_t row = 0; row < 3; row++) {
    for (size_t col = 0; col < 3; col++) {
      ret(row, col) = a(row, col) * b(row, col);
    }
  }
  return ret;
}

} // namespace tractor
