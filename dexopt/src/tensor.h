// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/tractor.h>

#include "ops.h"

#include <random>

namespace tractor {

template <class Scalar> class Tensor {
  std::vector<size_t> _shape;
  AlignedStdVector<Scalar> _data;
  template <class... Indices> size_t _index(const Indices &... indices) const {
    if (_shape.size() != sizeof...(Indices)) {
      throw std::runtime_error("incorrect number of tensor index dimensions");
    }
    std::array<size_t, sizeof...(Indices)> ii = {indices...};
    size_t ret = 0;
    for (size_t i = 0; i < sizeof...(Indices); i++) {
      if (ii.at(i) >= _shape.at(i)) {
        throw std::runtime_error("tensor index out of range");
      }
      ret *= _shape[i];
      ret += ii[i];
    }
    return ret;
  }

public:
  Tensor() {}
  template <class... Shape> void resize(const Shape &... shape) {
    std::vector<size_t> sh = {shape...};
    if (sh != _shape) {
      _shape = sh;
      //_data.resize(size(), Scalar(0));
      _data.resize(size(), Scalar());
    }
  }
  template <class... Indices>
  auto &operator()(const Indices &... indices) const {
    return _data.at(_index(indices...));
  }
  template <class... Indices> auto &operator()(const Indices &... indices) {
    return _data.at(_index(indices...));
  }
  size_t size() const {
    if (_shape.empty()) {
      return 0;
    }
    size_t s = 1;
    for (auto &v : _shape) {
      s *= v;
    }
    return s;
  }
  size_t bytes() const { return size() * sizeof(Scalar); }
  const void *data() const { return _data.data(); }
  void *data() { return _data.data(); }
  auto &shape() const { return _shape; }
  bool empty() const { return size() == 0; }
  auto &operator[](size_t i) const { return _data.at(i); }
  auto &operator[](size_t i) { return _data.at(i); }
  bool isApprox(const Tensor<Scalar> &other) const {
    if (_shape != other._shape) {
      return false;
    }
    for (size_t i = 0; i < _data.size(); i++) {
      if (std::abs(_data[i] - other._data[i]) > 1e-6) {
        return false;
      }
    }
    return true;
  }
  template <class Derived> Tensor(const Eigen::MatrixBase<Derived> &m) {
    if (Derived::IsVectorAtCompileTime) {
      resize(m.size());
      for (size_t i = 0; i < m.size(); i++) {
        (*this)(i) = m(i);
      }
    } else {
      resize(m.rows(), m.cols());
      for (size_t r = 0; r < m.rows(); r++) {
        for (size_t c = 0; c < m.cols(); c++) {
          (*this)(r, c) = m(r, c);
        }
      }
    }
  }
  size_t rows() const {
    if (_shape.size() != 2) {
      throw std::runtime_error("tensor is not a matrix");
    }
    return _shape.at(0);
  }
  size_t cols() const {
    if (_shape.size() != 2) {
      throw std::runtime_error("tensor is not a matrix");
    }
    return _shape.at(1);
  }
  Tensor<Scalar> range(size_t start, size_t size) const {
    Tensor<Scalar> ret;
    ret.resize(size);
    if (_shape.size() != 1) {
      throw std::runtime_error(
          "this opperation currently only supports 1d vectors");
    }
    for (size_t i = 0; i < size; i++) {
      ret[i] = (*this)[i + start];
    }
    return ret;
  }
};

#define TRACTOR_CHECK_TENSOR_DIMENSIONS(tensor, dimensions)                    \
  if (tensor.shape().size() != dimensions) {                                   \
    throw std::runtime_error("tensor shape mismatch");                         \
  }

template <class Scalar>
std::ostream &operator<<(std::ostream &stream, const Tensor<Scalar> &tensor) {
  stream << "[ ";
  size_t s = tensor.size();
  for (size_t i = 0; i < s; i++) {
    std::cout << tensor[i] << " ";
  }
  stream << "]";
  return stream;
}

template <class ActivationScalar, class WeightScalar>
Tensor<ActivationScalar> tensor_mul_vec_mat(const Tensor<ActivationScalar> &a,
                                            const Tensor<WeightScalar> &b) {
  TRACTOR_CHECK_TENSOR_DIMENSIONS(a, 1);
  TRACTOR_CHECK_TENSOR_DIMENSIONS(b, 2);
  if (a.size() != b.rows()) {
    throw std::runtime_error("incompatible tensor shapes");
  }
  Tensor<ActivationScalar> r;
  r.resize(b.shape()[1]);
  size_t cols = b.shape()[1];
  size_t rows = b.shape()[0];
  for (size_t col = 0; col < cols; col++) {
    size_t row = 0;
    for (; row + 3 < rows; row += 4) {

      std::array<ActivationScalar, 4> weights;
      for (size_t i = 0; i < 4; i++) {
        batch(b(row + i, col), weights[i]);
      }

      r(col) = dot4add(

          a(row + 0), a(row + 1), a(row + 2), a(row + 3),

          weights[0], weights[1], weights[2], weights[3],

          r(col));
    }
    for (; row < rows; row++) {

      ActivationScalar weight;
      batch(b(row, col), weight);

      r(col) = madd(a(row), weight, r(col));
    }
  }
  return r;
}

template <class Scalar>
Tensor<Scalar> operator+(const Tensor<Scalar> &a, const Tensor<Scalar> &b) {
  if (a.shape() != b.shape()) {
    throw std::runtime_error("tensor shape mismatch");
  }
  size_t s = a.size();
  Tensor<Scalar> r = a;
  for (size_t i = 0; i < s; i++) {
    r[i] += b[i];
  }
  return r;
}

} // namespace tractor
