// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <Eigen/Dense>
#include <immintrin.h> // for __m256
#include <cstdint>

namespace tractor {

template <class T, size_t S> class alignas(32) BatchStorage {
public:
  T _data[S];
  inline void _check() const {
    // for (size_t i = 0; i < S; i++) {
    //   if (!std::isfinite(_data[i])) {
    //     throw std::runtime_error("not finite");
    //   }
    // }
  }
};

template <size_t S> class alignas(32) BatchStorage<double, S> {
  static constexpr std::enable_if_t<S / 4 * 4 == S> *_validate_size = nullptr;

public:
  union {
    double _data[S];
    __m256d _simd[S / 4];
  };
  inline void _check() const {
    /*
  if ((((uintptr_t)(void *)this) & 31) != 0) {
    throw std::runtime_error("alignment error " +
                             std::to_string((uintptr_t)this));
  }
  */
  }
};

// -----------------------------------------------------------------------------

template <class T, size_t S>
class alignas(32) Batch : public BatchStorage<T, S> {

public:
  static constexpr size_t Size = S;
  inline Batch(const Batch &other) {
    other._check();
    this->_check();
    for (size_t i = 0; i < S; i++) {
      this->_data[i] = other._data[i];
    }
  }
  inline Batch() {
    this->_check();
    for (size_t i = 0; i < S; i++) {
      this->_data[i] = T();
    }
  }
  explicit inline Batch(const T &v) {
    this->_check();
    for (size_t i = 0; i < S; i++) {
      this->_data[i] = v;
    }
  }
  inline auto &data() const {
    this->_check();
    return this->_data;
  }
  inline auto &data() {
    this->_check();
    return this->_data;
  }
  inline auto &operator[](size_t i) const {
    this->_check();
    return this->_data[i];
  }
  inline auto &operator[](size_t i) {
    this->_check();
    return this->_data[i];
  }
};

// -----------------------------------------------------------------------------

template <class T, size_t S> inline auto operator-(const Batch<T, S> &v) {
  Batch<T, S> ret;
  for (size_t i = 0; i < S; i++) {
    ret[i] = -v[i];
  }
  return ret;
}

template <class T, size_t S> auto operator+(const Batch<T, S> &v) { return v; }

#define BATCH_OP_2(op)                                                         \
  template <class L, class R, size_t S>                                        \
  inline auto operator op(const Batch<L, S> &l, const Batch<R, S> &r) {        \
    Batch<decltype(l[0] op r[0]), S> ret;                                      \
    for (size_t i = 0; i < S; i++) {                                           \
      ret[i] = l[i] op r[i];                                                   \
    }                                                                          \
    return ret;                                                                \
  }
BATCH_OP_2(+)
BATCH_OP_2(-)
BATCH_OP_2(*)
BATCH_OP_2(/)

#define BATCH_OP_2_X(op)                                                       \
  template <class L, class R, size_t S>                                        \
  inline Batch<L, S> &operator op(Batch<L, S> &l, const Batch<R, S> &r) {      \
    for (size_t i = 0; i < S; i++) {                                           \
      l[i] op r[i];                                                            \
    }                                                                          \
    return l;                                                                  \
  }
BATCH_OP_2_X(+=)
BATCH_OP_2_X(-=)
BATCH_OP_2_X(*=)
BATCH_OP_2_X(/=)

// -----------------------------------------------------------------------------

template <size_t S>
inline Batch<double, S> operator+(const Batch<double, S> &a,
                                  const Batch<double, S> &b) {
  a._check();
  b._check();
  Batch<double, S> ret;
  for (size_t i = 0; i < S / 4; i++) {
    ret._simd[i] = _mm256_add_pd(a._simd[i], b._simd[i]);
  }
  return ret;
}

template <size_t S>
inline Batch<double, S> operator-(const Batch<double, S> &a,
                                  const Batch<double, S> &b) {
  a._check();
  b._check();
  Batch<double, S> ret;
  for (size_t i = 0; i < S / 4; i++) {
    ret._simd[i] = _mm256_sub_pd(a._simd[i], b._simd[i]);
  }
  return ret;
}

template <size_t S>
inline Batch<double, S> operator*(const Batch<double, S> &a,
                                  const Batch<double, S> &b) {
  a._check();
  b._check();
  Batch<double, S> ret;
  for (size_t i = 0; i < S / 4; i++) {
    ret._simd[i] = _mm256_mul_pd(a._simd[i], b._simd[i]);
  }
  return ret;
}

template <size_t S>
inline Batch<double, S> operator/(const Batch<double, S> &a,
                                  const Batch<double, S> &b) {
  a._check();
  b._check();
  Batch<double, S> ret;
  for (size_t i = 0; i < S / 4; i++) {
    ret._simd[i] = _mm256_div_pd(a._simd[i], b._simd[i]);
  }
  return ret;
}

template <size_t S>
inline Batch<double, S> &operator+=(Batch<double, S> &a,
                                    const Batch<double, S> &b) {
  a._check();
  b._check();
  for (size_t i = 0; i < S / 4; i++) {
    a._simd[i] = _mm256_add_pd(a._simd[i], b._simd[i]);
  }
  return a;
}

template <size_t S>
inline Batch<double, S> &operator-=(Batch<double, S> &a,
                                    const Batch<double, S> &b) {
  a._check();
  b._check();
  for (size_t i = 0; i < S / 4; i++) {
    a._simd[i] = _mm256_sub_pd(a._simd[i], b._simd[i]);
  }
  return a;
}

template <size_t S>
inline Batch<double, S> &operator*=(Batch<double, S> &a,
                                    const Batch<double, S> &b) {
  a._check();
  b._check();
  for (size_t i = 0; i < S / 4; i++) {
    a._simd[i] = _mm256_mul_pd(a._simd[i], b._simd[i]);
  }
  return a;
}

template <size_t S>
inline Batch<double, S> &operator/=(Batch<double, S> &a,
                                    const Batch<double, S> &b) {
  a._check();
  b._check();
  for (size_t i = 0; i < S / 4; i++) {
    a._simd[i] = _mm256_div_pd(a._simd[i], b._simd[i]);
  }
  return a;
}

template <size_t S>
inline Batch<double, S> operator-(const Batch<double, S> &a) {
  a._check();
  Batch<double, S> ret;
  for (size_t i = 0; i < S / 4; i++) {
    ret._simd[i] = _mm256_sub_pd(_mm256_set1_pd(0.0), a._simd[i]);
  }
  return ret;
}

// -----------------------------------------------------------------------------

template <class T> struct BatchScalar { typedef T Type; };
template <class T, size_t S> struct BatchScalar<Batch<T, S>> {
  typedef T Type;
};

template <class T> struct BatchSize { static constexpr size_t Size = 1; };
template <class T, size_t S> struct BatchSize<Batch<T, S>> {
  static constexpr size_t Size = S;
};

template <class F> class BatchLoop {
  F _f;

public:
  BatchLoop(const F &f) : _f(f) {}

  template <class... Args>
  auto run(Args &&... args) const -> decltype(std::declval<F>()(args...)) {
    _f(args...);
  }

  template <size_t S, class... Args>
  static constexpr size_t _findBatchSize(const Batch<Args, S> &... args) {
    return S;
  }

  template <class... Args>
  auto run(Args &&... args) const -> decltype(std::declval<F>()(args[0]...)) {
    static constexpr size_t N = _findBatchSize(args...);
    for (size_t i = 0; i < N; i++) {
      _f(args[i]...);
    }
  }
};
template <class F> auto makeBatchLoop(const F &f) { return BatchLoop<F>(f); }

template <class T> auto firstBatchElement(const T &v) { return v; }
template <class T, size_t S>
auto firstBatchElement(const tractor::Batch<T, S> &v) {
  return v[0];
}

typedef Batch<float, 4> Batch4f;
typedef Batch<double, 4> Batch4d;

typedef Batch<float, 8> Batch8f;
typedef Batch<double, 8> Batch8d;

typedef Batch<float, 16> Batch16f;
typedef Batch<double, 16> Batch16d;

} // namespace tractor

#define BATCH_FN(name)                                                         \
  namespace std {                                                              \
  template <class T, size_t S>                                                 \
  tractor::Batch<T, S> name(const tractor::Batch<T, S> &v) {                   \
    tractor::Batch<T, S> ret;                                                  \
    for (size_t i = 0; i < S; i++) {                                           \
      ret[i] = name(v[i]);                                                     \
    }                                                                          \
    return ret;                                                                \
  }                                                                            \
  }
BATCH_FN(sin)
BATCH_FN(cos)
BATCH_FN(tan)
BATCH_FN(sqrt)
BATCH_FN(log)
BATCH_FN(exp)
BATCH_FN(tanh)
BATCH_FN(acos)
BATCH_FN(asin)
BATCH_FN(atan)

namespace std {

template <class T, size_t S>
tractor::Batch<T, S> max(const tractor::Batch<T, S> &l,
                         const tractor::Batch<T, S> &r) {
  tractor::Batch<T, S> ret;
  for (size_t i = 0; i < S; i++) {
    ret[i] = std::max(l[i], r[i]);
  }
  return ret;
}

template <class T, size_t S>
tractor::Batch<T, S> min(const tractor::Batch<T, S> &l,
                         const tractor::Batch<T, S> &r) {
  tractor::Batch<T, S> ret;
  for (size_t i = 0; i < S; i++) {
    ret[i] = std::min(l[i], r[i]);
  }
  return ret;
}

} // namespace std
