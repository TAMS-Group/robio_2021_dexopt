// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/core/ops.h>
#include <tractor/core/recorder.h>

namespace tractor {

class VarBase {};

template <class T> class alignas(T) Var : public VarBase {
  T _x = T();

  // static constexpr uintptr_t ctest = 148791287391823;
  // uintptr_t vtest = 0;

public:
  typedef T Value;

  Var() {
    // vtest = ((uintptr_t)this ^ ctest);
    if (auto *inst = Recorder::instance()) {
      inst->constant(this);
    }
  }

  Var(const T &v) : _x(v) {
    // vtest = ((uintptr_t)this ^ ctest);
    if (auto *inst = Recorder::instance()) {
      inst->constant(this);
    }
  }

  Var(const Var &other) {
    // vtest = ((uintptr_t)this ^ ctest);
    _x = other._x;
    if (auto *inst = Recorder::instance()) {
      inst->move(&other._x, &_x);
    }
  }

  explicit operator T() const { return _x; }

  T &value() {
    // if (vtest != ((uintptr_t)this ^ ctest)) {
    //   throw std::runtime_error("var not initialized");
    // }
    return _x;
  }

  const T &value() const {
    // if (vtest != ((uintptr_t)this ^ ctest)) {
    //   throw std::runtime_error("var not initialized");
    // }
    return _x;
  }

  Var(Var &&other) {
    _x = other._x;
    if (auto *inst = Recorder::instance()) {
      inst->rewrite(&other._x, &_x);
    }
  }

  Var &operator=(const Var &other) {
    _x = other._x;
    if (auto *inst = Recorder::instance()) {
      inst->move(&other._x, &_x);
    }
    return *this;
  }

  Var &operator=(Var &&other) {
    _x = other._x;
    if (auto *inst = Recorder::instance()) {
      inst->rewrite(&other._x, &_x);
    }
    return *this;
  }
};

#define VAR_OP(op, fn)                                                         \
                                                                               \
  template <class A, class B,                                                  \
            class Ret = decltype(                                              \
                tractor::fn(*(const A *)nullptr, *(const B *)nullptr))>        \
  Ret operator op(const A &a, const B &b) {                                    \
    return std::move(tractor::fn(a, b));                                       \
  }                                                                            \
                                                                               \
  template <class A, class B>                                                  \
  auto &operator op##=(tractor::Var<A> &a, const B &b) {                       \
    a = std::move(tractor::fn(a, b));                                          \
    return a;                                                                  \
  }

VAR_OP(+, add)
VAR_OP(-, sub)
VAR_OP(*, mul)
VAR_OP(/, div)

template <class T> Var<T> operator-(const Var<T> &v) { return minus(v); }

typedef Var<float> tfloat;
typedef Var<double> tdouble;

template <class T>
std::ostream &operator<<(std::ostream &stream, const Var<T> &v) {
  return stream << v.value();
}

template <class T> const T &value(const T &v) { return v; }
template <class T> const T &value(const Var<T> &v) { return v.value(); }

template <class T> T &value(T &v) { return v; }
template <class T> T &value(Var<T> &v) { return v.value(); }

} // namespace tractor
