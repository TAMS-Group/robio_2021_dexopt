// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "operator.h"
#include "recorder.h"

namespace tractor {

template <class T> auto range_constraint(const T &a, const T &lo, const T &hi) {
  return T(0);
}

TRACTOR_OP(range_constraint, (const T &a, const T &lo, const T &hi),
           { return T(0); })

TRACTOR_D(prepare, range_constraint,
          (const T &a, const T &lo, const T &hi, const T &x), {})
TRACTOR_D(forward, range_constraint,
          (const T &a, const T &lo, const T &hi, T &x), { x = T(0); })
TRACTOR_D(reverse, range_constraint, (T & a, T &lo, T &hi, const T &x), {
  a = T(0);
  lo = T(0);
  hi = T(0);
})

TRACTOR_D_LOOP(project, range_constraint,
               (const T &a, const T &lo, const T &hi, const T &da,
                const T &padding, T &dx),
               (a, lo, hi, da, padding, dx), {
                 T v = a + da;
                 int n = 0;
                 if (v < lo + padding) {
                   v = lo + padding;
                   n++;
                 }
                 if (v > hi - padding) {
                   v = hi - padding;
                   n++;
                 }
                 if (n == 2) {
                   v = (hi + lo) * T(0.5);
                 }
                 dx = v - a;
               })

TRACTOR_D_LOOP(barrier_init, range_constraint,
               (const T &a, const T &lo, const T &hi, const T &da, T &dx,
                T &ddx),
               (a, lo, hi, da, dx, ddx), {
                 T p = a + da;
                 T u = T(-1) / std::max(T(0), p - lo);
                 T v = T(+1) / std::max(T(0), hi - p);
                 dx = u + v;
                 ddx = (u * u) + (v * v);
               })
TRACTOR_D(barrier_step, range_constraint, (const T &dda, const T &da, T &dx),
          { dx = dda * da; })
TRACTOR_D(barrier_diagonal, range_constraint, (const T &dda, T &ddx),
          { ddx = dda; })

TRACTOR_D_LOOP(penalty_init, range_constraint,
               (const T &a, const T &lo, const T &hi, const T &da,
                const T &padding, T &dx, T &ddx),
               (a, lo, hi, da, padding, dx, ddx), {
                 T p = a + da;
                 T lo2 = lo + padding;
                 T hi2 = hi - padding;
                 dx = T(0);
                 ddx = T(0);
                 int n = 0;
                 if (p < lo2) {
                   dx = (lo2 - p);
                   ddx = T(1);
                   n++;
                 }
                 if (p > hi2) {
                   dx = (hi2 - p);
                   ddx = T(1);
                   n++;
                 }
                 if (n == 2) {
                   dx = ((lo2 + hi2) * T(0.5) - p);
                 }
               })
TRACTOR_D(penalty_step, range_constraint, (const T &dda, const T &da, T &dx),
          { dx = dda * da; })
TRACTOR_D(penalty_diagonal, range_constraint, (const T &dda, T &ddx),
          { ddx = dda; })

// -------------------------------------------------------------------

template <class T>
auto range_trust_region_constraint(const T &a, const T &lo, const T &hi,
                                   const T &tr) {
  return T(0);
}

TRACTOR_OP(range_trust_region_constraint,
           (const T &a, const T &lo, const T &hi, const T &tr),
           { return T(0); })

TRACTOR_D(prepare, range_trust_region_constraint,
          (const T &a, const T &lo, const T &hi, const T &tr, const T &x), {})
TRACTOR_D(forward, range_trust_region_constraint,
          (const T &a, const T &lo, const T &hi, const T &tr, T &x),
          { x = T(0); })
TRACTOR_D(reverse, range_trust_region_constraint,
          (T & a, T &lo, T &hi, T &tr, const T &x), {
            a = T(0);
            lo = T(0);
            hi = T(0);
            tr = T(0);
          })

TRACTOR_D_LOOP(project, range_trust_region_constraint,
               (const T &a, const T &lo, const T &hi, const T &tr, const T &da,
                const T &padding, T &dx),
               (a, lo, hi, tr, da, padding, dx), {
                 T v = a + da;
                 T lo2 = std::max(a - tr, lo) + padding;
                 T hi2 = std::min(a + tr, hi) - padding;
                 int n = 0;
                 if (v < lo2) {
                   v = lo2;
                   n++;
                 }
                 if (v > hi2) {
                   v = hi2;
                   n++;
                 }
                 if (n == 2) {
                   v = (hi2 + lo2) * T(0.5);
                 }
                 dx = v - a;
               })

TRACTOR_D_LOOP(barrier_init, range_trust_region_constraint,
               (const T &a, const T &lo, const T &hi, const T &tr, const T &da,
                T &dx, T &ddx),
               (a, lo, hi, tr, da, dx, ddx), {
                 T p = a + da;
                 T lo2 = std::max(a - tr, lo);
                 T hi2 = std::min(a + tr, hi);
                 T u = T(-1) / std::max(T(0), p - lo2);
                 T v = T(+1) / std::max(T(0), hi2 - p);
                 dx = u + v;
                 ddx = (u * u) + (v * v);
               })
TRACTOR_D(barrier_step, range_trust_region_constraint,
          (const T &dda, const T &da, T &dx), { dx = dda * da; })
TRACTOR_D(barrier_diagonal, range_trust_region_constraint,
          (const T &dda, T &ddx), { ddx = dda; })

TRACTOR_D_LOOP(penalty_init, range_trust_region_constraint,
               (const T &a, const T &lo, const T &hi, const T &tr, const T &da,
                const T &padding, T &dx, T &ddx),
               (a, lo, hi, tr, da, padding, dx, ddx), {
                 T p = a + da;
                 T lo2 = std::max(a - tr, lo) + padding;
                 T hi2 = std::min(a + tr, hi) - padding;
                 dx = T(0);
                 ddx = T(0);
                 int n = 0;
                 if (p < lo2) {
                   dx = (lo2 - p);
                   ddx = T(1);
                   n++;
                 }
                 if (p > hi2) {
                   dx = (hi2 - p);
                   ddx = T(1);
                   n++;
                 }
                 if (n == 2) {
                   dx = ((lo2 + hi2) * T(0.5) - p);
                 }
               })
TRACTOR_D(penalty_step, range_trust_region_constraint,
          (const T &dda, const T &da, T &dx), { dx = dda * da; })
TRACTOR_D(penalty_diagonal, range_trust_region_constraint,
          (const T &dda, T &ddx), { ddx = dda; })

// -------------------------------------------------------------------

template <class T> auto trust_region_constraint(const T &a, const T &tr) {
  return T(0);
}

TRACTOR_OP(trust_region_constraint, (const T &a, const T &tr), { return T(0); })

TRACTOR_D(prepare, trust_region_constraint,
          (const T &a, const T &tr, const T &x), {})
TRACTOR_D(forward, trust_region_constraint, (const T &a, const T &tr, T &x),
          { x = T(0); })
TRACTOR_D(reverse, trust_region_constraint, (T & a, T &tr, const T &x), {
  a = T(0);
  tr = T(0);
})

TRACTOR_D_LOOP(project, trust_region_constraint,
               (const T &a, const T &tr, const T &da, const T &padding, T &dx),
               (a, tr, da, padding, dx), {
                 T v = a + da;
                 T lo2 = (a - tr) + padding;
                 T hi2 = (a + tr) - padding;
                 int n = 0;
                 if (v < lo2) {
                   v = lo2;
                   n++;
                 }
                 if (v > hi2) {
                   v = hi2;
                   n++;
                 }
                 if (n == 2) {
                   v = (hi2 + lo2) * T(0.5);
                 }
                 dx = v - a;
               })

TRACTOR_D_LOOP(barrier_init, trust_region_constraint,
               (const T &a, const T &tr, const T &da, T &dx, T &ddx),
               (a, tr, da, dx, ddx), {
                 T p = a + da;
                 T lo2 = (a - tr);
                 T hi2 = (a + tr);
                 T u = T(-1) / std::max(T(0), p - lo2);
                 T v = T(+1) / std::max(T(0), hi2 - p);
                 dx = u + v;
                 ddx = (u * u) + (v * v);
               })
TRACTOR_D(barrier_step, trust_region_constraint,
          (const T &dda, const T &da, T &dx), { dx = dda * da; })
TRACTOR_D(barrier_diagonal, trust_region_constraint, (const T &dda, T &ddx),
          { ddx = dda; })

TRACTOR_D_LOOP(penalty_init, trust_region_constraint,
               (const T &a, const T &tr, const T &da, const T &padding, T &dx,
                T &ddx),
               (a, tr, da, padding, dx, ddx), {
                 T p = a + da;
                 T lo2 = a - tr + padding;
                 T hi2 = a + tr - padding;
                 dx = T(0);
                 ddx = T(0);
                 int n = 0;
                 if (p < lo2) {
                   dx = (lo2 - p);
                   ddx = T(1);
                   n++;
                 }
                 if (p > hi2) {
                   dx = (hi2 - p);
                   ddx = T(1);
                   n++;
                 }
                 if (n == 2) {
                   dx = ((lo2 + hi2) * T(0.5) - p);
                 }
               })
TRACTOR_D(penalty_step, trust_region_constraint,
          (const T &dda, const T &da, T &dx), { dx = dda * da; })
TRACTOR_D(penalty_diagonal, trust_region_constraint, (const T &dda, T &ddx),
          { ddx = dda; })

// -------------------------------------------------------------------

// template <class T> inline void variable(T &p) {}
template <class T>
inline Program::Input *
variable(T &p, Program::InputMode mode = Program::InputMode::Variable) {
  return nullptr;
}
template <class T>
inline Program::Input *
variable(T &p, const T &lo, const T &hi,
         Program::InputMode mode = Program::InputMode::Variable) {
  return nullptr;
}
template <class T>
inline Program::Input *
variable(Var<T> &p, Program::InputMode mode = Program::InputMode::Variable) {
  if (auto inst = Recorder::instance()) {
    return inst->input(&p, mode);
  }
  return nullptr;
}
template <class T>
inline Program::Input *
variable(Var<T> &p, const T &lo, const T &hi,
         Program::InputMode mode = Program::InputMode::Variable) {
  if (auto inst = Recorder::instance()) {
    goal(range_constraint(p, lo, hi));
    auto *var = inst->input(&p, lo, hi, mode);
    var->name() = "[" + std::to_string(lo) + "," + std::to_string(hi) + "]";
    return var;
  }
  return nullptr;
}

template <class... TT> inline Program::Input *slackVariable(TT &&... args) {
  auto *input = variable(args..., Program::InputMode::SlackVariable);
  if (input) {
    if (input->name().empty()) {
      input->name() = "slack";
    } else {
      input->name() = "slack " + input->name();
    }
  }
  return input;
}

/*
TRACTOR_OP(constraint_positive, (const T &a),
           {
           //return a > 0 ? T(0) : std::numeric_limits<T>::max();
           return T(0);
       })
TRACTOR_D(prepare, constraint_positive, (const T &a, const T &x), {})
TRACTOR_D(forward, constraint_positive, (const T &a, T &x), { x = T(0); })
TRACTOR_D(reverse, constraint_positive, (T & a, const T &x), { a = T(0); })
TRACTOR_D(project, constraint_positive,
          (const T &a, const T &da, const T &padding, T &dx),
          { dx = std::max(padding, a + da) - a; })
TRACTOR_D(barrier_init, constraint_positive,
          (const T &a, const T &da, T &dx, T &ddx), {
            dx = T(1) / std::max(T(0), a + da);
            ddx = -(dx * dx);
          })
TRACTOR_D(barrier_step, constraint_positive, (const T &dda, const T &da, T &dx),
          { dx = dda * da; })
*/

/*
TRACTOR_OP(interval_constraint, (const T &a), { return T(0); })
TRACTOR_D(prepare, interval_constraint, (const T &a, const T &x), {})
TRACTOR_D(forward, interval_constraint, (const T &a, T &x), { x = T(0); })
TRACTOR_D(reverse, interval_constraint, (T & a, const T &x), { a = T(0); })
TRACTOR_D(project, interval_constraint,
          (const T &a, const T &da, const T &padding, T &dx), {
            T v = a + da;
            int n = 0;
            if (v < padding - T(1)) {
              v = padding - T(1);
              n++;
            }
            if (v > T(1) - padding) {
              v = T(1) - padding;
              n++;
            }
            if (n == 2) {
              v = T(0);
            }
            dx = v - a;
          })
TRACTOR_D(barrier_init, interval_constraint,
          (const T &a, const T &da, T &dx, T &ddx), {
            T u = T(1) / std::max(T(0), T(1) + (a + da));
            T v = T(1) / std::max(T(0), T(1) - (a + da));
            dx = u + v;
            ddx = -(u * u) - (v * v);
          })
TRACTOR_D(barrier_step, interval_constraint, (const T &dda, const T &da, T &dx),
          { dx = dda * da; })
*/

} // namespace tractor
