// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/core/operator.h>

#include <limits>

namespace tractor {

using std::cos;
using std::exp;
using std::log;
using std::sin;
using std::sqrt;
using std::tanh;

TRACTOR_OP(move, (const T &a), { return a; })
TRACTOR_D(prepare, move, (const T &a, const T &x), {})
TRACTOR_D(forward, move, (const T &da, T &dx), { dx = da; })
TRACTOR_D(reverse, move, (T & da, const T &dx), { da = dx; })

TRACTOR_OP(zero, (T & x), { x = T(0); })
TRACTOR_D(prepare, zero, (const T &x), {})
TRACTOR_D(forward, zero, (T & dx), { dx = T(0); })
TRACTOR_D(reverse, zero, (const T &dx), {})

TRACTOR_OP(add, (const T &a, const T &b), { return a + b; })
TRACTOR_D(prepare, add, (const T &a, const T &b, const T &x), {})
TRACTOR_D(forward, add, (const T &da, const T &db, T &dx), { dx = da + db; })
TRACTOR_D(reverse, add, (T & da, T &db, const T &dx), {
  da = dx;
  db = dx;
})

TRACTOR_OP(sub, (const T &a, const T &b), { return a - b; })
TRACTOR_D(prepare, sub, (const T &a, const T &b, const T &x), {})
TRACTOR_D(forward, sub, (const T &da, const T &db, T &dx), { dx = da - db; })
TRACTOR_D(reverse, sub, (T & da, T &db, const T &dx), {
  da = dx;
  db = -dx;
})

TRACTOR_OP(minus, (const T &a), { return -a; })
TRACTOR_D(prepare, minus, (const T &a, const T &x), {})
TRACTOR_D(forward, minus, (const T &da, T &dx), { dx = -da; })
TRACTOR_D(reverse, minus, (T & da, const T &dx), { da = -dx; })

TRACTOR_OP(mul, (const T &a, const T &b), { return a * b; })
TRACTOR_D(prepare, mul,
          (const T &a, const T &b, const T &x, std::array<T, 2> &p), {
            p[0] = a;
            p[1] = b;
          })
TRACTOR_D(forward, mul,
          (const std::array<T, 2> &p, const T &da, const T &db, T &dx),
          { dx = da * p[1] + p[0] * db; })
TRACTOR_D(reverse, mul, (const std::array<T, 2> &p, T &da, T &db, const T &dx),
          {
            da = dx * p[1];
            db = dx * p[0];
          })

TRACTOR_OP(div, (const T &a, const T &b), { return a / b; })
TRACTOR_D(prepare, div,
          (const T &a, const T &b, const T &x, std::array<T, 3> &p), {
            p[0] = a;
            p[1] = b;
            p[2] = T(1) / b;
          })
TRACTOR_D(forward, div,
          (const std::array<T, 3> &p, const T &da, const T &db, T &dx),
          { dx = (da * p[1] - p[0] * db) * (p[2] * p[2]); })
TRACTOR_D(reverse, div, (const std::array<T, 3> &p, T &da, T &db, const T &dx),
          {
            const T &r = p[2];
            da = dx * r;
            db = -(dx * p[0]) * (r * r);
          })

template <class T> T madd(const T &a, const T &b, const T &c) {
  return a * b + c;
}
TRACTOR_OP(madd, (const T &a, const T &b, const T &c), { return a * b + c; })
TRACTOR_D(prepare, madd,
          (const T &a, const T &b, const T &c, const T &x, std::array<T, 2> &p),
          {
            p[0] = a;
            p[1] = b;
          })
TRACTOR_D(forward, madd,
          (const std::array<T, 2> &p, const T &da, const T &db, const T &dc,
           T &dx),
          { dx = da * p[1] + p[0] * db + dc; })
TRACTOR_D(reverse, madd,
          (const std::array<T, 2> &p, T &da, T &db, T &dc, const T &dx), {
            da = dx * p[1];
            db = dx * p[0];
            dc = dx;
          })

TRACTOR_OP(sin, (const T &a), { return T(sin(a)); })
TRACTOR_D(prepare, sin, (const T &a, const T &x, T &p), { p = cos(a); })
TRACTOR_D(forward, sin, (const T &p, const T &da, T &dx), { dx = da * p; })
TRACTOR_D(reverse, sin, (const T &p, T &da, const T &dx), { da = dx * p; })

TRACTOR_OP(cos, (const T &a), { return T(cos(a)); })
TRACTOR_D(prepare, cos, (const T &a, const T &x, T &p), { p = -sin(a); })
TRACTOR_D(forward, cos, (const T &p, const T &da, T &dx), { dx = da * p; })
TRACTOR_D(reverse, cos, (const T &p, T &da, const T &dx), { da = dx * p; })

TRACTOR_OP(sincos, (const T &a, T &s, T &c), {
  s = sin(a);
  c = cos(a);
})
TRACTOR_D(prepare, sincos, (const T &a, const T &s, const T &c, T &ps, T &pc), {
  ps = c;
  pc = -s;
})
TRACTOR_D(forward, sincos,
          (const T &ps, const T &pc, const T &da, T &ds, T &dc), {
            ds = da * ps;
            dc = da * pc;
          })
TRACTOR_D(reverse, sincos,
          (const T &ps, const T &pc, T &da, const T &ds, const T &dc),
          { da = ds * ps + dc * pc; })

TRACTOR_OP(log, (const T &a), { return T(log(a)); })
TRACTOR_D(prepare, log, (const T &a, const T &x, T &p), { p = T(1) / a; })
TRACTOR_D(forward, log, (const T &p, const T &da, T &dx), { dx = da * p; })
TRACTOR_D(reverse, log, (const T &p, T &da, const T &dx), { da = dx * p; })

TRACTOR_OP(exp, (const T &a), { return T(exp(a)); })
TRACTOR_D(prepare, exp, (const T &a, const T &x, T &p), { p = x; })
TRACTOR_D(forward, exp, (const T &p, const T &da, T &dx), { dx = da * p; })
TRACTOR_D(reverse, exp, (const T &p, T &da, const T &dx), { da = dx * p; })

TRACTOR_OP(tanh, (const T &a), { return T(tanh(a)); })
TRACTOR_D(prepare, tanh, (const T &a, const T &x, T &p), { p = T(1) - x * x; })
TRACTOR_D(forward, tanh, (const T &p, const T &da, T &dx), { dx = da * p; })
TRACTOR_D(reverse, tanh, (const T &p, T &da, const T &dx), { da = dx * p; })

TRACTOR_OP(square, (const T &a), { return a * a; })
TRACTOR_D(prepare, square, (const T &a, const T &x, T &p), { p = T(2) * a; })
TRACTOR_D(forward, square, (const T &p, const T &da, T &dx), { dx = p * da; })
TRACTOR_D(reverse, square, (const T &p, T &da, const T &dx), { da = p * dx; })

TRACTOR_OP(sqrt, (const T &a), { return T(sqrt(a)); })
TRACTOR_D(prepare, sqrt, (const T &a, const T &x, T &p), { p = T(0.5) / x; })
TRACTOR_D(forward, sqrt, (const T &p, const T &da, T &dx), { dx = p * da; })
TRACTOR_D(reverse, sqrt, (const T &p, T &da, const T &dx), { da = p * dx; })

} // namespace tractor
