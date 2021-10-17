// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <cmath>

namespace tractor {

template <class Scalar, class Function>
static Scalar minimizeTernary(const Function &f, const Scalar &tolerance,
                              Scalar low = Scalar(0), Scalar high = Scalar(1)) {
  Scalar span = std::abs(high - low);
  while (tolerance < span) {
    Scalar a = (low + low + high) * Scalar(1.0 / 3.0);
    Scalar b = (low + high + high) * Scalar(1.0 / 3.0);
    auto fa = f(a);
    auto fb = f(b);
    if (fb < fa) {
      low = a;
    } else {
      high = b;
    }
    span *= Scalar(2.0 / 3.0);
  }
  Scalar a = low;
  Scalar b = (high + low) * Scalar(0.5);
  auto fa = f(a);
  auto fb = f(b);
  if (fb < fa) {
    return b;
  } else {
    return a;
  }
}

/*
template <class Scalar, class Function>
static Scalar minimizeTernary(const Function &f, const Scalar &tolerance,
                              const Scalar &low = Scalar(0),
                              const Scalar &high = Scalar(1)) {
  if (!(tolerance < std::abs(high - low))) {
    Scalar a = low;
    Scalar b = (high + low) * Scalar(0.5);
    auto fa = f(a);
    auto fb = f(b);
    if (fb < fa) {
      return b;
    } else {
      return a;
    }
  }
  Scalar a = (low + low + high) * Scalar(1.0 / 3.0);
  Scalar b = (low + high + high) * Scalar(1.0 / 3.0);
  auto fa = f(a);
  auto fb = f(b);
  if (fb < fa) {
    return minimizeTernary(f, tolerance, b, high);
  } else {
    return minimizeTernary(f, tolerance, low, b);
  }
}
*/

template <class Scalar, class Function>
static Scalar minimizeGoldenSection(const Function &f, const Scalar &tolerance,
                                    const Scalar &low = Scalar(0),
                                    const Scalar &high = Scalar(1)) {
  static constexpr Scalar step = (std::sqrt(5.0) - 1.0) * 0.5;
  Scalar x1 = low;
  Scalar x4 = high;
  Scalar x2 = x4 - (x4 - x1) * step;
  Scalar x3 = x1 + (x4 - x1) * step;
  auto f2 = f(x2);
  auto f3 = f(x3);
  while (std::abs(x4 - x1) > tolerance) {
    // std::cout << "gss1 " << x1 << " " << x2 << " " << x3 << " " << x4 << " "
    //          << f2 << " " << f3 << std::endl;
    if (f3 < f2) {
      x1 = x2;
      x2 = x4 - (x4 - x1) * step;
      x3 = x1 + (x4 - x1) * step;
      f2 = f3;
      f3 = f(x3);
      // f2 = f(x2);
    } else {
      x4 = x3;
      x2 = x4 - (x4 - x1) * step;
      x3 = x1 + (x4 - x1) * step;
      f3 = f2;
      f2 = f(x2);
      // f3 = f(x3);
    }
    // std::cout << "gss2 " << x1 << " " << x2 << " " << x3 << " " << x4 << " "
    //          << f2 << " " << f3 << std::endl;
  }
  Scalar x = (x2 + x3) * Scalar(0.5);
  // Scalar x = x2;
  // return x;
  auto fx = f(x);
  /*
  if (x > low + tolerance * Scalar(5) && f(x - tolerance * Scalar(2)) < fx) {
    std::cout << x - tolerance * Scalar(2) << " "
              << f(x - tolerance * Scalar(2)) << std::endl;
    throw std::runtime_error("golden section line search failed 1");
  }
  if (x < high - tolerance * Scalar(5) && f(x + tolerance * Scalar(2)) < fx) {
    std::cout << x + tolerance * Scalar(2) << " "
              << f(x + tolerance * Scalar(2)) << std::endl;
    throw std::runtime_error("golden section line search failed 2");
  }
  */
  if (f(high) < fx) {
    return high;
  }
  if (fx < f(low)) {
    return x;
  }
  return low;
}

template <class Scalar, class Function>
static Scalar rootBisect(const Function &f, const Scalar &tolerance,
                         const Scalar &low = Scalar(0),
                         const Scalar &high = Scalar(1)) {

  Scalar x1 = low;
  Scalar x3 = high;
  // auto f1 = f(x1);
  // auto f3 = f(x3);
  // std::cout << "bisect " << f1 << " " << f3 << std::endl;

  while (std::abs(x3 - x1) > tolerance) {

    Scalar x2 = (x1 + x3) * Scalar(0.5);
    auto f2 = f(x2);

    // std::cout << "bisect " << x1 << " " << x2 << " " << x3 << " " << f1 << "
    // "
    //          << f2 << " " << f3 << std::endl;
    /*
    if (std::abs(f2) <= tolerance) {
      x1 = x3 = x2;
      f1 = f3 = f2;
      break;
    }
    */
    /*
    if ((f2 >= 0 && f3 <= 0) || (f2 <= 0 && f3 >= 0)) {
      x1 = x2;
      f1 = f2;
    } else {
      x3 = x2;
      f3 = f2;
    }
    */

    /*
    if ((f1 >= 0 && f2 <= 0) || (f1 <= 0 && f2 >= 0) || !std::isfinite(f3)) {
      x3 = x2;
      f3 = f2;
      continue;
    }

    if ((f2 > 0 && f3 < 0) || (f2 < 0 && f3 > 0)) {
      x1 = x2;
      f1 = f2;
      continue;
    }

    x3 = x2;
    f3 = f2;
    */

    if (f2 < Scalar(0)) {
      x1 = x2;
    } else {
      x3 = x2;
    }
  }

  // Scalar x = (x1 + x3) * Scalar(0.5);
  /*auto fx = f(x);
  if (std::abs(f(high)) < std::abs(fx)) {
    return high;
  }
  if (std::abs(fx) < std::abs(f(low))) {
    return x;
  }
  return low;*/

  return x1;
}

/*
template <class Scalar, class Function>
static Scalar rootSecant(const Function &f, const Scalar &tolerance,
                         const Scalar &low = Scalar(0),
                         const Scalar &high = Scalar(1)) {
  Scalar x1 = low;
  Scalar x3 = high;
  auto f1 = f(x1);
  auto f3 = f(x3);
  if (!std::isfinite(f1)) {
    return low;
  }
  // std::cout << "bisect " << f1 << " " << f3 << std::endl;
  // Scalar x2l = x1;
  while (std::abs(x3 - x1) > tolerance) {
    Scalar x2;
    if (f1 != f3 && ((f1 >= 0 && f3 <= 0) || (f1 <= 0 && f3 >= 0))) {
      x2 = x3 - f3 * (x3 - x1) / (f3 - f1);
      if (x2 <= x1 || x2 >= x3) {
        x2 = (x1 + x3) * Scalar(0.5);
      }
      // x2 = std::max(x1, std::min(x3, x2));
    } else {
      x2 = (x1 + x3) * Scalar(0.5);
    }
    // if (x2 == x2l) {
    //  break;
    //}
    // x2l = x2;
    auto f2 = f(x2);
    // std::cout << "secant " << x1 << " " << x2 << " " << x3 << " " << f1 << "
    // "
    //          << f2 << " " << f3 << std::endl;
    if (std::abs(f2) <= tolerance) {
      x1 = x3 = x2;
      f1 = f3 = f2;
      break;
    }
    if ((f2 >= 0 && f3 <= 0) || (f2 <= 0 && f3 >= 0)) {
      x1 = x2;
      f1 = f2;
      // std::cout << "a" << std::endl;
    } else {
      x3 = x2;
      f3 = f2;
      // std::cout << "b" << std::endl;
    }
  }
  Scalar x = (x1 + x3) * Scalar(0.5);
  auto fx = f(x);
  if (std::abs(f(high)) < std::abs(fx)) {
    return high;
  }
  if (std::abs(fx) < std::abs(f(low))) {
    return x;
  }
  return low;
}
*/

/*
template <class Scalar, class Function>
static Scalar rootSecant(const Function &f, const Scalar &tolerance,
                         const Scalar &low = Scalar(0),
                         const Scalar &high = Scalar(1)) {
  Scalar x1 = low;
  Scalar x3 = high;
  auto f1 = f(x1);
  auto f3 = f(x3);
  if (!std::isfinite(f1)) {
    return low;
  }
  while (std::abs(x3 - x1) > tolerance) {
    Scalar x2 = (x1 * f3 - x3 * f1) / (f3 - f1);
    auto f2 = f(x2);
    x1 = x3;
    f1 = f3;
    x3 = x2;
    f3 = f2;
    if (std::abs(f2) <= tolerance) {
      x1 = x3 = x2;
      f1 = f3 = f2;
    }
  }
  Scalar x = (x1 + x3) * Scalar(0.5);
  auto fx = f(x);
  if (std::abs(f(high)) < std::abs(fx)) {
    return high;
  }
  if (std::abs(fx) < std::abs(f(low))) {
    return x;
  }
  return low;
}
*/

} // namespace tractor
