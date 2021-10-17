// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <cstring>
#include <type_traits>
#include <typeindex>

namespace tractor {

class TypeInfo {
  size_t _size = 0;
  std::type_index _type = typeid(void);
  size_t _alignment = 0;

public:
  inline TypeInfo() {}
  template <class T> static inline TypeInfo get() {
    typedef typename std::decay<T>::type X;
    TypeInfo t;
    t._size = sizeof(X);
    t._type = typeid(X);
    t._alignment = std::alignment_of<X>::value;
    return t;
  }
  inline size_t size() const { return _size; }
  inline const std::type_index &type() const { return _type; }
  inline const char *name() const { return _type.name(); }
  static const TypeInfo &gradientType(const TypeInfo &type);
  const TypeInfo &gradientType() const { return gradientType(*this); }
  static void registerGradientType(const TypeInfo &type,
                                   const TypeInfo &gradient);
  size_t alignment() const { return _alignment; }
};

#define TRACTOR_GRADIENT_TYPE_CONCAT_2(a, b) a##b

#define TRACTOR_GRADIENT_TYPE_CONCAT(a, b) TRACTOR_GRADIENT_TYPE_CONCAT_2(a, b)

#define TRACTOR_GRADIENT_TYPE(type, gradient)                                  \
  static int TRACTOR_GRADIENT_TYPE_CONCAT(g_gradient_type_reg_, __COUNTER__) = \
      (TypeInfo::registerGradientType(TypeInfo::get<type>(),                   \
                                      TypeInfo::get<gradient>()),              \
       0);

#define TRACTOR_GRADIENT_TYPE_SPECIALIZE(type, gradient, scalar)               \
  namespace TRACTOR_GRADIENT_TYPE_CONCAT(g_gradient_type_reg_, __COUNTER__) {  \
    typedef scalar T;                                                          \
    TRACTOR_GRADIENT_TYPE(type, gradient)                                      \
  }

#define TRACTOR_GRADIENT_TYPE_TEMPLATE(type, gradient)                         \
  TRACTOR_GRADIENT_TYPE_SPECIALIZE(type, gradient, double)                     \
  TRACTOR_GRADIENT_TYPE_SPECIALIZE(type, gradient, float)                      \
  TRACTOR_GRADIENT_TYPE_SPECIALIZE(type, gradient, int)

inline bool operator==(const TypeInfo &a, const TypeInfo &b) {
  return a.type() == b.type();
}

inline bool operator!=(const TypeInfo &a, const TypeInfo &b) {
  return a.type() != b.type();
}

inline bool operator<(const TypeInfo &a, const TypeInfo &b) {
  return a.type() < b.type();
}

inline bool operator>(const TypeInfo &a, const TypeInfo &b) {
  return a.type() > b.type();
}

inline bool operator<=(const TypeInfo &a, const TypeInfo &b) {
  return a.type() <= b.type();
}

inline bool operator>=(const TypeInfo &a, const TypeInfo &b) {
  return a.type() >= b.type();
}

} // namespace tractor
