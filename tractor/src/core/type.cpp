// (c) 2020-2021 Philipp Ruppel

#include <tractor/core/type.h>

#include <iostream>
#include <map>

namespace tractor {

static std::map<TypeInfo, TypeInfo> g_gradient_type_map;

const TypeInfo &TypeInfo::gradientType(const TypeInfo &type) {
  auto it = g_gradient_type_map.find(type);
  if (it != g_gradient_type_map.end()) {
    return it->second;
  } else {
    return type;
  }
}

void TypeInfo::registerGradientType(const TypeInfo &type,
                                    const TypeInfo &gradient) {
  // std::cout << "gradient type " << type.name() << " " << gradient.name()
  //            << std::endl;
  g_gradient_type_map[type] = gradient;
}

} // namespace tractor
