// (c) 2020-2021 Philipp Ruppel

#include <tractor/core/operator.h>

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace tractor {

struct OperatorRegistry {
  std::unordered_map<std::string, const Operator *> name_map;
  std::unordered_map<std::type_index, OperatorModeMap *> group_map;
  std::unordered_map<
      std::type_index,
      std::unordered_map<std::type_index, std::unordered_set<const Operator *>>>
      op_map;
  static OperatorRegistry *instance() {
    static OperatorRegistry *instance = new OperatorRegistry();
    return instance;
  }
};

Operator::Operator(const std::string &name, const std::type_info &mode,
                   const std::type_info &op, const std::type_info &group)
    : _name(name), _op(op), _mode(mode) {
  // std::cout << name << " " << mode.name() << " " << group.name() <<
  // std::endl;
  auto *registry = OperatorRegistry::instance();
  registry->name_map[name] = this;
  auto &map = registry->group_map[group];
  if (!map) {
    map = new OperatorModeMap();
  }
  _map = map;
  map->put(OperatorModeMap::index(mode), this);
  registry->op_map[mode][op].insert(this);
}

Operator::~Operator() {}

const Operator *
Operator::tryFind(const std::type_index &mode, const std::type_index &op,
                  const std::initializer_list<std::type_index> &types) {
  auto *registry = OperatorRegistry::instance();
  auto it_mode = registry->op_map.find(mode);
  if (it_mode == registry->op_map.end()) {
    return nullptr;
  }
  auto it_op = it_mode->second.find(op);
  if (it_op == it_mode->second.end()) {
    return nullptr;
  }
  for (auto *op : it_op->second) {
    auto it_type = types.begin();
    auto it_arg = op->arguments().begin();
    while (true) {
      /*if (it_arg != op->arguments().end() && it_arg->isOutput()) {
        ++it_arg;
        continue;
      }*/
      if (it_type == types.end() &&
          (it_arg == op->arguments().end() || it_arg->isOutput())) {
        return op;
      }
      if (it_type == types.end()) {
        break;
      }
      if (it_arg == op->arguments().end()) {
        break;
      }
      if (it_arg->type() == *it_type) {
        ++it_arg;
        ++it_type;
        continue;
      } else {
        break;
      }
    }
  }
  return nullptr;
}

const Operator *Operator::tryFind(const std::type_index &mode,
                                  const std::type_index &group) {
  auto *registry = OperatorRegistry::instance();
  auto it_group = registry->group_map.find(group);
  if (it_group != registry->group_map.end()) {
    return it_group->second->at(OperatorModeMap::index(mode));
  } else {
    return nullptr;
  }
}

const Operator *Operator::tryFind(const std::string &name) {
  auto *registry = OperatorRegistry::instance();
  auto iter = registry->name_map.find(name);
  if (iter != registry->name_map.end()) {
    return iter->second;
  } else {
    return nullptr;
  }
}

std::vector<const Operator *> Operator::all() {
  auto *registry = OperatorRegistry::instance();
  std::vector<const Operator *> ret;
  for (auto &pair : registry->name_map) {
    ret.push_back(pair.second);
  }
  return ret;
}

size_t OperatorModeMap::index(const std::type_index &type) {
  static std::unordered_map<std::type_index, size_t> map;
  auto &i = map[type];
  if (!i) {
    i = map.size();
    // std::cout << "mode id " << type.name() << " " << i << std::endl;
  }
  return i;
}

} // namespace tractor
