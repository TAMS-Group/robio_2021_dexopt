// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <cstring>

#include <tractor/core/allocator.h>
#include <tractor/core/program.h>

namespace tractor {

template <class T> class Var;

class Recorder {

  // const size_t memory_alignment = 64;

  Program *_program = nullptr;

  std::vector<Program::Instruction> _instructions;
  std::vector<Program::Input> _inputs;
  std::vector<Program::Output> _outputs;
  std::vector<Program::Parameter> _parameters;
  std::vector<Program::Goal> _goals;
  std::vector<Program::Constant> _constants;
  std::vector<uint8_t> _const_data;
  std::vector<uint8_t> _bound_data;
  // size_t _memory_size = memory_alignment;

  Allocator _alloc;

  template <class T> void outputImpl(const Var<T> *p, bool bind) {

    // uintptr_t temp = ((uintptr_t)_memory_size | 0x8000000000000000ul);
    //_memory_size += sizeof(T);

    // uintptr_t temp =
    //    (((uintptr_t)_alloc.alloc(sizeof(T))) | 0x8000000000000000ul);

    uintptr_t temp =
        (((uintptr_t)_alloc.alloc(TypeInfo::get<T>())) | 0x8000000000000000ul);

    move(&p->value(), (T *)temp);
    _outputs.emplace_back(TypeInfo::get<T>(), temp, 0,
                          bind ? (uintptr_t)&p->value() : 0);
  }

  void finish(Program &program);

public:
  static Recorder *instance();

  Recorder(Program *program);
  ~Recorder();

  Recorder(const Recorder &) = delete;
  Recorder &operator=(const Recorder &) = delete;

  template <class... Args> inline void op(const Operator *op, Args *... args) {
    _instructions.push_back((uintptr_t)op);
    const void *pointers[] = {(const void *)args...};
    for (size_t i = 0; i < sizeof...(args); i++) {
      _instructions.push_back((uintptr_t)pointers[i]);
    }
  }

  template <class T> inline void move(const T *from, T *to);

  template <class T> inline void rewrite(const T *from, T *to) {
    if (!_instructions.empty()) {
      if (_instructions.back().code() == (uintptr_t)from) {
        _instructions.back() = Program::Instruction((uintptr_t)to);
        return;
      }
    }
    move(from, to);
  }

  template <class T>
  auto *input(Var<T> *p,
              Program::InputMode mode = Program::InputMode::Variable) {
    _inputs.emplace_back(TypeInfo::get<T>(), (uintptr_t)&p->value(), 0,
                         (uintptr_t)&p->value(), -1, -1, mode);
    return &_inputs.back();
  }

  template <class T>
  auto *input(Var<T> *p, const T &lower, const T &upper,
              Program::InputMode mode = Program::InputMode::Variable) {
    size_t offset_lower = _bound_data.size();
    size_t offset_upper = _bound_data.size() + sizeof(T);
    _inputs.emplace_back(TypeInfo::get<T>(), (uintptr_t)&p->value(), 0,
                         (uintptr_t)&p->value(), offset_lower, offset_upper,
                         mode);
    _bound_data.resize(_bound_data.size() + sizeof(T) * 2);
    std::memcpy(_bound_data.data() + offset_lower, &lower, sizeof(T));
    std::memcpy(_bound_data.data() + offset_upper, &upper, sizeof(T));
    return &_inputs.back();
  }

  template <class T> void parameter(const Var<T> *p) {
    _parameters.emplace_back(TypeInfo::get<T>(), (uintptr_t)&p->value(), 0,
                             (uintptr_t)&p->value());
  }

  template <class T> void output(const Var<T> *p) { outputImpl(p, true); }

  template <class T> void constant(const Var<T> *p) {

    size_t start = _const_data.size();
    _const_data.resize(start + sizeof(T));
    std::memcpy(_const_data.data() + start, &p->value(), sizeof(T));

    uintptr_t addr = _alloc.alloc(TypeInfo::get<T>());

    _constants.emplace_back(TypeInfo::get<T>(), addr, (uintptr_t)start);

    // move((const T *)((uintptr_t)_memory_size | 0x8000000000000000ul),
    //     (T *)&p->value());
    //_memory_size += sizeof(T);

    uintptr_t temp = (addr | 0x8000000000000000ul);
    move((const T *)temp, (T *)&p->value());
  }

  template <class T>
  inline void goal(const Var<T> &v, int priority = 0,
                   const std::string &name = std::string()) {
    _goals.emplace_back(_outputs.size(), priority);
    outputImpl(&v, false);
    _outputs.back().name() = name;
  }
};

template <class T>
inline void goal(const Var<T> &v, int priority = 0,
                 const std::string &name = std::string()) {
  if (auto inst = Recorder::instance()) {
    inst->goal(v, priority, name);
  }
}

template <class T>
inline void goal(const T &v, int priority = 0,
                 const std::string &name = std::string()) {}

template <class T> inline void output(Var<T> &p) {
  if (auto inst = Recorder::instance()) {
    inst->output(&p);
  }
}

template <class T> inline void parameter(T &p) {}

template <class T> inline void parameter(Var<T> &p) {
  if (auto inst = Recorder::instance()) {
    inst->parameter(&p);
  }
}

template <class... Args>
inline void recordOperation(const Operator *op, Args *... args) {
  if (auto inst = Recorder::instance()) {
    inst->op(op, args...);
  }
}

} // namespace tractor
