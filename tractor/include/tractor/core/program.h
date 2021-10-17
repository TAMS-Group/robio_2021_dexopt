// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <cstring>
#include <functional>
#include <iostream>
#include <tuple>
#include <type_traits>
#include <typeindex>
#include <typeinfo>
#include <vector>

#include "array.h"
#include "buffer.h"
#include "type.h"

namespace tractor {

class compute;

class prepare;

class forward;
class reverse;

class project;

class barrier_init;
class barrier_step;
class barrier_diagonal;

class penalty_init;
class penalty_step;
class penalty_diagonal;

class Operator;

class Program {

public:
  template <class Instruction> class InstructionIterator {
    Instruction *_ptr = nullptr;

  public:
    inline InstructionIterator(Instruction *ptr) : _ptr(ptr) {}
    inline InstructionIterator &operator++() {
      _ptr += (1 + _ptr->argumentCount());
      return *this;
    }
    inline Instruction &operator*() const { return *_ptr; }
    inline bool operator==(const InstructionIterator &other) const {
      return _ptr == other._ptr;
    }
    inline bool operator!=(const InstructionIterator &other) const {
      return _ptr != other._ptr;
    }
    friend class Program;
  };

  class Instruction {
    uintptr_t _v = 0;
    inline const uintptr_t &get(size_t i) const { return (&_v)[i]; }
    inline uintptr_t &get(size_t i) { return (&_v)[i]; }

  public:
    inline Instruction() {}
    inline Instruction(uintptr_t v) : _v(v) {}
    inline Instruction(const Operator *op) : _v((uintptr_t)op) {}
    inline const Operator *op() const { return (const Operator *)get(0); }
    inline const uintptr_t &arg(size_t index) const { return get(index + 1); }
    inline uintptr_t &arg(size_t index) { return get(index + 1); }
    inline const uintptr_t &code() const { return _v; }
    inline uintptr_t &code() { return _v; }
    size_t argumentCount() const;
    inline auto args() const {
      return ArrayRef<const uintptr_t>(&_v + 1, &_v + 1 + argumentCount());
    }
    inline auto args() {
      return ArrayRef<uintptr_t>(&_v + 1, &_v + 1 + argumentCount());
    }
  };

  template <class Impl> class PortBase {
    TypeInfo _type;
    uintptr_t _address = 0;
    uintptr_t _offset = 0;
    std::string _name;

  public:
    inline PortBase(TypeInfo type, uintptr_t address, uintptr_t offset)
        : _type(type), _address(address), _offset(offset) {}

    template <class T, class... Args>
    static inline Impl make(const Args &... args) {
      return Impl(TypeInfo::get<T>(), args...);
    }

    inline const TypeInfo &typeInfo() const { return _type; }
    inline TypeInfo &typeInfo() { return _type; }
    inline const std::type_index &type() const { return _type.type(); }
    inline size_t size() const { return _type.size(); }

    inline uintptr_t address() const { return _address; }
    inline uintptr_t &address() { return _address; }

    inline uintptr_t offset() const { return _offset; }
    inline uintptr_t &offset() { return _offset; }

    auto &name() const { return _name; }
    auto &name() { return _name; }
  };

  template <class Impl> class BindablePortBase : public PortBase<Impl> {
    uintptr_t _binding = 0;

  public:
    inline BindablePortBase(TypeInfo type, uintptr_t address, uintptr_t offset,
                            uintptr_t binding)
        : PortBase<Impl>(type, address, offset), _binding(binding) {}
    inline uintptr_t binding() const { return _binding; }
    inline uintptr_t &binding() { return _binding; }
  };

  class Output;

  enum class InputMode {
    Variable,
    SlackVariable,
  };

  class Input : public BindablePortBase<Input> {
    intptr_t _lower_bound = -1;
    intptr_t _upper_bound = -1;
    // int _phase = 0;
    InputMode _mode = InputMode::Variable;

  public:
    inline Input(TypeInfo type, uintptr_t address, uintptr_t offset,
                 uintptr_t binding, intptr_t lower_bound = -1,
                 intptr_t upper_bound = -1,
                 InputMode mode = InputMode::Variable //, int phase = 0
                 )
        : BindablePortBase<Input>(type, address, offset, binding),
          _lower_bound(lower_bound), _upper_bound(upper_bound), //_phase(phase)
          _mode(mode) {}
    inline bool hasLowerBound() const { return _lower_bound >= intptr_t(0); }
    inline bool hasUpperBound() const { return _upper_bound >= intptr_t(0); }
    inline intptr_t lowerBoundAddress() const { return _lower_bound; }
    inline intptr_t upperBoundAddress() const { return _upper_bound; }
    // inline int phase() const { return _phase; }
    inline auto &mode() const { return _mode; }
    inline auto &mode() { return _mode; }
    explicit inline Input(const Output &output);
  };

  class Output : public BindablePortBase<Output> {

  public:
    inline Output(TypeInfo type, uintptr_t address, uintptr_t offset,
                  uintptr_t binding)
        : BindablePortBase<Output>(type, address, offset, binding) {}
    explicit inline Output(const Input &output);
  };

  class Parameter : public BindablePortBase<Parameter> {

  public:
    inline Parameter(TypeInfo type, uintptr_t address, uintptr_t offset,
                     uintptr_t binding)
        : BindablePortBase<Parameter>(type, address, offset, binding) {}
  };

  class Constant : public PortBase<Constant> {

  public:
    inline Constant(TypeInfo type, uintptr_t address, uintptr_t offset)
        : PortBase<Constant>(type, address, offset) {}
  };

  class Goal {
    size_t _port = 0;
    int _priority = 0;

  public:
    inline Goal(size_t port, int priority) : _port(port), _priority(priority) {}
    inline auto &port() const { return _port; }
    inline auto &priority() const { return _priority; }
    inline auto &port() { return _port; }
    inline auto &priority() { return _priority; }
  };

protected:
  std::vector<Instruction> _instructions;
  std::vector<Input> _inputs;
  std::vector<Output> _outputs;
  std::vector<Constant> _constants;
  std::vector<Parameter> _parameters;
  std::vector<uint8_t> _const_data;
  Buffer _bound_data;
  std::vector<Goal> _goals;
  size_t _memory_size = 0;

public:
  inline auto instructions() const {
    return ArrayRef<const Instruction, InstructionIterator<const Instruction>>(
        _instructions);
  }
  inline auto instructions() {
    return ArrayRef<Instruction, InstructionIterator<Instruction>>(
        _instructions);
  }
  template <class Iterator>
  inline void setInstructions(const Iterator &begin, const Iterator &end) {
    _instructions.assign(begin, end);
  }
  template <class Container>
  inline void setInstructions(const Container &container) {
    _instructions.assign(container.begin(), container.end());
  }

  inline auto code() const {
    return ArrayRef<const Instruction>(_instructions);
  }
  inline auto code() { return ArrayRef<Instruction>(_instructions); }
  inline void addCode(const Instruction &code) {
    _instructions.emplace_back(code);
  }

  inline auto goals() const { return ArrayRef<const Goal>(_goals); }
  inline auto goals() { return ArrayRef<Goal>(_goals); }
  inline auto &goal(size_t i) const { return _goals[i]; }
  inline auto &goal(size_t i) { return _goals[i]; }
  inline void addGoal(const Goal &goal) { _goals.push_back(goal); }

  inline auto inputs() const { return ArrayRef<const Input>(_inputs); }
  inline auto inputs() { return ArrayRef<Input>(_inputs); }
  inline auto &input(size_t i) const { return _inputs[i]; }
  inline auto &input(size_t i) { return _inputs[i]; }
  inline void addInput(const Input &port) { _inputs.push_back(port); }
  template <class List> inline void setInputs(const List &inputs) {
    _inputs.clear();
    for (auto &input : inputs) {
      _inputs.push_back(input);
    }
  }

  inline auto outputs() const { return ArrayRef<const Output>(_outputs); }
  inline auto outputs() { return ArrayRef<Output>(_outputs); }
  inline auto &output(size_t i) const { return _outputs[i]; }
  inline auto &output(size_t i) { return _outputs[i]; }
  inline void addOutput(const Output &port) { _outputs.push_back(port); }

  inline auto constants() const { return ArrayRef<const Constant>(_constants); }
  inline auto constants() { return ArrayRef<Constant>(_constants); }
  inline auto &constant(size_t i) const { return _constants[i]; }
  inline auto &constant(size_t i) { return _constants[i]; }
  inline void addConstant(const Constant &port) { _constants.push_back(port); }
  template <class List> inline void setConstants(const List &constants) {
    _constants.clear();
    for (auto &constant : constants) {
      _constants.push_back(constant);
    }
  }

  inline auto parameters() const {
    return ArrayRef<const Parameter>(_parameters);
  }
  inline auto parameters() { return ArrayRef<Parameter>(_parameters); }
  inline auto &parameter(size_t i) const { return _parameters[i]; }
  inline auto &parameter(size_t i) { return _parameters[i]; }
  inline void addParameter(const Parameter &port) {
    _parameters.push_back(port);
  }

  inline auto &constData() const { return _const_data; }
  template <class T> inline void setConstData(const T &d) {
    _const_data.assign(d.begin(), d.end());
  }

  inline auto &boundData() const { return _bound_data; }
  template <class T> inline void setBoundData(const T &d) {
    _bound_data.assign(d.begin(), d.end());
  }

  void clear() {
    _memory_size = 0;
    _instructions.clear();
    _inputs.clear();
    _outputs.clear();
    _constants.clear();
    _const_data.clear();
    _goals.clear();
  }

  size_t memorySize() const { return _memory_size; }
  // void setMemorySize(size_t s) { _memory_size = s; }
  void updateMemorySize(size_t s) { _memory_size = s; }

  Program() {}
  Program(const std::function<void()> &function) { record(function); }
  void record(const std::function<void()> &function);
};

inline Program::Input::Input(const Program::Output &p)
    : Program::Input(p.typeInfo(), p.address(), p.offset(), p.binding()) {}

inline Program::Output::Output(const Program::Input &p)
    : Program::Output(p.typeInfo(), p.address(), p.offset(), p.binding()) {}

std::ostream &operator<<(std::ostream &stream, const Program &prog);

} // namespace tractor
