// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <cstring>

#include <tractor/core/operator.h>

namespace tractor {

struct Memory {
  virtual ~Memory() {}
};

class Executable {
private:
  Buffer _temp;
  size_t _input_size = 0, _output_size = 0, _param_size = 0;
  bool _compiled = false;
  void _checkCompiled() const {
    if (!_compiled) {
      throw std::runtime_error("call compile(...) before use");
    }
  }
  virtual void _compile(const Program &program) = 0;
  virtual void _execute(const std::shared_ptr<Memory> &memory) const = 0;
  virtual void _input(const Buffer &input,
                      const std::shared_ptr<Memory> &memory) const = 0;
  virtual void _output(const std::shared_ptr<const Memory> &memory,
                       Buffer &output) const = 0;
  virtual void _parameterize(const Buffer &data,
                             const std::shared_ptr<Memory> &memory) const = 0;

protected:
  std::vector<Program::Input> _inputs;
  std::vector<Program::Output> _outputs;
  std::vector<Program::Constant> _constants;
  std::vector<Program::Parameter> _parameters;
  size_t _memory_size = 0;
  std::vector<uint8_t> _const_data;

public:
  virtual ~Executable() {}
  void compile(const Program &program);
  void execute(const std::shared_ptr<Memory> &memory) const;
  void input(const Buffer &input, const std::shared_ptr<Memory> &memory) const;
  void output(const std::shared_ptr<const Memory> &memory,
              Buffer &output) const;
  ArrayRef<const Program::Input> inputs() const {
    _checkCompiled();
    return _inputs;
  }
  ArrayRef<const Program::Output> outputs() const {
    _checkCompiled();
    return _outputs;
  }
  ArrayRef<const Program::Parameter> parameters() const {
    _checkCompiled();
    return _parameters;
  }
  size_t inputBufferSize() const {
    _checkCompiled();
    return _input_size;
  }
  size_t outputBufferSize() const {
    _checkCompiled();
    return _output_size;
  }
  template <class Vector>
  void inputVector(Vector &&inputv, const std::shared_ptr<Memory> &memory) {
    _checkCompiled();
    _temp.fromVector(inputs(), inputv);
    input(_temp, memory);
  }
  template <class Vector>
  void outputVector(const std::shared_ptr<const Memory> &memory,
                    Vector &&outputv) {
    // std::cout << "a" << __LINE__ << std::endl;
    _checkCompiled();
    // std::cout << "a" << __LINE__ << std::endl;
    output(memory, _temp);
    // std::cout << "a" << __LINE__ << std::endl;
    _temp.toVector(outputs(), outputv);
    // std::cout << "a" << __LINE__ << std::endl;
  }
  template <class Input, class Output>
  void run(Input &&input, const std::shared_ptr<Memory> &memory,
           Output &&output) {
    inputVector(input, memory);
    // std::cout << "a" << __LINE__ << std::endl;
    execute(memory);
    // std::cout << "a" << __LINE__ << std::endl;
    outputVector(memory, output);
    // std::cout << "a" << __LINE__ << std::endl;
  }
  void parameterize(const Buffer &data,
                    const std::shared_ptr<Memory> &memory) const;
  void parameterize(const std::shared_ptr<Memory> &memory);
  template <class Vector>
  void parameterVector(const Vector &paramv,
                       const std::shared_ptr<Memory> &memory) {
    _checkCompiled();
    _temp.fromVector(parameters(), paramv);
    parameterize(_temp, memory);
  }
};

struct Engine {
  virtual std::shared_ptr<Memory> createMemory() = 0;
  virtual std::shared_ptr<Executable> createExecutable() = 0;
  std::shared_ptr<Executable> compile(const Program &program);
};

} // namespace tractor
