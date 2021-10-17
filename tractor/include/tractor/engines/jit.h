// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/engines/base.h>

namespace tractor {

class JITEngine : public EngineBase {

private:
  class JITProgramBase {
  protected:
    void *_data = 0;
    size_t _size = 0;
    void _free();

  public:
    JITProgramBase();
    ~JITProgramBase();
    JITProgramBase(const JITProgramBase &) = delete;
    JITProgramBase &operator=(const JITProgramBase &) = delete;
    void write(const void *data, size_t size);
    void alloc(size_t size);
    const void *data() const { return _data; }
  };

  template <class Signature> class JITProgram : public JITProgramBase {

  public:
    // template <class... T> auto call(const T &... args) const {
    //  return ((const Signature *)_data)(args...);
    //}

    template <class... T> void call(const T &... args) const {
      ((const Signature *)_data)(args...);
    }
  };

  class ExecutableImpl : public ExecutableBase {
    std::vector<uintptr_t> _arguments;
    JITProgram<void(void *)> _jit_function;

  public:
    ExecutableImpl();
    ~ExecutableImpl();

  protected:
    virtual void _compile(const Program &program) override;
    virtual void _execute(const std::shared_ptr<Memory> &memory) const override;
  };

public:
  virtual std::shared_ptr<Executable> createExecutable() override {
    return std::make_shared<ExecutableImpl>();
  }
};

} // namespace tractor
