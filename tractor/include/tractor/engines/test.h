// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/engines/base.h>

namespace tractor {

class TestEngine : public Engine {

  struct MemoryUsage {
    std::type_index type = typeid(void);
  };

  class MemoryImpl : public Memory {
    std::vector<uint8_t> _data;
    std::vector<MemoryUsage> _usage;

  public:
    MemoryImpl() {}
    MemoryImpl(const MemoryImpl &) = delete;
    MemoryImpl &operator=(const MemoryImpl &) = delete;
    inline const void *data() const { return _data.data(); }
    inline void *data() { return _data.data(); }
    inline size_t size() const { return _data.size(); }
    void resize(size_t s) {
      _data.resize(s);
      _usage.resize(s);
    }
    void zero(size_t s) {
      _data.clear();
      _data.resize(s, 0);
      _usage.clear();
      _usage.resize(s);
    }
    template <class Port> void testWrite(const Port &port) {
      _usage.at(port.address()).type = port.type();
    }
    template <class Port> void testRead(const Port &port) const {
      auto &t = _usage.at(port.address()).type;
      if (t != port.type()) {
        if (t == typeid(void)) {
          throw std::runtime_error("output error, not initialized");
        } else {
          throw std::runtime_error("output error, type mismatch");
        }
      }
    }
    auto &usage(size_t i) const { return _usage.at(i); }
    auto &usage(size_t i) { return _usage.at(i); }
  };

  class ExecutableImpl : public Executable {
    Program _program;

  protected:
    ExecutableImpl(const ExecutableImpl &) = delete;
    ExecutableImpl &operator=(const ExecutableImpl &) = delete;
    virtual void _compile(const Program &program) override;
    virtual void _execute(const std::shared_ptr<Memory> &memory) const override;
    virtual void _input(const Buffer &input,
                        const std::shared_ptr<Memory> &memory) const override;
    virtual void
    _parameterize(const Buffer &data,
                  const std::shared_ptr<Memory> &memory) const override;
    virtual void _output(const std::shared_ptr<const Memory> &memory,
                         Buffer &output) const override;

  public:
    ExecutableImpl() {}
  };

public:
  virtual std::shared_ptr<Executable> createExecutable() override {
    return std::make_shared<ExecutableImpl>();
  }
  virtual std::shared_ptr<Memory> createMemory() override {
    return std::make_shared<MemoryImpl>();
  }
};

} // namespace tractor
