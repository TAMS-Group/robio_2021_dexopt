// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/core/engine.h>

namespace tractor {

class EngineBase : public Engine {

protected:
  class MemoryImpl : public Memory {
    AlignedStdVector<uint8_t> _data;

  public:
    MemoryImpl() {}
    MemoryImpl(const MemoryImpl &) = delete;
    MemoryImpl &operator=(const MemoryImpl &) = delete;
    inline const void *data() const { return _data.data(); }
    inline void *data() { return _data.data(); }
    inline size_t size() const { return _data.size(); }
    void resize(size_t s) { _data.resize(s); }
    void zero(size_t s) {
      _data.clear();
      _data.resize(s, 0);
    }
  };

protected:
  class ExecutableBase : public Executable {

  public:
    ExecutableBase() {}
    ExecutableBase(const ExecutableBase &) = delete;
    ExecutableBase &operator=(const ExecutableBase &) = delete;
    virtual void _input(const Buffer &input,
                        const std::shared_ptr<Memory> &memory) const override;
    virtual void
    _parameterize(const Buffer &data,
                  const std::shared_ptr<Memory> &memory) const override;
    virtual void _output(const std::shared_ptr<const Memory> &memory,
                         Buffer &output) const override;
  };

public:
  virtual std::shared_ptr<Memory> createMemory() override {
    return std::make_shared<MemoryImpl>();
  }
};

} // namespace tractor
