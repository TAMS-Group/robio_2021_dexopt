// (c) 2020-2021 Philipp Ruppel

#include <tractor/engines/simple.h>

#include <tractor/core/profiler.h>

namespace tractor {

void EngineBase::ExecutableBase::_parameterize(
    const Buffer &data, const std::shared_ptr<Memory> &memory) const {
  auto &temp = *(MemoryImpl *)(memory.get());
  temp.resize(std::max(temp.size(), _memory_size));
  for (auto &port : _parameters) {
    std::memcpy((uint8_t *)temp.data() + port.address(),
                (const uint8_t *)data.data() + port.offset(), port.size());
  }
}

void EngineBase::ExecutableBase::_input(
    const Buffer &input, const std::shared_ptr<Memory> &memory) const {
  auto &temp = *(MemoryImpl *)(memory.get());
  temp.resize(std::max(temp.size(), _memory_size));
  {
    TRACTOR_PROFILER("_input");
    for (auto &port : _inputs) {
      std::memcpy((uint8_t *)temp.data() + port.address(),
                  (const uint8_t *)input.data() + port.offset(), port.size());
    }
  }
}

void EngineBase::ExecutableBase::_output(
    const std::shared_ptr<const Memory> &memory, Buffer &output) const {
  auto &temp = *(const MemoryImpl *)(memory.get());
  output.resize(outputBufferSize());
  for (auto &port : _outputs) {
    std::memcpy((uint8_t *)output.data() + port.offset(),
                (const uint8_t *)temp.data() + port.address(), port.size());
  }
}

} // namespace tractor
