// (c) 2020-2021 Philipp Ruppel

#include <tractor/engines/simple.h>

namespace tractor {

void SimpleEngine::ExecutableImpl::_compile(const Program &program) {
  _instructions.clear();
  _arguments.clear();
  for (auto &instp : program.instructions()) {
    Instruction inst;
    auto *op = instp.op();
    inst.op = op->functions().indirect;
    inst.base = _arguments.size();
    _instructions.push_back(inst);
    for (auto &arg : instp.args()) {
      _arguments.push_back(arg);
    }
  }
}

void SimpleEngine::ExecutableImpl::_execute(
    const std::shared_ptr<Memory> &memory) const {

  auto &temp = *(MemoryImpl *)(memory.get());
  temp.resize(std::max(temp.size(), _memory_size));
  for (auto &port : _constants) {
    std::memcpy((uint8_t *)temp.data() + port.address(),
                _const_data.data() + port.offset(), port.size());
  }

  for (auto &inst : _instructions) {
    inst.op(temp.data(), _arguments.data() + inst.base);
  }
}

} // namespace tractor
