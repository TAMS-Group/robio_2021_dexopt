// (c) 2020-2021 Philipp Ruppel

#include <tractor/engines/loop.h>

#include <algorithm>
#include <unordered_map>

namespace tractor {

void scheduleSimple(std::vector<const Program::Instruction *> &instructions) {

  std::vector<
      std::tuple<size_t, const Operator *, const Program::Instruction *>>
      level_inst;
  std::unordered_map<size_t, size_t> addr_level;
  for (auto *instp : instructions) {
    auto &inst = *instp;
    size_t level = 0;
    for (size_t iarg = 0; iarg < inst.argumentCount(); iarg++) {
      if (inst.op()->arg(iarg).isInput()) {
        level = std::max(level, addr_level[inst.arg(iarg)]);
      }
    }
    level++;
    for (size_t iarg = 0; iarg < inst.argumentCount(); iarg++) {
      if (inst.op()->arg(iarg).isOutput()) {
        addr_level[inst.arg(iarg)] = level;
      }
    }
    level_inst.emplace_back(level, inst.op(), &inst);
  }

  std::sort(level_inst.begin(), level_inst.end());

  instructions.clear();
  for (auto &tup : level_inst) {
    instructions.push_back(std::get<2>(tup));
  }
}

void LoopEngine::ExecutableImpl::_compile(const Program &program) {
  _instructions.clear();
  _arguments.clear();

  std::vector<const Program::Instruction *> instructions;
  for (auto &inst : program.instructions()) {
    instructions.push_back(&inst);
  }

  scheduleSimple(instructions);

  std::vector<const Operator *> ops;

  for (auto *instp : instructions) {
    auto *op = instp->op();
    if (_instructions.empty() ||
        _instructions.back().loop_function != op->functions().loop) {
      Instruction inst;
      inst.loop_function = op->functions().loop;
      inst.base = _arguments.size();
      inst.iterations = 1;
      _instructions.push_back(inst);
      ops.push_back(op);
    } else {
      auto &inst = _instructions.back();
      inst.iterations++;
    }
    for (auto &arg : instp->args()) {
      _arguments.push_back(arg);
    }
  }

  std::cout << "loops" << std::endl;
  for (size_t i = 0; i < ops.size(); i++) {
    std::cout << _instructions[i].iterations << " " << ops[i]->name()
              << std::endl;
  }
  std::cout << std::endl;
}

void LoopEngine::ExecutableImpl::_execute(
    const std::shared_ptr<Memory> &memory) const {

  auto &temp = *(MemoryImpl *)(memory.get());
  temp.resize(std::max(temp.size(), _memory_size));
  for (auto &port : _constants) {
    std::memcpy((uint8_t *)temp.data() + port.address(),
                _const_data.data() + port.offset(), port.size());
  }

  for (auto &inst : _instructions) {
    inst.loop_function(temp.data(), _arguments.data() + inst.base,
                       inst.iterations);
  }
}

} // namespace tractor
