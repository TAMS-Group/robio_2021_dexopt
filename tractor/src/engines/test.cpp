// (c) 2020-2021 Philipp Ruppel

#include <tractor/engines/test.h>

#include <tractor/core/ops.h>

namespace tractor {

void TestEngine::ExecutableImpl::_parameterize(
    const Buffer &data, const std::shared_ptr<Memory> &memory) const {
  auto &temp = *(MemoryImpl *)(memory.get());
  temp.resize(std::max(temp.size(), _memory_size));
  for (auto &port : _parameters) {
    std::memcpy((uint8_t *)temp.data() + port.address(),
                (const uint8_t *)data.data() + port.offset(), port.size());
    temp.testWrite(port);
  }
}

void TestEngine::ExecutableImpl::_input(
    const Buffer &input, const std::shared_ptr<Memory> &memory) const {
  auto &temp = *(MemoryImpl *)(memory.get());
  temp.resize(std::max(temp.size(), _memory_size));
  {
    for (auto &port : _inputs) {
      std::memcpy((uint8_t *)temp.data() + port.address(),
                  (const uint8_t *)input.data() + port.offset(), port.size());
      temp.testWrite(port);
    }
  }
}

void TestEngine::ExecutableImpl::_output(
    const std::shared_ptr<const Memory> &memory, Buffer &output) const {
  auto &temp = *(const MemoryImpl *)(memory.get());
  output.resize(outputBufferSize());
  for (auto &port : _outputs) {
    std::memcpy((uint8_t *)output.data() + port.offset(),
                (const uint8_t *)temp.data() + port.address(), port.size());
    temp.testRead(port);
  }
}

void TestEngine::ExecutableImpl::_compile(const Program &program) {
  _program = program;
}

void TestEngine::ExecutableImpl::_execute(
    const std::shared_ptr<Memory> &memory) const {

  // std::cout << "test engine begin" << std::endl;

  /*{
    std::string s;
    std::cin >> s;
}*/

  auto &temp = *(MemoryImpl *)(memory.get());
  temp.resize(std::max(temp.size(), _memory_size));
  for (auto &port : _constants) {
    std::memcpy((uint8_t *)temp.data() + port.address(),
                _const_data.data() + port.offset(), port.size());
    temp.testWrite(port);
  }

  static std::string teststr = "test";

  for (auto &inst : _program.instructions()) {

#if 1
    for (size_t iarg = 0; iarg < inst.argumentCount(); iarg++) {
      auto &usage = temp.usage(inst.arg(iarg));
      if (inst.op()->arg(iarg).isInput()) {
        if (usage.type != inst.op()->arg(iarg).type()) {
          if (usage.type == typeid(void)) {
            throw std::runtime_error("read error, not initialized");
          } else {
            throw std::runtime_error("read error, type mismatch");
          }
        }
      }
      if (inst.op()->arg(iarg).isOutput()) {
        usage.type = inst.op()->arg(iarg).type();
      }
      /*if (inst.op()->arg(iarg).isOutput()) {
        std::memset((uint8_t *)temp.data() + inst.arg(iarg), 0,
                    inst.op()->arg(iarg).size());
      }*/
      if (inst.op()->arg(iarg).isOutput()) {
        for (size_t i = 0; i < inst.op()->arg(iarg).size(); i++) {
          ((uint8_t *)temp.data())[inst.arg(iarg) + i] =
              teststr[i % teststr.size()];
        }
      }
    }
#endif

    // std::cout << inst.op()->name() << std::endl;

    inst.op()->functions().indirect(temp.data(), &inst.arg(0));

#if 0
    if ( //! inst.op()->is<op_add>() && !inst.op()->is<op_sub>() &&
        inst.argumentCount() > 2) {
      for (size_t iarg = 0; iarg < inst.argumentCount(); iarg++) {
        if (inst.op()->arg(iarg).isOutput() &&
            inst.op()->arg(iarg).size() > 8) {
          bool eq = true;
          for (size_t i = 0; i < inst.op()->arg(iarg).size(); i++) {
            if (((uint8_t *)temp.data())[inst.arg(iarg) + i] !=
                teststr[i % teststr.size()]) {
              eq = false;
            }
          }
          if (eq) {

            std::cout << "output not modified " << std::to_string(iarg) << " "
                      << inst.op()->name() << " "
                      << std::to_string(inst.op()->arg(iarg).size())
                      << std::endl;

            /*{
              std::string s;
              std::cin >> s;
          }*/

            /*throw std::runtime_error(
                "output not modified " + std::to_string(iarg) + " " +
                inst.op()->name() + " " +
                std::to_string(inst.op()->arg(iarg).size()));*/
          }
        }
      }
    }
#endif
  }

  // std::cout << "test engine end" << std::endl;
}

} // namespace tractor
