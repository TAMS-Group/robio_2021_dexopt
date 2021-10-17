// (c) 2020-2021 Philipp Ruppel

#include <tractor/core/program.h>

#include <tractor/core/operator.h>
#include <tractor/core/recorder.h>

namespace tractor {

size_t Program::Instruction::argumentCount() const {
  return op()->argumentCount();
}

template <class T>
void printBuffer(std::ostream &stream, const char *label, const T &data) {
  std::string str((char *)&*data.begin(), (char *)&*data.end());
  stream << label << " " << data.size() << " " << std::hash<std::string>()(str)
         << std::endl;
}

template <class T>
void printPorts(std::ostream &stream, const char *label, const T &data) {
  printBuffer(stream, label, data);
  for (auto &port : data) {
    stream << "  - " << (const void *)port.address() << " " << port.name()
           << std::endl;
  }
}

void Program::record(const std::function<void()> &function) {
  Recorder rec(this);
  function();
}

std::ostream &operator<<(std::ostream &stream, const Program &prog) {

  printPorts(stream, "inputs", prog.inputs());
  printPorts(stream, "parameters", prog.parameters());
  printPorts(stream, "outputs", prog.outputs());
  printBuffer(stream, "goals", prog.goals());
  printPorts(stream, "constants", prog.constants());
  printBuffer(stream, "constdata", prog.constData());
  printBuffer(stream, "code", prog.code());

  /*
  for (auto &port : prog.inputs()) {
    stream << "input " << (void *)port.address() << " "
           << (void *)port.binding() << " " << port.size() << " "
           << port.type().name() << std::endl;
  }

  for (auto &port : prog.parameters()) {
    stream << "parameter " << (void *)port.address() << " "
           << (void *)port.offset() << " " << (void *)port.binding() << " "
           << port.size() << " " << port.type().name() << std::endl;
  }

  for (auto &port : prog.outputs()) {
    stream << "output " << (void *)port.address() << " "
           << (void *)port.offset() << " " << (void *)port.binding() << " "
           << port.size() << " " << port.type().name() << std::endl;
  }

  for (auto &goal : prog.goals()) {
    stream << "goal " << goal.port() << " " << goal.priority() << std::endl;
  }

  for (auto &port : prog.constants()) {
    stream << "constant " << (void *)port.address() << " " << port.size() << " "
           << port.type().name() << std::endl;
  }

  stream << "memory size " << prog.memorySize() << std::endl;
  */

  for (auto &inst : prog.instructions()) {
    stream << "instruction ";
    stream << inst.op()->name();
    for (auto arg : inst.args()) {
      stream << " " << (void *)arg;
    }
    stream << std::endl;
  }

  return stream;
}

} // namespace tractor
