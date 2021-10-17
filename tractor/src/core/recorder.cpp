// (c) 2020-2021 Philipp Ruppel

#include <tractor/core/recorder.h>

#include <tractor/core/operator.h>
#include <tractor/core/ops.h>

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace tractor {

static thread_local Recorder *g_recorder_instance = nullptr;

Recorder *Recorder::instance() { return g_recorder_instance; }

Recorder::Recorder(Program *program) : _program(program) {
  _const_data.resize(64, 0);
  //_memory_size = memory_alignment;
  _alloc.clear();
  if (g_recorder_instance) {
    throw std::runtime_error("already recording");
  }
  g_recorder_instance = this;
}

Recorder::~Recorder() {
  finish(*_program);
  if (g_recorder_instance != this) {
    throw std::runtime_error("recorder not active anymore");
  }
  g_recorder_instance = nullptr;
}

static void removeUnusedConstants(Program &program) {

  std::cout << "removing unused constants" << std::endl;

  std::unordered_set<uintptr_t> used;

  for (auto &inst : program.instructions()) {
    for (size_t iarg = 0; iarg < inst.argumentCount(); iarg++) {
      if (inst.op()->arg(iarg).isInput()) {
        used.insert(inst.arg(iarg));
      }
    }
  }

  for (auto &port : program.outputs()) {
    used.insert(port.address());
  }

  std::vector<Program::Constant> constants;
  for (auto &constant : program.constants()) {
    if (used.find(constant.address()) != used.end()) {
      constants.push_back(constant);
    }
  }
  program.setConstants(constants);
}

static void removeUnusedInstructions(Program &program) {

  std::cout << "removing unused instructions" << std::endl;

  std::vector<uint8_t> used(program.memorySize(), 0);

  for (auto &port : program.outputs()) {
    used[port.address()] = true;
  }

  size_t used_count = 0;

  std::vector<const Program::Instruction *> instructions;
  for (auto &inst : program.instructions()) {
    instructions.push_back(&inst);
  }
  std::reverse(instructions.begin(), instructions.end());

  std::vector<Program::Instruction> new_insts;

  for (auto *instp : instructions) {
    auto &inst = *instp;
    bool op_used = false;
    for (size_t iarg = 0; iarg < inst.argumentCount(); iarg++) {
      if (inst.op()->arg(iarg).isOutput()) {
        if (used[inst.arg(iarg)]) {
          op_used = true;
        }
      }
    }
    if (op_used) {
      used_count++;
      for (ssize_t iarg = inst.argumentCount() - 1; iarg >= 0; iarg--) {
        new_insts.push_back(inst.arg(iarg));
      }
      new_insts.push_back(inst.op());
      for (size_t iarg = 0; iarg < inst.argumentCount(); iarg++) {
        if (inst.op()->arg(iarg).isInput()) {
          used[inst.arg(iarg)] = true;
        }
      }
    }
  }

  program.setInstructions(new_insts.rbegin(), new_insts.rend());

  std::cout << instructions.size() << " ops" << std::endl;
  std::cout << used_count << " used (" << used_count * 100 / instructions.size()
            << "%)" << std::endl;
  std::cout << (instructions.size() - used_count) << " unused ("
            << (instructions.size() - used_count) * 100 / instructions.size()
            << "%)" << std::endl;
}

static void precomputeConstants(Program &program) {

  std::cout << "precomputing constants" << std::endl;

  AlignedStdVector<uint8_t> constness(program.memorySize(), 0);
  for (auto &port : program.constants()) {
    constness[port.address()] = true;
  }

  AlignedStdVector<uint8_t> memory(program.memorySize(), 0);
  for (auto &port : program.constants()) {
    std::memcpy(memory.data() + port.address(),
                program.constData().data() + port.offset(), port.size());
  }

  std::vector<Program::Instruction> new_insts;
  auto new_const_data = program.constData();

  // size_t new_memory_size = program.memorySize();
  Allocator alloc;
  alloc.keep(program);

  size_t const_move_count = 0;
  size_t const_op_count = 0;
  size_t op_count = 0;
  for (auto &inst : program.instructions()) {
    bool is_const = true;
    for (size_t iarg = 0; iarg < inst.argumentCount(); iarg++) {
      if (inst.op()->arg(iarg).isInput()) {
        if (!constness[inst.arg(iarg)]) {
          is_const = false;
        }
      }
    }
    for (size_t iarg = 0; iarg < inst.argumentCount(); iarg++) {
      if (inst.op()->arg(iarg).isOutput()) {
        constness[inst.arg(iarg)] = is_const;
      }
    }
    if (is_const) {
      if (inst.op()->is<op_move>()) {
        const_move_count++;
      } else {
        const_op_count++;
      }
      inst.op()->functions().indirect(memory.data(), &inst.arg(0));
      for (size_t iarg = 0; iarg < inst.argumentCount(); iarg++) {
        if (inst.op()->arg(iarg).isOutput()) {
          auto *move_op =
              Operator::find<compute, op_move>(inst.op()->arg(iarg).type());
          auto new_addr = alloc.alloc(inst.op()->arg(iarg).typeInfo());
          new_insts.push_back(move_op);
          new_insts.push_back(new_addr);
          new_insts.push_back(inst.arg(iarg));
          program.addConstant(Program::Constant(inst.op()->arg(iarg).typeInfo(),
                                                new_addr,
                                                new_const_data.size()));
          // new_memory_size += inst.op()->arg(iarg).size();
          for (size_t i = 0; i < inst.op()->arg(iarg).size(); i++) {
            new_const_data.push_back(memory[inst.arg(iarg) + i]);
          }
        }
      }
    } else {
      new_insts.push_back(inst.op());
      for (size_t iarg = 0; iarg < inst.argumentCount(); iarg++) {
        new_insts.push_back(inst.arg(iarg));
      }
    }
    op_count++;
  }

  program.setInstructions(new_insts);
  program.setConstData(new_const_data);
  // program.setMemorySize(new_memory_size);
  alloc.apply(program);

  std::cout << op_count << " ops" << std::endl;
  std::cout << const_move_count << " const move ("
            << const_move_count * 100 / op_count << "%)" << std::endl;
  std::cout << const_op_count << " const ops ("
            << const_op_count * 100 / op_count << "%)" << std::endl;
}

static void skipMoves(Program &program) {

  std::cout << "skipping redundant moves" << std::endl;

  std::unordered_map<size_t, size_t> move_dst_to_src;
  std::vector<Program::Instruction> new_instructions;
  size_t arg_count = 0;
  size_t rewrite_count = 0;
  for (auto &inst : program.instructions()) {
    new_instructions.push_back(inst.op());
    for (size_t iarg = 0; iarg < inst.argumentCount(); iarg++) {
      auto argc = inst.arg(iarg);
      if (inst.op()->arg(iarg).isOutput()) {
        move_dst_to_src.erase(inst.arg(iarg));
      }
      if (inst.op()->arg(iarg).isInput()) {
        auto it = move_dst_to_src.find(argc);
        if (it != move_dst_to_src.end()) {
          argc = it->second;
          rewrite_count++;
        }
      }
      arg_count++;
      new_instructions.push_back(argc);
    }
    if (inst.op()->is<op_move>()) {
      auto argc = inst.arg(0);
      {
        auto it = move_dst_to_src.find(argc);
        if (it != move_dst_to_src.end()) {
          argc = it->second;
        }
      }
      move_dst_to_src[inst.arg(1)] = argc;
    }
  }
  /*
  for (auto &port : program.outputs()) {
    auto &argc = port.address();
    auto it = move_dst_to_src.find(argc);
    if (it != move_dst_to_src.end()) {
      argc = it->second;
      rewrite_count++;
    }
  }
  */
  program.setInstructions(new_instructions.begin(), new_instructions.end());
  std::cout << rewrite_count << " moves / " << arg_count << " args skipped"
            << std::endl;
}

#if 0
static void checkMemory(const Program &program) {

  std::cout << "checking memory" << std::endl;

  std::unordered_set<uint64_t> memory;

  for (auto &port : program.inputs()) {
    // std::cout << "input " << port.address() << " " << port.size() <<
    // std::endl;
    for (size_t i = 0; i < port.size(); i++) {
      memory.insert(port.address() + i);
    }
  }

  for (auto &port : program.constants()) {
    // std::cout << "constant " << port.address() << " " << port.size()
    //          << std::endl;
    for (size_t i = 0; i < port.size(); i++) {
      memory.insert(port.address() + i);
    }
  }

  for (auto &port : program.parameters()) {
    // std::cout << "parameter " << port.address() << " " << port.size()
    //          << std::endl;
    for (size_t i = 0; i < port.size(); i++) {
      memory.insert(port.address() + i);
    }
  }

  for (auto &inst : program.instructions()) {
    for (size_t iarg = 0; iarg < inst.op()->argumentCount(); iarg++) {
      for (size_t i = 0; i < inst.op()->arg(iarg).size(); i++) {
        if (inst.op()->arg(iarg).isInput()) {
          if (memory.find(inst.arg(iarg) + i) == memory.end()) {
            for (auto &inst2 : program.instructions()) {
              std::cout << "op " << inst2.op()->name();
              for (size_t iarg = 0; iarg < inst2.op()->argumentCount();
                   iarg++) {
                std::cout << " " << inst2.arg(iarg) << ":"
                          << inst2.op()->arg(iarg).size();
              }
              std::cout << std::endl;
              if (&inst2 == &inst) {
                break;
              }
            }
            for (auto &port : program.inputs()) {
              std::cout << "input " << port.address() << " " << port.size()
                        << std::endl;
            }
            for (auto &port : program.constants()) {
              std::cout << "constant " << port.address() << " " << port.size()
                        << std::endl;
            }
            for (auto &port : program.parameters()) {
              std::cout << "parameter " << port.address() << " " << port.size()
                        << std::endl;
            }
            throw std::runtime_error(
                "read from uninitialized memory " + inst.op()->name() + " " +
                std::to_string(iarg) + " " + std::to_string(i) + " " +
                std::to_string(inst.op()->arg(iarg).size()) + " " +
                std::to_string(inst.arg(iarg)));
          }
        }
        if (inst.op()->arg(iarg).isOutput()) {
          memory.insert(inst.arg(iarg) + i);
        }
      }
    }
  }

  for (auto &port : program.outputs()) {
    for (size_t i = 0; i < port.size(); i++) {
      if (memory.find(port.address() + i) == memory.end()) {
        throw std::runtime_error("read from uninitialized memory");
      }
    }
  }
}
#endif

#if 0
static void checkMemory(const Program &program) {

  std::cout << "checking memory" << std::endl;

  std::unordered_set<uint64_t> memory;

  for (auto &port : program.inputs()) {
    memory.insert(port.address());
  }

  for (auto &port : program.constants()) {
    memory.insert(port.address());
  }

  for (auto &port : program.parameters()) {
    memory.insert(port.address());
  }

  for (auto &inst : program.instructions()) {
    for (size_t iarg = 0; iarg < inst.op()->argumentCount(); iarg++) {
      if (inst.op()->arg(iarg).isInput()) {
        if (memory.find(inst.arg(iarg)) == memory.end()) {
          for (auto &inst2 : program.instructions()) {
            std::cout << "op " << inst2.op()->name();
            for (size_t iarg = 0; iarg < inst2.op()->argumentCount(); iarg++) {
              std::cout << " " << inst2.arg(iarg) << ":"
                        << inst2.op()->arg(iarg).size();
            }
            std::cout << std::endl;
            if (&inst2 == &inst) {
              break;
            }
          }
          for (auto &port : program.inputs()) {
            std::cout << "input " << port.address() << " " << port.size()
                      << std::endl;
          }
          for (auto &port : program.constants()) {
            std::cout << "constant " << port.address() << " " << port.size()
                      << std::endl;
          }
          for (auto &port : program.parameters()) {
            std::cout << "parameter " << port.address() << " " << port.size()
                      << std::endl;
          }
          throw std::runtime_error("read from uninitialized memory " +
                                   inst.op()->name() + " " +
                                   std::to_string(iarg) + " " +
                                   std::to_string(inst.op()->arg(iarg).size()) +
                                   " " + std::to_string(inst.arg(iarg)));
        }
      }
      if (inst.op()->arg(iarg).isOutput()) {
        memory.insert(inst.arg(iarg));
      }
    }
  }

  for (auto &port : program.outputs()) {
    if (memory.find(port.address()) == memory.end()) {
      throw std::runtime_error("read from uninitialized memory");
    }
  }
}
#endif

#if 1
static void checkMemory(const Program &program) {

  std::cout << "checking memory" << std::endl;

  std::vector<uint8_t> memory;
  memory.resize(program.memorySize(), 0);

  for (auto &port : program.inputs()) {
    memory[port.address()] = 1;
  }

  for (auto &port : program.constants()) {
    memory[port.address()] = 1;
  }

  for (auto &port : program.parameters()) {
    memory[port.address()] = 1;
  }

  for (auto &inst : program.instructions()) {
    for (size_t iarg = 0; iarg < inst.op()->argumentCount(); iarg++) {
      if ((inst.arg(iarg) % inst.op()->arg(iarg).typeInfo().alignment()) != 0) {
        // throw std::runtime_error("memory check: alignment error");
        throw std::runtime_error("memory check: alignment error " +
                                 inst.op()->name() + " " +
                                 std::to_string(iarg) + " " +
                                 std::to_string(inst.op()->arg(iarg).size()) +
                                 " " + std::to_string(inst.arg(iarg)));
      }
      if (inst.op()->arg(iarg).isInput()) {
        if (!memory[inst.arg(iarg)]) {
          for (auto &inst2 : program.instructions()) {
            std::cout << "op " << inst2.op()->name();
            for (size_t iarg = 0; iarg < inst2.op()->argumentCount(); iarg++) {
              std::cout << " " << inst2.arg(iarg) << ":"
                        << inst2.op()->arg(iarg).size();
            }
            std::cout << std::endl;
            if (&inst2 == &inst) {
              break;
            }
          }
          for (auto &port : program.inputs()) {
            std::cout << "input " << port.address() << " " << port.size()
                      << std::endl;
          }
          for (auto &port : program.constants()) {
            std::cout << "constant " << port.address() << " " << port.size()
                      << std::endl;
          }
          for (auto &port : program.parameters()) {
            std::cout << "parameter " << port.address() << " " << port.size()
                      << std::endl;
          }
          throw std::runtime_error("read from uninitialized memory " +
                                   inst.op()->name() + " " +
                                   std::to_string(iarg) + " " +
                                   std::to_string(inst.op()->arg(iarg).size()) +
                                   " " + std::to_string(inst.arg(iarg)));
        }
      }
      if (inst.op()->arg(iarg).isOutput()) {
        memory[inst.arg(iarg)] = 1;
      }
    }
  }

  for (auto &port : program.outputs()) {
    if (!memory[port.address()]) {
      throw std::runtime_error("read from uninitialized memory");
    }
  }
}
#endif

void Recorder::finish(Program &program) {

  {
    program.clear();

    std::unordered_map<uintptr_t, uintptr_t> page_map;
    auto map = [&](uintptr_t a, const TypeInfo &type, bool alloc = false) {
      if (a & 0x8000000000000000ul) {
        return a & ~0x8000000000000000ul;
      }
      auto &addr = page_map[a];
      if (!addr || alloc) {
        // addr = _memory_size;
        //_memory_size += size;
        addr = _alloc.alloc(type);
      }
      return addr;
    };

    std::unordered_map<size_t, const void *> address_to_output;
    for (auto &rec_inst :
         ArrayRef<Program::Instruction,
                  Program::InstructionIterator<const Program::Instruction>>(
             _instructions)) {
      auto *op = rec_inst.op();
      for (size_t i = 0; i < op->argumentCount(); i++) {
        auto &rec_arg = rec_inst.arg(i);
        auto &op_arg = op->arg(i);
        if (op_arg.isOutput()) {
          address_to_output[rec_arg] = &rec_arg;
        }
      }
    }

    std::unordered_map<const void *, size_t> output_to_address;
    {
      size_t offset = 0;
      for (auto port : _inputs) {
        // auto addr = _memory_size;
        //_memory_size += port.size();
        auto addr = _alloc.alloc(port.typeInfo());
        output_to_address[address_to_output[port.address()]] = addr;
        port.address() = addr;
        port.offset() = offset;
        offset += port.size();
        program.addInput(port);
      }
    }
    {
      size_t offset = 0;
      for (auto port : _parameters) {
        // auto addr = _memory_size;
        //_memory_size += port.size();
        auto addr = _alloc.alloc(port.typeInfo());
        output_to_address[address_to_output[port.address()]] = addr;
        port.address() = addr;
        port.offset() = offset;
        offset += port.size();
        program.addParameter(port);
      }
    }

    std::vector<Program::Instruction> prog_insts;
    for (auto &rec_inst :
         ArrayRef<Program::Instruction,
                  Program::InstructionIterator<const Program::Instruction>>(
             _instructions)) {
      auto *op = rec_inst.op();
      prog_insts.emplace_back(rec_inst.code());
      for (size_t i = 0; i < op->argumentCount(); i++) {
        auto &rec_arg = rec_inst.arg(i);
        auto &op_arg = op->arg(i);

        auto prog_arg = map(rec_arg, op_arg.typeInfo(), op_arg.isOutput());
        prog_insts.emplace_back(prog_arg);

        {
          auto out_it = output_to_address.find(&rec_arg);
          if (op_arg.isOutput() && out_it != output_to_address.end()) {
            page_map[rec_arg] = out_it->second;
          }
        }
      }
    }

    {
      size_t offset = 0;
      for (auto port : _outputs) {
        port.address() = map(port.address(), port.typeInfo());
        port.offset() = offset;
        offset += port.size();
        program.addOutput(port);
      }
    }

    for (auto &goal : _goals) {
      program.addGoal(goal);
    }

    for (auto port : _constants) {
      program.addConstant(port);
    }

    program.setBoundData(_bound_data);
    program.setConstData(_const_data);
    program.setInstructions(prog_insts);
    // program.setMemorySize(_memory_size);
    _alloc.apply(program);
  }

  checkMemory(program);

  std::cout << "code size " << program.code().size() << std::endl;

  std::cout << "precompute constants" << std::endl;
  precomputeConstants(program);

  std::cout << "code size " << program.code().size() << std::endl;

  std::cout << "skip moves" << std::endl;
  skipMoves(program);

  std::cout << "code size " << program.code().size() << std::endl;

  std::cout << "remove unused instructions" << std::endl;
  removeUnusedInstructions(program);

  std::cout << "remove unused constants" << std::endl;
  removeUnusedConstants(program);

  std::cout << "code size " << program.code().size() << std::endl;

  checkMemory(program);
}

} // namespace tractor
