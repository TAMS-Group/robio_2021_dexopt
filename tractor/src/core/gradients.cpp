// (c) 2020-2021 Philipp Ruppel

#include <tractor/core/gradients.h>

#include <tractor/core/allocator.h>
#include <tractor/core/ops.h>

#include <algorithm>
#include <map>
#include <unordered_map>

namespace tractor {

template <class Ports> static void packPortOffsets(Ports &&ports) {
  size_t offset = 0;
  for (auto &port : ports) {
    port.offset() = offset;
    offset += port.size();
  }
}

void buildGradients(const Program &src, Program &prep, Program *_fprop,
                    Program *_bprop, Program *_hessian, Program *accumulate) {

  Program fprop_dummy, bprop_dummy;
  if (_hessian || accumulate) {
    if (!_fprop) {
      _fprop = &fprop_dummy;
    }
    if (!_bprop) {
      _bprop = &bprop_dummy;
    }
  }

  std::vector<ssize_t> prep_index;
  std::vector<Program::Instruction> prep_insts;
  {
    prep.clear();
    Allocator alloc;
    alloc.keep(src);
    size_t inst_index = 0;
    for (auto &inst : src.instructions()) {
      auto *prep_op = inst.op()->tryFindVariant<prepare>();
      if (prep_op) {
        // std::cout << prep_op->name() << std::endl;
        prep_index.push_back(prep_insts.size());
        prep_insts.emplace_back((uintptr_t)prep_op);
        for (size_t i = 0; i < inst.argumentCount(); i++) {
          prep_insts.emplace_back(inst.arg(i));
        }
        for (size_t i = inst.argumentCount(); i < prep_op->argumentCount();
             i++) {
          // prep_insts.emplace_back(memory_size);
          // memory_size += prep_op->arg(i).size();
          prep_insts.emplace_back(alloc.alloc(prep_op->arg(i).typeInfo()));
        }
      } else {
        prep_index.push_back(-1);
      }
      inst_index++;
    }
    // prep.setMemorySize(memory_size);
    alloc.apply(prep);
    prep.setInstructions(prep_insts.begin(), prep_insts.end());
  }

  if (_bprop) {

    auto &bprop = *_bprop;
    bprop.clear();

    for (auto port : src.inputs()) {
      port.binding() = 0;
      port.address() += prep.memorySize();
      port.typeInfo() = port.typeInfo().gradientType();
      bprop.addOutput(Program::Output(port));
    }
    packPortOffsets(bprop.outputs());

    for (auto port : src.outputs()) {
      port.binding() = 0;
      port.address() += prep.memorySize();
      port.typeInfo() = port.typeInfo().gradientType();
      bprop.addInput(Program::Input(port));
    }
    packPortOffsets(bprop.inputs());

    {
      std::vector<Program::Instruction> instructions;
      size_t inst_index = 0;
      for (auto &inst : src.instructions()) {
        auto *op = inst.op()->variant<reverse>();
        if (!op) {
          throw std::runtime_error("no reverse op " + inst.op()->name());
        }
        for (ssize_t i = inst.argumentCount() - 1; i >= 0; i--) {
          auto addr = inst.arg(i) + prep.memorySize();
          instructions.emplace_back(addr);
        }
        ssize_t prep_i = prep_index[inst_index];
        if (prep_i < 0) {
          for (ssize_t i = inst.argumentCount() - 1; i >= 0; i--) {
            instructions.emplace_back(inst.arg(i));
          }
        } else {
          auto &prep_inst = prep_insts[prep_i];
          for (ssize_t i = prep_inst.argumentCount() - 1;
               i >= inst.argumentCount(); i--) {
            instructions.emplace_back(prep_inst.arg(i));
          }
        }
        instructions.emplace_back((uintptr_t)op);
        inst_index++;
      }
      std::reverse(instructions.begin(), instructions.end());
      bprop.setInstructions(instructions.begin(), instructions.end());
      bprop.updateMemorySize(src.memorySize() + prep.memorySize());
    }

    {
      // size_t memory_size = bprop.memorySize();
      Allocator alloc;
      alloc.keep(bprop);
      std::vector<Program::Instruction> instructions;
      std::vector<Program::Instruction> sum_instructions;
      std::unordered_set<size_t> mem_set;
      for (auto &inst : bprop.instructions()) {
        auto *op = inst.op();
        instructions.emplace_back((uintptr_t)op);
        sum_instructions.clear();
        for (size_t i = 0; i < inst.argumentCount(); i++) {
          if (op->arg(i).isOutput() &&
              (mem_set.find(inst.arg(i)) != mem_set.end())) {

            auto alloc_a = alloc.alloc(inst.op()->arg(i).typeInfo());
            auto alloc_b = alloc.alloc(inst.op()->arg(i).typeInfo());

            {
              auto *move_op =
                  Operator::find<compute, op_move>(op->arg(i).type());
              sum_instructions.push_back((uintptr_t)move_op);
              sum_instructions.push_back(inst.arg(i));

              // sum_instructions.push_back(memory_size + op->arg(i).size());
              sum_instructions.push_back(alloc_b);
            }
            {
              auto *add_op = Operator::find<compute, op_add>(op->arg(i).type(),
                                                             op->arg(i).type());
              sum_instructions.push_back((uintptr_t)add_op);

              // sum_instructions.push_back(memory_size + op->arg(i).size());
              sum_instructions.push_back(alloc_b);

              // sum_instructions.push_back(memory_size);
              sum_instructions.push_back(alloc_a);

              sum_instructions.push_back(inst.arg(i));
            }

            // instructions.emplace_back(memory_size);
            instructions.emplace_back(alloc_a);

            // memory_size += op->arg(i).size() * 2;
          } else {
            instructions.emplace_back(inst.arg(i));
          }
          if (op->arg(i).isOutput()) {
            mem_set.insert(inst.arg(i));
          }
        }
        for (auto &code : sum_instructions) {
          instructions.push_back(code);
        }
      }
      // bprop.setMemorySize(memory_size);
      alloc.apply(bprop);
      bprop.setInstructions(instructions.begin(), instructions.end());
    }

    /*{
      size_t const_size = 0;
      std::unordered_set<size_t> input_set;
      for (auto &input : bprop.inputs()) {
        input_set.insert(input.address());
      }
      for (auto &inst : bprop.instructions()) {
        auto *op = inst.op();
        for (size_t i = 0; i < inst.argumentCount(); i++) {
          if (inst.arg(i) >= prep.memorySize() && op->arg(i).isInput()) {
            if (input_set.insert(inst.arg(i)).second) {
              const_size = std::max(const_size, op->arg(i).size());
              bprop.addConstant(
                  Program::Constant(op->arg(i).typeInfo(), inst.arg(i), 0));
            }
          }
        }
      }
      {
        std::vector<uint8_t> const_data(const_size, 0);
        bprop.setConstData(const_data);
      }
  }*/

    {
      std::vector<Program::Instruction> instructions;

      std::unordered_set<size_t> input_set;
      for (auto &input : bprop.inputs()) {
        input_set.insert(input.address());
      }
      for (auto &inst : bprop.instructions()) {
        auto *op = inst.op();
        for (size_t i = 0; i < inst.argumentCount(); i++) {
          if (inst.arg(i) >= prep.memorySize()) {
            if (input_set.insert(inst.arg(i)).second) {
              if (op->arg(i).isInput()) {
                auto *zero_op =
                    Operator::find<compute, op_zero>(op->arg(i).type());
                instructions.emplace_back(zero_op);
                instructions.emplace_back(inst.arg(i));
              }
            }
          }
        }
      }

      for (auto &code : bprop.code()) {
        instructions.emplace_back(code);
      }

      bprop.setInstructions(instructions.begin(), instructions.end());
    }
  }

  if (_fprop) {
    auto &fprop = *_fprop;
    fprop.clear();

    for (auto port : src.inputs()) {
      port.binding() = 0;
      port.address() += prep.memorySize();
      port.typeInfo() = port.typeInfo().gradientType();
      fprop.addInput(port);
    }
    packPortOffsets(fprop.inputs());

    for (auto port : src.outputs()) {
      port.binding() = 0;
      port.address() += prep.memorySize();
      port.typeInfo() = port.typeInfo().gradientType();
      fprop.addOutput(port);
    }
    packPortOffsets(fprop.outputs());

    if (0) {
      size_t const_size = 0;
      for (auto port : src.constants()) {
        const_size = std::max(const_size, port.size());
        port.offset() = 0;
        port.address() += prep.memorySize();
        port.typeInfo() = port.typeInfo().gradientType();
        fprop.addConstant(port);
      }
      for (auto port : src.parameters()) {
        const_size = std::max(const_size, port.size());
        fprop.addConstant(Program::Constant(port.typeInfo().gradientType(),
                                            port.address() + prep.memorySize(),
                                            0));
      }
      fprop.setConstData(std::vector<uint8_t>(const_size, 0));
    }

    fprop.updateMemorySize(src.memorySize() + prep.memorySize());

    if (1) {
      for (auto &port : src.constants()) {
        auto *zero_op = Operator::find<compute, op_zero>(
            port.typeInfo().gradientType().type());
        fprop.addCode(zero_op);
        fprop.addCode(port.address() + prep.memorySize());
      }
      for (auto &port : src.parameters()) {
        auto *zero_op = Operator::find<compute, op_zero>(
            port.typeInfo().gradientType().type());
        fprop.addCode(zero_op);
        fprop.addCode(port.address() + prep.memorySize());
      }
    }

    size_t inst_index = 0;
    for (auto &inst : src.instructions()) {
      auto *op = inst.op()->variant<forward>();
      if (!op) {
        throw std::runtime_error("no forward op " + inst.op()->name());
      }
      fprop.addCode((uintptr_t)op);
      ssize_t prep_i = prep_index[inst_index];
      if (prep_i < 0) {
        for (size_t i = 0; i < inst.argumentCount(); i++) {
          fprop.addCode(inst.arg(i));
        }
      } else {
        auto &prep_inst = prep_insts[prep_i];
        for (size_t i = inst.argumentCount(); i < prep_inst.argumentCount();
             i++) {
          fprop.addCode(prep_inst.arg(i));
        }
      }
      for (ssize_t i = 0; i < inst.argumentCount(); i++) {
        auto addr = inst.arg(i) + prep.memorySize();
        fprop.addCode(addr);
      }
      inst_index++;
    }
  }

  /*
  if (_hessian) {
    auto &hprop = *_hessian;
    const auto &fprop = *_fprop;
    const auto &bprop = *_bprop;
    hprop.clear();
    for (auto &port : fprop.inputs()) {
      hprop.addInput(port);
    }
    for (auto &port : bprop.outputs()) {
      hprop.addOutput(port);
    }
    for (auto &port : bprop.constants()) {
      hprop.addConstant(port);
    }
    hprop.setConstData(bprop.constData());
    for (auto &inst : fprop.code()) {
      hprop.addCode(inst);
    }
    for (auto &inst : bprop.code()) {
      hprop.addCode(inst);
    }
    hprop.setMemorySize(std::max(bprop.memorySize(), fprop.memorySize()));
  }
  */

  if (_hessian) {
    auto &hprop = *_hessian;
    hprop.clear();
    const auto &fprop = *_fprop;
    const auto &bprop = *_bprop;
    for (auto &port : fprop.inputs()) {
      hprop.addInput(port);
    }
    for (auto &port : bprop.outputs()) {
      hprop.addOutput(port);
    }
    for (auto &inst : fprop.code()) {
      hprop.addCode(inst);
    }
    for (auto &inst : bprop.code()) {
      hprop.addCode(inst);
    }
    hprop.updateMemorySize(std::max(bprop.memorySize(), fprop.memorySize()));
  }

  if (accumulate) {
    const auto &fprop = *_fprop;
    auto &accu = *accumulate;
    accu.clear();
    // size_t memory_size = 0;
    Allocator alloc;
    std::vector<Program::Input> aa;
    std::vector<Program::Input> bb;
    std::vector<Program::Output> xx;
    for (auto port : src.inputs()) {
      // port.address() = memory_size;
      // memory_size += port.size();
      port.address() = alloc.alloc(port.typeInfo());
      aa.push_back(port);
      accu.addInput(port);
    }
    for (auto port : fprop.inputs()) {
      // port.address() = memory_size;
      // memory_size += port.size();
      port.address() = alloc.alloc(port.typeInfo());
      bb.push_back(port);
      accu.addInput(port);
    }
    packPortOffsets(accu.inputs());
    for (auto port : src.inputs()) {
      // port.address() = memory_size;
      // memory_size += port.size();
      port.address() = alloc.alloc(port.typeInfo());
      xx.push_back(Program::Output(port));
      accu.addOutput(Program::Output(port));
    }
    packPortOffsets(accu.outputs());
    for (size_t i = 0; i < aa.size(); i++) {
      auto *add_op =
          Operator::find<compute, op_add>(aa[i].type(), bb[i].type());
      accu.addCode(add_op);
      accu.addCode(aa[i].address());
      accu.addCode(bb[i].address());
      accu.addCode(xx[i].address());
    }
    // accu.setMemorySize(memory_size);
    alloc.apply(accu);
  }
}

#if 0
void buildDual(const Program &prog, const Program &fprop, const Program &bprop,
               const TypeInfo &parameter_type, Program &gradients,
               Program &residual) {

  {
    residual = bprop;

    size_t offset = 0;
    for (auto &port : residual.outputs()) {
      offset = std::max(offset, port.offset() + port.size());
    }

    for (auto &goal : prog.goals()) {
      if (goal.priority() >= 1) {
        auto &input = residual.input(goal.port());
        residual.addOutput(
            Program::Output(input.typeInfo(), input.address(), offset, 0));
        offset += input.size();
      }
    }
  }

  {
    gradients.clear();
    size_t memory_size = std::max(bprop.memorySize(), fprop.memorySize());
    size_t port_offset = 0;

    for (auto &port : fprop.inputs()) {
      gradients.addInput(port);
      port_offset = std::max(port_offset, port.offset() + port.size());
    }

    for (auto &port : bprop.outputs()) {
      gradients.addOutput(port);
    }

    for (auto &inst : fprop.code()) {
      gradients.addCode(inst);
    }

    gradients.addParameter(
        Program::Parameter(parameter_type, memory_size, 0, 0));
    auto object_weight_address = memory_size;
    memory_size += parameter_type.size();

    for (auto &goal : prog.goals()) {

      auto fprop_output = fprop.output(goal.port());

      if (goal.priority() >= 1) {

        // lagrange multiplier input
        auto lagrange_multiplier_input = Program::Input(
            fprop_output.typeInfo(), memory_size, port_offset, 0);
        lagrange_multiplier_input.name() = "lagrange multiplier";
        gradients.addInput(lagrange_multiplier_input);
        memory_size += lagrange_multiplier_input.size();

        // constraint output
        {
          auto *move_op = Operator::find<compute, op_move>(fprop_output.type());
          gradients.addCode(move_op);
          gradients.addCode(fprop_output.address());
          gradients.addCode(memory_size);

          auto constraint_output = fprop_output;
          constraint_output.address() = memory_size;
          constraint_output.offset() = port_offset;
          gradients.addOutput(constraint_output);

          memory_size += fprop_output.size();
        }

        // add lagrange multipliers
        /*{
          auto *move_op = Operator::find<compute, op_move>(fprop_output.type());
          gradients.addCode(move_op);
          gradients.addCode(fprop_output.address());
          gradients.addCode(memory_size);
        }
        */
        {
          auto *mul_op = Operator::find<compute, op_mul>(fprop_output.type(),
                                                         parameter_type.type());
          gradients.addCode(mul_op);
          gradients.addCode(fprop_output.address());
          gradients.addCode(object_weight_address);
          gradients.addCode(memory_size);
        }
        {
          auto *add_op = Operator::find<compute, op_add>(fprop_output.type(),
                                                         fprop_output.type());
          gradients.addCode(add_op);
          gradients.addCode(memory_size);
          gradients.addCode(lagrange_multiplier_input.address());
          gradients.addCode(fprop_output.address());
        }
        memory_size += fprop_output.size();

        // alloc space
        port_offset += fprop_output.size();

      } else {

        // scale objectives
        {
          auto *mul_op = Operator::find<compute, op_mul>(fprop_output.type(),
                                                         parameter_type.type());
          gradients.addCode(mul_op);
          gradients.addCode(fprop_output.address());
          gradients.addCode(object_weight_address);
          gradients.addCode(memory_size);
        }
        {
          auto *move_op = Operator::find<compute, op_move>(fprop_output.type());
          gradients.addCode(move_op);
          gradients.addCode(memory_size);
          gradients.addCode(fprop_output.address());
        }

        // alloc space
        memory_size += fprop_output.size();
      }
    }

    for (auto &inst : bprop.code()) {
      gradients.addCode(inst);
    }

    gradients.setMemorySize(memory_size);
  }
}
#endif

#if 0
void buildConstraints(const Program &prog, const Program &fprop,
                      const Program &bprop, const Program &hprop,
                      const TypeInfo &padding_type, Program *proj,
                      Program *barrier_init, Program *barrier_step,
                      Program *barrier_diagonal, Program *penalty_init,
                      Program *penalty_step, Program *penalty_diagonal) {

  proj->clear();

  barrier_init->clear();
  barrier_step->clear();
  barrier_diagonal->clear();

  penalty_init->clear();
  penalty_step->clear();
  penalty_diagonal->clear();

  size_t memory_size =
      std::max(prog.memorySize(),
               std::max(std::max(fprop.memorySize(), bprop.memorySize()),
                        hprop.memorySize()));

  struct PortInfo {
    TypeInfo type;
    uintptr_t input = 0;
    uintptr_t output = 0;
    PortInfo() {}
    PortInfo(TypeInfo type, uintptr_t input, uintptr_t output)
        : type(type), input(input), output(output) {}
  };

  std::unordered_map<uintptr_t, PortInfo> port_map;

  {
    auto it_nonlinear = prog.inputs().begin();
    auto it_linear = fprop.inputs().begin();
    while (it_nonlinear != prog.inputs().end()) {

      proj->addInput(*it_linear);

      barrier_init->addInput(*it_linear);
      barrier_step->addInput(*it_linear);

      penalty_init->addInput(*it_linear);
      penalty_step->addInput(*it_linear);

      uintptr_t out_addr = memory_size;
      memory_size += it_linear->size();
      port_map[it_nonlinear->address()] =
          PortInfo(it_linear->typeInfo(), it_linear->address(), out_addr);
      auto output = Program::Output(it_linear->typeInfo(), out_addr,
                                    it_linear->offset(), 0);
      proj->addOutput(output);

      barrier_init->addOutput(output);
      barrier_step->addOutput(output);
      barrier_diagonal->addOutput(output);

      penalty_init->addOutput(output);
      penalty_step->addOutput(output);
      penalty_diagonal->addOutput(output);

      ++it_nonlinear;
      ++it_linear;
    }
  }

  auto padding_addr = memory_size;
  memory_size += padding_type.size();
  proj->addParameter(Program::Parameter(padding_type, padding_addr, 0, 0));
  penalty_init->addParameter(
      Program::Parameter(padding_type, padding_addr, 0, 0));

  for (auto &inst : prog.instructions()) {

    auto *op_proj = inst.op()->tryFindVariant<project>();

    auto *op_barrier_init = inst.op()->tryFindVariant<tractor::barrier_init>();
    auto *op_barrier_step = inst.op()->tryFindVariant<tractor::barrier_step>();
    auto *op_barrier_diagonal =
        inst.op()->tryFindVariant<tractor::barrier_diagonal>();

    auto *op_penalty_init = inst.op()->tryFindVariant<tractor::penalty_init>();
    auto *op_penalty_step = inst.op()->tryFindVariant<tractor::penalty_step>();
    auto *op_penalty_diagonal =
        inst.op()->tryFindVariant<tractor::penalty_diagonal>();

    if (!op_proj && !op_barrier_init && !op_barrier_step &&
        !op_barrier_diagonal && !op_penalty_init && !op_penalty_step &&
        !op_penalty_diagonal) {
      continue;
    }

    if (!op_proj || !op_barrier_init || !op_barrier_step ||
        !op_barrier_diagonal || !op_penalty_init || !op_penalty_step ||
        !op_penalty_diagonal) {
      throw std::runtime_error("incomplete constraint op " + inst.op()->name());
    }

    auto it_port_info = port_map.find(inst.arg(0));
    if (it_port_info == port_map.end()) {
      throw std::runtime_error("add slack variable");
    }
    auto &port_info = it_port_info->second;

    {
      proj->addCode(op_proj);
      for (size_t i = 0; i < inst.argumentCount(); i++) {
        if (inst.op()->arg(i).isInput()) {
          proj->addCode(inst.arg(i));
        }
      }
      proj->addCode(port_info.input);
      proj->addCode(padding_addr);
      proj->addCode(port_info.output);
    }

    auto temp_addr = memory_size;
    memory_size += std::max(
        op_barrier_init->arg(op_barrier_init->argumentCount() - 1).size(),
        op_penalty_init->arg(op_penalty_init->argumentCount() - 1).size());

    {
      barrier_init->addCode(op_barrier_init);
      for (size_t i = 0; i < inst.argumentCount(); i++) {
        if (inst.op()->arg(i).isInput()) {
          barrier_init->addCode(inst.arg(i));
        }
      }
      barrier_init->addCode(port_info.input);
      barrier_init->addCode(port_info.output);
      barrier_init->addCode(temp_addr);

      barrier_step->addCode(op_barrier_step);
      barrier_step->addCode(temp_addr);
      barrier_step->addCode(port_info.input);
      barrier_step->addCode(port_info.output);

      barrier_diagonal->addCode(op_barrier_diagonal);
      barrier_diagonal->addCode(temp_addr);
      barrier_diagonal->addCode(port_info.output);
    }

    {
      penalty_init->addCode(op_penalty_init);
      for (size_t i = 0; i < inst.argumentCount(); i++) {
        if (inst.op()->arg(i).isInput()) {
          penalty_init->addCode(inst.arg(i));
        }
      }
      penalty_init->addCode(port_info.input);
      penalty_init->addCode(padding_addr);
      penalty_init->addCode(port_info.output);
      penalty_init->addCode(temp_addr);

      penalty_step->addCode(op_penalty_step);
      penalty_step->addCode(temp_addr);
      penalty_step->addCode(port_info.input);
      penalty_step->addCode(port_info.output);

      penalty_diagonal->addCode(op_penalty_diagonal);
      penalty_diagonal->addCode(temp_addr);
      penalty_diagonal->addCode(port_info.output);
    }

    port_map.erase(it_port_info);
  }

  for (auto &port_info_pair : port_map) {
    auto &port_info = port_info_pair.second;

    {
      auto *move_op = Operator::find<compute, op_move>(port_info.type.type());
      proj->addCode(move_op);
      proj->addCode(port_info.input);
      proj->addCode(port_info.output);
    }

    {
      auto *zero_op = Operator::find<compute, op_zero>(port_info.type.type());

      {
        barrier_init->addCode(zero_op);
        barrier_init->addCode(port_info.output);

        barrier_step->addCode(zero_op);
        barrier_step->addCode(port_info.output);

        barrier_diagonal->addCode(zero_op);
        barrier_diagonal->addCode(port_info.output);
      }

      {
        penalty_init->addCode(zero_op);
        penalty_init->addCode(port_info.output);

        penalty_step->addCode(zero_op);
        penalty_step->addCode(port_info.output);

        penalty_diagonal->addCode(zero_op);
        penalty_diagonal->addCode(port_info.output);
      }
    }
  }

  proj->setMemorySize(memory_size);

  barrier_init->setMemorySize(memory_size);
  barrier_step->setMemorySize(memory_size);
  barrier_diagonal->setMemorySize(memory_size);

  penalty_init->setMemorySize(memory_size);
  penalty_step->setMemorySize(memory_size);
  penalty_diagonal->setMemorySize(memory_size);
}
#endif

#if 1

void buildConstraints(const Program &prog, const Program &fprop,
                      const Program &bprop, const Program &hprop,
                      const TypeInfo &padding_type, Program *proj,
                      Program *barrier_init, Program *barrier_step,
                      Program *barrier_diagonal, Program *penalty_init,
                      Program *penalty_step, Program *penalty_diagonal) {

  proj->clear();

  barrier_init->clear();
  barrier_step->clear();
  barrier_diagonal->clear();

  penalty_init->clear();
  penalty_step->clear();
  penalty_diagonal->clear();

  Allocator alloc;
  alloc.init(std::max(prog.memorySize(),
                      std::max(std::max(fprop.memorySize(), bprop.memorySize()),
                               hprop.memorySize())));

  struct PortInfo {
    TypeInfo type;
    uintptr_t input = 0;
    uintptr_t output = 0;
    PortInfo() {}
    PortInfo(TypeInfo type, uintptr_t input, uintptr_t output)
        : type(type), input(input), output(output) {}
  };

  std::unordered_map<uintptr_t, PortInfo> port_map;

  {
    auto it_nonlinear = prog.inputs().begin();
    auto it_linear = fprop.inputs().begin();
    while (it_nonlinear != prog.inputs().end()) {

      proj->addInput(*it_linear);

      barrier_init->addInput(*it_linear);
      barrier_step->addInput(*it_linear);

      // penalty_init->addInput(*it_linear);
      // penalty_step->addInput(*it_linear);

      // uintptr_t out_addr = memory_size;
      // memory_size += it_linear->size();
      uintptr_t out_addr = alloc.alloc(it_linear->typeInfo());

      port_map[it_nonlinear->address()] =
          PortInfo(it_linear->typeInfo(), it_linear->address(), out_addr);
      auto output = Program::Output(it_linear->typeInfo(), out_addr,
                                    it_linear->offset(), 0);
      proj->addOutput(output);

      barrier_init->addOutput(output);
      barrier_step->addOutput(output);
      barrier_diagonal->addOutput(output);

      // penalty_init->addOutput(output);
      // penalty_step->addOutput(output);
      // penalty_diagonal->addOutput(output);

      ++it_nonlinear;
      ++it_linear;
    }
  }

  // auto padding_addr = memory_size;
  // memory_size += padding_type.size();
  auto padding_addr = alloc.alloc(padding_type);

  proj->addParameter(Program::Parameter(padding_type, padding_addr, 0, 0));
  // penalty_init->addParameter(
  //      Program::Parameter(padding_type, padding_addr, 0, 0));

  for (auto &inst : prog.instructions()) {

    auto *op_proj = inst.op()->tryFindVariant<project>();

    auto *op_barrier_init = inst.op()->tryFindVariant<tractor::barrier_init>();
    auto *op_barrier_step = inst.op()->tryFindVariant<tractor::barrier_step>();
    auto *op_barrier_diagonal =
        inst.op()->tryFindVariant<tractor::barrier_diagonal>();

    /*auto *op_penalty_init =
    inst.op()->tryFindVariant<tractor::penalty_init>(); auto *op_penalty_step =
    inst.op()->tryFindVariant<tractor::penalty_step>(); auto
    *op_penalty_diagonal =
        inst.op()->tryFindVariant<tractor::penalty_diagonal>();*/

    if (!op_proj && !op_barrier_init && !op_barrier_step &&
        !op_barrier_diagonal /*&& !op_penalty_init && !op_penalty_step &&
        !op_penalty_diagonal*/) {
      continue;
    }

    if (!op_proj || !op_barrier_init || !op_barrier_step ||
        !op_barrier_diagonal /*|| !op_penalty_init || !op_penalty_step ||
        !op_penalty_diagonal*/) {
      throw std::runtime_error("incomplete constraint op " + inst.op()->name());
    }

    auto it_port_info = port_map.find(inst.arg(0));
    if (it_port_info == port_map.end()) {
      throw std::runtime_error("add slack variable");
    }
    auto &port_info = it_port_info->second;

    {
      proj->addCode(op_proj);
      for (size_t i = 0; i < inst.argumentCount(); i++) {
        if (inst.op()->arg(i).isInput()) {
          proj->addCode(inst.arg(i));
        }
      }
      proj->addCode(port_info.input);
      proj->addCode(padding_addr);
      proj->addCode(port_info.output);
    }

    // auto temp_addr = memory_size;
    // memory_size +=
    //    op_barrier_init->arg(op_barrier_init->argumentCount() - 1).size();
    auto temp_addr = alloc.alloc(
        op_barrier_init->arg(op_barrier_init->argumentCount() - 1).typeInfo());

    {
      barrier_init->addCode(op_barrier_init);
      for (size_t i = 0; i < inst.argumentCount(); i++) {
        if (inst.op()->arg(i).isInput()) {
          barrier_init->addCode(inst.arg(i));
        }
      }
      barrier_init->addCode(port_info.input);
      barrier_init->addCode(port_info.output);
      barrier_init->addCode(temp_addr);

      barrier_step->addCode(op_barrier_step);
      barrier_step->addCode(temp_addr);
      barrier_step->addCode(port_info.input);
      barrier_step->addCode(port_info.output);

      barrier_diagonal->addCode(op_barrier_diagonal);
      barrier_diagonal->addCode(temp_addr);
      barrier_diagonal->addCode(port_info.output);
    }

    /*
    {
      penalty_init->addCode(op_penalty_init);
      for (size_t i = 0; i < inst.argumentCount(); i++) {
        if (inst.op()->arg(i).isInput()) {
          penalty_init->addCode(inst.arg(i));
        }
      }
      penalty_init->addCode(port_info.input);
      penalty_init->addCode(padding_addr);
      penalty_init->addCode(port_info.output);
      penalty_init->addCode(temp_addr);

      penalty_step->addCode(op_penalty_step);
      penalty_step->addCode(temp_addr);
      penalty_step->addCode(port_info.input);
      penalty_step->addCode(port_info.output);

      penalty_diagonal->addCode(op_penalty_diagonal);
      penalty_diagonal->addCode(temp_addr);
      penalty_diagonal->addCode(port_info.output);
    }
    */

    port_map.erase(it_port_info);
  }

  for (auto &port_info_pair : port_map) {
    auto &port_info = port_info_pair.second;

    {
      auto *move_op = Operator::find<compute, op_move>(port_info.type.type());
      proj->addCode(move_op);
      proj->addCode(port_info.input);
      proj->addCode(port_info.output);
    }

    {
      auto *zero_op = Operator::find<compute, op_zero>(port_info.type.type());

      {
        barrier_init->addCode(zero_op);
        barrier_init->addCode(port_info.output);

        barrier_step->addCode(zero_op);
        barrier_step->addCode(port_info.output);

        barrier_diagonal->addCode(zero_op);
        barrier_diagonal->addCode(port_info.output);
      }

      /*
      {
        penalty_init->addCode(zero_op);
        penalty_init->addCode(port_info.output);

        penalty_step->addCode(zero_op);
        penalty_step->addCode(port_info.output);

        penalty_diagonal->addCode(zero_op);
        penalty_diagonal->addCode(port_info.output);
      }
      */
    }
  }

  /*
  proj->setMemorySize(memory_size);

  barrier_init->setMemorySize(memory_size);
  barrier_step->setMemorySize(memory_size);
  barrier_diagonal->setMemorySize(memory_size);
  */

  alloc.apply(*proj);
  alloc.apply(*barrier_init);
  alloc.apply(*barrier_step);
  alloc.apply(*barrier_diagonal);

  /*
  penalty_init->setMemorySize(memory_size);
  penalty_step->setMemorySize(memory_size);
  penalty_diagonal->setMemorySize(memory_size);
  */
}

#endif

} // namespace tractor
