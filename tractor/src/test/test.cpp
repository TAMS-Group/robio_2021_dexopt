// (c) 2020-2021 Philipp Ruppel

#include <tractor/test/test.h>

#include <tractor/core/eigen.h>
#include <tractor/core/engine.h>
#include <tractor/core/gradients.h>
#include <tractor/engines/simple.h>
#include <tractor/geometry/fast.h>

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace tractor {

void testScalarGradients() {

  // std::cout << "test scalar consistency" << std::endl;

  typedef double Scalar;

  auto all_operators = Operator::all();
  std::sort(all_operators.begin(), all_operators.end(),
            [](const Operator *a, const Operator *b) {
              return a->name() < b->name();
            });

  auto engine = std::make_shared<SimpleEngine>();
  auto memory = engine->createMemory();
  auto x_prog = engine->createExecutable();
  auto x_prep = engine->createExecutable();
  auto x_fprop = engine->createExecutable();
  auto x_bprop = engine->createExecutable();

  for (auto *op : all_operators) {
    if (!op->isMode<compute>()) {
      continue;
    }
    if (op->name().size() < 2 ||
        op->name().substr(op->name().size() - 2) != "_d") {
      continue;
    }
    if (!op->tryFindVariant<forward>()) {
      continue;
    }
    if (!op->tryFindVariant<reverse>()) {
      continue;
    }

    {
      bool is_scalar = true;
      for (auto &arg : op->arguments()) {
        if (arg.size() != sizeof(Scalar)) {
          is_scalar = false;
        }
      }
      if (!is_scalar) {
        continue;
      }
    }

    // std::cout << op->name() << " ";
    // std::cout.flush();

    Program p_prog;
    Program p_prep;
    Program p_fprop;
    Program p_bprop;

    std::vector<Program::Instruction> instructions;
    size_t memory_size = 0;
    instructions.push_back((uintptr_t)op);
    for (auto &arg : op->arguments()) {
      instructions.push_back(memory_size);
      if (arg.isInput()) {
        p_prog.addInput(
            Program::Input(arg.typeInfo(), memory_size, memory_size, 0));
      }
      if (arg.isOutput()) {
        p_prog.addOutput(
            Program::Output(arg.typeInfo(), memory_size, memory_size, 0));
      }
      memory_size += arg.size();
    }
    p_prog.updateMemorySize(memory_size);
    p_prog.setInstructions(instructions);

    buildGradients(p_prog, p_prep, &p_fprop, &p_bprop);

    x_prog->compile(p_prog);
    x_prep->compile(p_prep);
    x_fprop->compile(p_fprop);
    x_bprop->compile(p_bprop);

    Scalar delta = 1e-8;
    Scalar tolerance = 1e-5;

    Buffer l_p, l_a, l_b, l_g;
    Buffer r_p, r_a, r_b, r_g;
    l_p.resize(p_prog.memorySize());
    l_a.resize(p_prog.memorySize());
    l_b.resize(p_prog.memorySize());
    l_g.resize(p_prog.memorySize());

    size_t iteration_count = 20;
    size_t forward_success = 0;
    size_t reverse_success = 0;
    for (size_t iteration = 0; iteration < iteration_count; iteration++) {
      for (auto &port : p_prog.inputs()) {
        Scalar p = rand() * 2.0 / RAND_MAX - 1.0;
        Scalar g = rand() * 2.0 / RAND_MAX - 1.0;
        l_p.at<Scalar>(port) = p;
        l_g.at<Scalar>(port) = g;
        l_a.at<Scalar>(port) = p - g * delta * 0.5;
        l_b.at<Scalar>(port) = p + g * delta * 0.5;
      }

      x_prog->input(l_a, memory);
      x_prog->execute(memory);
      x_prog->output(memory, r_a);

      x_prog->input(l_b, memory);
      x_prog->execute(memory);
      x_prog->output(memory, r_b);

      x_prog->input(l_p, memory);
      x_prog->execute(memory);
      x_prog->output(memory, r_p);

      x_prep->execute(memory);

      x_fprop->input(l_g, memory);
      x_fprop->execute(memory);
      x_fprop->output(memory, r_g);

      {
        bool ok = true;
        auto it_p_port = p_prog.outputs().begin();
        auto it_g_port = p_fprop.outputs().begin();
        for (size_t iport = 0; iport < p_prog.outputs().size(); iport++) {
          auto &p_port = *(it_p_port++);
          auto &g_port = *(it_g_port++);
          Scalar ngrad =
              (r_b.at<Scalar>(p_port) - r_a.at<Scalar>(p_port)) / delta;
          Scalar pgrad = r_g.at<Scalar>(g_port);
          // std::cout << op->name() << " " << ngrad << " " << pgrad <<
          // std::endl;
          if (std::isfinite(r_p.at<Scalar>(p_port)) &&
              !(std::abs(ngrad - pgrad) < tolerance)) {
            ok = false;
          }
        }
        if (ok) {
          forward_success++;
        }
      }

      if (p_prog.inputs().empty() || p_prog.outputs().empty()) {
        continue;
      }

      size_t l_i = rand() % p_prog.inputs().size();
      size_t r_i = rand() % p_prog.outputs().size();

      Scalar g = rand() * 2.0 / RAND_MAX - 1.0;

      l_g.zero(p_prog.memorySize());
      r_g.zero(p_prog.memorySize());
      l_g.at<Scalar>(p_fprop.input(l_i)) = g;
      x_fprop->input(l_g, memory);
      x_fprop->execute(memory);
      x_fprop->output(memory, r_g);
      Scalar g_fprop = r_g.at<Scalar>(p_fprop.output(r_i));

      l_g.zero(p_prog.memorySize());
      r_g.zero(p_prog.memorySize());
      r_g.at<Scalar>(p_bprop.input(r_i)) = g;
      x_bprop->input(r_g, memory);
      x_bprop->execute(memory);
      x_bprop->output(memory, l_g);
      Scalar g_bprop = l_g.at<Scalar>(p_bprop.output(l_i));

      // std::cout << op->name() << " " << g_fprop << " " << g_bprop <<
      // std::endl;

      bool ok = true;
      for (auto &port : p_prog.outputs()) {
        if (!std::isfinite(r_p.at<Scalar>(port))) {
          ok = false;
        }
      }
      if (!ok || (std::abs(g_fprop - g_bprop) < tolerance)) {
        reverse_success++;
      }
    }

    std::cout << (forward_success + reverse_success) * 50 / iteration_count
              << "% " << op->name()
              << " forward:" << forward_success * 100 / iteration_count
              << "% reverse:" << reverse_success * 100 / iteration_count << "%"
              << std::endl;
  }
}

void testOutputs() {

  std::vector<uint8_t> data, data0;
  std::vector<uintptr_t> addresses;

  for (auto *op : Operator::all()) {

    if (op->name().size() < 2 ||
        op->name().substr(op->name().size() - 2) != "_d") {
      continue;
    }

    data.clear();
    addresses.clear();

    {
      size_t addr = 0;
      for (size_t iarg = 0; iarg < op->argumentCount(); iarg++) {
        addresses.push_back(addr);
        addr += op->arg(iarg).size();
      }
      data.clear();
      data.resize(addr, 0);
    }

    for (auto &v : data) {
      // v = rand();
      v = 0;
    }

    for (size_t iarg = 0; iarg < op->argumentCount(); iarg++) {
      if (op->arg(iarg).isOutput()) {
        for (size_t i = 0; i < op->arg(iarg).size(); i++) {
          data[addresses[iarg] + i] = rand();
        }
      }
    }

    data0 = data;

    op->functions().indirect(data.data(), addresses.data());

    {
      size_t addr = 0;
      for (size_t iarg = 0; iarg < op->argumentCount(); iarg++) {
        if (op->arg(iarg).isOutput()) {
          /*
          bool equal = true;
          for (size_t i = 0; i < op->arg(iarg).size(); i++) {
            if (data[addresses[iarg] + i] != data0[addresses[iarg] + i]) {
              equal = false;
            }
          }
          if (equal) {
            std::cout << "op " << op->name() << " arg " << iarg
                      << " output unchanged" << std::endl;
          }
          */
          size_t stride = 4;
          for (size_t base = 0; base < op->arg(iarg).size(); base += stride) {
            bool equal = true;
            for (size_t offset = 0; offset < stride; offset++) {
              if (data[addresses[iarg] + base + offset] !=
                  data0[addresses[iarg] + base + offset]) {
                equal = false;
              }
            }
            if (equal) {
              std::cout << "op " << op->name() << " arg " << iarg << " offset "
                        << base << " output unchanged" << std::endl;
            }
          }
        }
      }
    }
  }
}

void testSignatures() {

  for (auto *op : Operator::all()) {

    if (!op->isMode<compute>()) {
      continue;
    }

    for (size_t iarg = 1; iarg < op->argumentCount(); iarg++) {
      if (op->arg(iarg - 1).isOutput() && op->arg(iarg).isInput()) {
        std::cout << "op " << op->name() << " input after output" << std::endl;
      }
    }

    if (auto prepare_op = op->tryFindVariant<prepare>()) {
      if (prepare_op->argumentCount() < op->argumentCount()) {
        std::cout << "op " << prepare_op->name() << " argument missing"
                  << std::endl;
      }
      for (size_t iarg = 0; iarg < op->argumentCount(); iarg++) {
        if (prepare_op->arg(iarg).typeInfo() != op->arg(iarg).typeInfo()) {
          std::cout << "op " << prepare_op->name() << " arg " << iarg
                    << " mismatch" << std::endl;
        }
        if (!prepare_op->arg(iarg).isInput()) {
          std::cout << "op " << prepare_op->name() << " arg " << iarg
                    << " should be input" << std::endl;
        }
      }
      for (size_t iarg = op->argumentCount();
           iarg < prepare_op->argumentCount(); iarg++) {
        if (!prepare_op->arg(iarg).isOutput()) {
          std::cout << "op " << prepare_op->name() << " arg " << iarg
                    << " should be output" << std::endl;
        }
      }
    }

    if (auto forward_op = op->tryFindVariant<forward>()) {
      if (auto prepare_op = op->tryFindVariant<prepare>()) {

        if (forward_op->argumentCount() != prepare_op->argumentCount()) {
          std::cout << "op " << forward_op->name() << " argument missing"
                    << std::endl;
        }

        for (size_t iarg = 0;
             iarg < prepare_op->argumentCount() - op->argumentCount(); iarg++) {
          auto &a = prepare_op->arg(op->argumentCount() + iarg);
          auto &b = forward_op->arg(iarg);
          if (a.typeInfo() != b.typeInfo() || !b.isInput()) {
            std::cout << "op " << prepare_op->name() << " arg " << iarg
                      << " mismatch" << std::endl;
          }
        }

        for (size_t iarg = 0; iarg < op->argumentCount(); iarg++) {
          auto &a = op->arg(iarg);
          auto &b = forward_op->arg(prepare_op->argumentCount() -
                                    op->argumentCount() + iarg);
          if (a.typeInfo().gradientType() != b.typeInfo() ||
              a.isInput() != b.isInput()) {
            std::cout << "op " << forward_op->name() << " arg "
                      << prepare_op->argumentCount() - op->argumentCount() +
                             iarg
                      << " mismatch" << std::endl;
          }
        }

      } else {

        if (forward_op->argumentCount() != op->argumentCount() * 2) {
          std::cout << "op " << forward_op->name() << " argument missing"
                    << std::endl;
        }

        for (size_t iarg = 0; iarg < op->argumentCount(); iarg++) {
          auto &a = op->arg(iarg);
          auto &b = forward_op->arg(iarg);
          if (a.typeInfo() != b.typeInfo() || !b.isInput()) {
            std::cout << "op " << forward_op->name() << " arg " << iarg
                      << " mismatch" << std::endl;
          }
        }

        for (size_t iarg = 0; iarg < op->argumentCount(); iarg++) {
          auto &a = op->arg(iarg);
          auto &b = forward_op->arg(iarg + op->argumentCount());
          if (a.typeInfo().gradientType() != b.typeInfo() ||
              a.isInput() != b.isInput()) {
            std::cout << "op " << forward_op->name() << " arg "
                      << iarg + op->argumentCount() << " mismatch" << std::endl;
          }
        }
      }
    }

    if (auto reverse_op = op->tryFindVariant<reverse>()) {
      if (auto prepare_op = op->tryFindVariant<prepare>()) {

        if (reverse_op->argumentCount() != prepare_op->argumentCount()) {
          std::cout << "op " << reverse_op->name() << " argument missing"
                    << std::endl;
        }

        for (size_t iarg = 0;
             iarg < prepare_op->argumentCount() - op->argumentCount(); iarg++) {
          if (prepare_op->arg(op->argumentCount() + iarg).typeInfo() !=
              reverse_op->arg(iarg).typeInfo()) {
            std::cout << "op " << prepare_op->name() << " arg " << iarg
                      << " mismatch" << std::endl;
          }
        }

        for (size_t iarg = 0; iarg < op->argumentCount(); iarg++) {
          auto &a = op->arg(iarg);
          auto &b = reverse_op->arg(prepare_op->argumentCount() -
                                    op->argumentCount() + iarg);
          if (a.typeInfo().gradientType() != b.typeInfo() ||
              a.isInput() != b.isOutput()) {
            std::cout << "op " << reverse_op->name() << " arg "
                      << prepare_op->argumentCount() - op->argumentCount() +
                             iarg
                      << " mismatch" << std::endl;
          }
        }

      } else {

        if (reverse_op->argumentCount() != op->argumentCount() * 2) {
          std::cout << "op " << reverse_op->name() << " argument missing"
                    << std::endl;
        }

        for (size_t iarg = 0; iarg < op->argumentCount(); iarg++) {
          auto &a = op->arg(iarg);
          auto &b = reverse_op->arg(iarg);
          if (a.typeInfo() != b.typeInfo() || !b.isInput()) {
            std::cout << "op " << reverse_op->name() << " arg " << iarg
                      << " mismatch" << std::endl;
          }
        }

        for (size_t iarg = 0; iarg < op->argumentCount(); iarg++) {
          auto &a = op->arg(iarg);
          auto &b = reverse_op->arg(iarg + op->argumentCount());
          if (a.typeInfo().gradientType() != b.typeInfo() ||
              a.isInput() != b.isOutput()) {
            std::cout << "op " << reverse_op->name() << " arg "
                      << iarg + op->argumentCount() << " mismatch" << std::endl;
          }
        }
      }
    }
  }
}

void testGradientConsistency() {

  // std::cout << "test gradient consistency" << std::endl;

  typedef double Scalar;

  auto all_operators = Operator::all();
  std::sort(all_operators.begin(), all_operators.end(),
            [](const Operator *a, const Operator *b) {
              return a->name() < b->name();
            });

  auto engine = std::make_shared<SimpleEngine>();
  auto x_prog = engine->createExecutable();
  auto x_prep = engine->createExecutable();
  auto x_fprop = engine->createExecutable();
  auto x_bprop = engine->createExecutable();

  auto computeGradientMatrix = [](const Operator *op, const Program &program,
                                  const std::shared_ptr<Executable> &executable,
                                  const std::shared_ptr<Memory> &memory) {
    Eigen::MatrixXd gradient_matrix;

    size_t input_vector_size = 0;
    for (auto &port : program.inputs()) {
      input_vector_size += port.size();
    }
    Eigen::VectorXd gradient_input(input_vector_size / sizeof(Scalar));

    size_t output_vector_size = 0;
    for (auto &port : program.outputs()) {
      output_vector_size += port.size();
    }
    Eigen::VectorXd gradient_output(output_vector_size / sizeof(Scalar));

    /*
    Eigen::VectorXd gradient_output;

    ssize_t input_size = 0;
    ssize_t output_size = 0;
    for (auto &arg : op->arguments()) {
      if (arg.isInput()) {
        input_size += arg.size();
      }
      if (arg.isOutput()) {
        output_size += arg.size();
      }
    }
    for (auto &arg : op->variant<prepare>()->arguments()) {
      if (arg.isOutput()) {
        input_size -= arg.size();
      }
    }
    */

    // std::cout << __LINE__ << " " << input_vector_size << " " << input_size
    //            << " " << output_size << std::endl;

    /*gradient_matrix = Eigen::MatrixXd(output_vector_size / sizeof(Scalar),
                                      input_vector_size / sizeof(Scalar));
    gradient_matrix.setZero();*/

    for (size_t input_index = 0; input_index < gradient_input.size();
         input_index++) {

      gradient_input.setZero();
      gradient_input[input_index] = 1;

      executable->inputVector(gradient_input, memory);
      executable->execute(memory);
      executable->outputVector(memory, gradient_output);

      /*
      {
        size_t ioutput = 0;
        for (auto &arg : op->arguments()) {
          if (arg.isOutput()) {
            for (size_t i = 0;
                 i < program.output(ioutput).size() / sizeof(Scalar); i++) {
              if (i >= arg.size() / sizeof(Scalar)) {
                gradient_output[program.output(ioutput).offset() /
                                sizeof(Scalar)] = 0;
              }
            }
            ioutput++;
          }
        }
      }
      */

      if (input_index == 0) {
        gradient_matrix = Eigen::MatrixXd::Zero(gradient_output.size(),
                                                gradient_input.size());
      }
      gradient_matrix.col(input_index) = gradient_output;
    }

    // std::cout << gradient_matrix << std::endl;

    return gradient_matrix;
  };

  for (auto *op : all_operators) {

    // std::cout << op->name() << std::endl;

    if (!op->isMode<compute>()) {
      continue;
    }
    if (op->name().size() < 2 ||
        op->name().substr(op->name().size() - 2) != "_d") {
      continue;
    }
    if (!op->tryFindVariant<forward>()) {
      continue;
    }
    if (!op->tryFindVariant<reverse>()) {
      continue;
    }

    // std::cout << op->name() << ": ";
    // std::cout.flush();

    Program p_prog;
    Program p_prep;
    Program p_fprop;
    Program p_bprop;

    std::vector<Program::Instruction> instructions;
    size_t memory_size = 0;
    instructions.push_back((uintptr_t)op);
    for (auto &arg : op->arguments()) {
      instructions.push_back(memory_size);
      if (arg.isInput()) {
        p_prog.addInput(
            Program::Input(arg.typeInfo(), memory_size, memory_size, 0));
      }
      if (arg.isOutput()) {
        p_prog.addOutput(
            Program::Output(arg.typeInfo(), memory_size, memory_size, 0));
      }
      memory_size += arg.size();
    }
    p_prog.updateMemorySize(memory_size);
    p_prog.setInstructions(instructions);

    buildGradients(p_prog, p_prep, &p_fprop, &p_bprop);

    x_prog->compile(p_prog);
    x_prep->compile(p_prep);
    x_fprop->compile(p_fprop);
    x_bprop->compile(p_bprop);

    Buffer buffer;

    size_t iterations = 100;
    size_t successes = 0;

    for (size_t i = 0; i < iterations; i++) {

      std::vector<Scalar> inputs;
      for (auto &port : p_prog.inputs()) {
        for (size_t i = 0; i < port.size() / sizeof(Scalar); i++) {
          inputs.push_back(rand() * 2.0 / RAND_MAX - 1.0);
        }
      }

      auto memory = engine->createMemory();
      x_prog->inputVector(inputs, memory);
      x_prog->execute(memory);
      x_prep->execute(memory);
      Eigen::MatrixXd forward_gradient_matrix = computeGradientMatrix(
          op->variant<forward>(), p_fprop, x_fprop, memory);

      memory = engine->createMemory();
      x_prog->inputVector(inputs, memory);
      x_prog->execute(memory);
      x_prep->execute(memory);
      Eigen::MatrixXd reverse_gradient_matrix =
          computeGradientMatrix(op->variant<reverse>(), p_bprop, x_bprop,
                                memory)
              .transpose();

      if (forward_gradient_matrix.rows() != reverse_gradient_matrix.rows() ||
          forward_gradient_matrix.cols() != reverse_gradient_matrix.cols()) {
        continue;
      }

      bool err = false;
      for (size_t row = 0; row < forward_gradient_matrix.rows(); row++) {
        for (size_t col = 0; col < forward_gradient_matrix.cols(); col++) {
          auto &a = forward_gradient_matrix(row, col);
          auto &b = reverse_gradient_matrix(row, col);
          if ((std::abs(a - b) > 1e-6) &&
              (std::memcmp(&a, &b, sizeof(Scalar)) != 0)) {
            // if (!(a == b)) {
            // std::cout << "err " << a << " " << b << std::endl;
            err = true;
          }
        }
      }
      if (err) {
        continue;
      }

      successes++;

      /*Buffer input_data;
      for (auto &port : p_prog.inputs()) {
        Scalar p = rand() * 2.0 / RAND_MAX - 1.0;
        Scalar g = rand() * 2.0 / RAND_MAX - 1.0;
        l_p.at<Scalar>(port) = p;
        l_g.at<Scalar>(port) = g;
        l_a.at<Scalar>(port) = p - g * delta * 0.5;
        l_b.at<Scalar>(port) = p + g * delta * 0.5;
      }
      */
    }

    std::cout << std::round(successes * 100.0 / iterations) << "% "
              << op->name() << std::endl;
  }
}

} // namespace tractor
