// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/core/eigen.h>
#include <tractor/core/gradients.h>
#include <tractor/core/linesearch.h>
#include <tractor/core/profiler.h>
#include <tractor/core/program.h>
#include <tractor/core/solver.h>

#define TRACTOR_STRINGIFY_2(x) TRACTOR_STRINGIFY(x)

#define TRACTOR_CHECK_ALL_FINITE(x)                                            \
  if (!x.allFinite()) {                                                        \
    std::cout << std::endl                                                     \
              << x.size() << std::endl                                         \
              << std::endl                                                     \
              << x << std::endl                                                \
              << std::endl;                                                    \
    throw std::runtime_error(TRACTOR_STRINGIFY(                                \
        x) " not finite " __FILE__ ":" TRACTOR_STRINGIFY_2(__LINE__));         \
  }

#define TRACTOR_CHECK_FINITE(x)                                                \
  if (!std::isfinite(x)) {                                                     \
    throw std::runtime_error(TRACTOR_STRINGIFY(                                \
        x) " not finite " __FILE__ ":" TRACTOR_STRINGIFY_2(__LINE__));         \
  }

#if 1
#define TRACTOR_LOG_VAR(x)                                                     \
  std::cout << TRACTOR_STRINGIFY_2(x) << ": " << x << std::endl;
#else
#define TRACTOR_LOG_VAR(x)
#endif

#if 0
#define TRACTOR_LOG_VEC(x)                                                     \
  std::cout << TRACTOR_STRINGIFY_2(x) << std::endl                             \
            << x << std::endl                                                  \
            << std::endl;
#else
#define TRACTOR_LOG_VEC(x)
#endif

namespace tractor {

class SolverBase : public Solver {

  Buffer _accu_in, _accu_grad;

protected:
  Program _p_prog, _p_prep, _p_fprop, _p_bprop, _p_hprop, _p_accu;
  std::shared_ptr<Executable> _x_prog, _x_prep, _x_fprop, _x_bprop, _x_hprop,
      _x_accu;
  std::shared_ptr<Memory> _memory;
  std::shared_ptr<Executable> _x_project;
  std::shared_ptr<Executable> _x_barrier_init, _x_barrier_step,
      _x_barrier_diagonal;
  std::shared_ptr<Executable> _x_penalty_init, _x_penalty_step,
      _x_penalty_diagonal;
  Program _p_project;
  Program _p_barrier_init, _p_barrier_step, _p_barrier_diagonal;
  Program _p_penalty_init, _p_penalty_step, _p_penalty_diagonal;
  std::vector<int> _priority_list;
  std::vector<size_t> _constraint_indices;

  template <class Scalar>
  class RegularizedMatrixReplacement
      : public MatrixReplacement<Scalar>::ExecutableImpl {
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
    Scalar _regularization = Scalar(0);

  public:
    virtual void mul(const Vector &input, Vector &output) const override {
      TRACTOR_PROFILER("hprop");
      TRACTOR_CHECK_ALL_FINITE(input);
      MatrixReplacement<Scalar>::ExecutableImpl::mul(input, output);
      TRACTOR_CHECK_ALL_FINITE(output);
      if (_regularization > Scalar(0)) {
        output.array() += input.array() * _regularization;
      }
      TRACTOR_CHECK_ALL_FINITE(output);
    }
    void setRegularization(const Scalar &regularization) {
      _regularization = regularization;
    }
  };

  template <class Matrix, class Scalar = typename std::decay<
                              decltype((*(Matrix *)nullptr)(0, 0))>::type>
  static void
  _buildGradientMatrix(const std::shared_ptr<Executable> &executable,
                       const std::shared_ptr<Memory> &memory, Matrix &matrix,
                       const Scalar &delta = Scalar(1)) {
    size_t cols = executable->inputBufferSize() / sizeof(Scalar);
    size_t rows = executable->outputBufferSize() / sizeof(Scalar);
    matrix.setOnes(rows, cols);
    Eigen::VectorXd input;
    input.setZero(cols);
    Eigen::VectorXd output;
    Scalar delta_rcp = Scalar(1) / delta;
    for (size_t col = 0; col < cols; col++) {
      input[col] = delta;
      executable->run(input, memory, output);
      matrix.col(col) = output * delta_rcp;
      input[col] = 0;
    }
  }

  template <class Scalar> void _compileGradients(const Program &prog) {

    _p_prog = prog;
    _x_prog->compile(_p_prog);

    buildGradients(_p_prog, _p_prep, &_p_fprop, &_p_bprop, &_p_hprop, &_p_accu);
    _x_prep->compile(_p_prep);
    _x_fprop->compile(_p_fprop);
    _x_bprop->compile(_p_bprop);
    _x_hprop->compile(_p_hprop);
    _x_accu->compile(_p_accu);

    buildConstraints(_p_prog, _p_fprop, _p_bprop, _p_hprop,
                     TypeInfo::get<Scalar>(), &_p_project, &_p_barrier_init,
                     &_p_barrier_step, &_p_barrier_diagonal, &_p_penalty_init,
                     &_p_penalty_step, &_p_penalty_diagonal);

    _x_project->compile(_p_project);

    _x_barrier_init->compile(_p_barrier_init);
    _x_barrier_step->compile(_p_barrier_step);
    _x_barrier_diagonal->compile(_p_barrier_diagonal);

    _x_penalty_init->compile(_p_penalty_init);
    _x_penalty_step->compile(_p_penalty_step);
    _x_penalty_diagonal->compile(_p_penalty_diagonal);

    _constraint_indices.clear();
    _priority_list.clear();
    for (auto &goal : _p_prog.goals()) {
      auto &output = _p_fprop.output(goal.port());
      if (output.size() % sizeof(Scalar) != 0) {
        throw std::runtime_error("output type mismatch");
      }
      for (size_t element = 0; element < output.size() / sizeof(Scalar);
           element++) {
        if (goal.priority() == 1) {
          _constraint_indices.push_back(output.offset() / sizeof(Scalar) +
                                        element);
        }
        _priority_list.emplace_back(goal.priority());
      }
    }
    TRACTOR_LOG_VAR(_constraint_indices.size());
  }

  template <class Pos, class Grad> void accumulate(Pos &pos, const Grad &grad) {
    _accu_in.fromVectorDense(pos);
    _accu_grad.fromVectorDense(grad);
    _accu_in.append(_accu_grad);
    _x_accu->input(_accu_in, _memory);
    _x_accu->execute(_memory);
    _x_accu->outputVector(_memory, pos);
  }

  SolverBase(const std::shared_ptr<Engine> &engine) : Solver(engine) {
    _memory = engine->createMemory();
    _x_accu = engine->createExecutable();
    _x_barrier_diagonal = engine->createExecutable();
    _x_barrier_init = engine->createExecutable();
    _x_barrier_step = engine->createExecutable();
    _x_bprop = engine->createExecutable();
    _x_fprop = engine->createExecutable();
    _x_hprop = engine->createExecutable();
    _x_penalty_diagonal = engine->createExecutable();
    _x_penalty_init = engine->createExecutable();
    _x_penalty_step = engine->createExecutable();
    _x_prep = engine->createExecutable();
    _x_prog = engine->createExecutable();
    _x_project = engine->createExecutable();
  }
};

} // namespace tractor
