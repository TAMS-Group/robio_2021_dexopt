// (c) 2020-2021 Philipp Ruppel

#pragma once

#if 0

#include <tractor/core/eigen.h>
#include <tractor/core/gradients.h>
#include <tractor/core/solver.h>

namespace tractor {

// Solves equality-constrained equations
template <class Scalar,
          class LinearSolver = Eigen::ConjugateGradient<
              MatrixReplacement<Scalar>, Eigen::Lower | Eigen::Upper,
              Eigen::IdentityPreconditioner>>
class EqualityConstrainedSolver : public Solver {

  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;

  std::shared_ptr<Executable> _x_prog, _x_prep, _x_residual, _x_gradient;
  std::shared_ptr<Memory> _memory;

  Program _p_prog, _p_prep, _p_bprop, _p_fprop, _p_residual, _p_gradient;
  Buffer _buf_left, _buf_right;

  LinearSolver _linear_solver;

  MatrixReplacement<Scalar> _hgrad;

  Vector _residuals;
  Vector _nonlinear_solution;
  Vector _linear_solution;
  Vector _errors;

protected:
  virtual void _compile(const Program &prog) override {
    _p_prog = prog;

    buildGradients(_p_prog, _p_prep, &_p_fprop, &_p_bprop);
    _log("prep", _p_prep);
    _log("fprop", _p_fprop);
    _log("bprop", _p_bprop);
    _x_prog->compile(_p_prog);
    _x_prep->compile(_p_prep);

    buildDual(_p_prog, _p_fprop, _p_bprop, _p_gradient, _p_residual);
    _log("gradient", _p_gradient);
    _log("residual", _p_residual);
    _x_residual->compile(_p_residual);
    _x_gradient->compile(_p_gradient);
  }

  virtual void _input(const Buffer &buffer) override {
    buffer.toVector(_p_prog.inputs(), _nonlinear_solution);
  }

  virtual void _output(Buffer &buffer) override {
    buffer.fromVector(_p_prog.inputs(), _nonlinear_solution);
  }

  virtual void _parameterize(const Buffer &buffer) override {
    _x_prog->parameterize(buffer, _memory);
  }

  virtual double _step() override {

    _x_prog->inputVector(_nonlinear_solution, _memory);
    _x_prog->execute(_memory);
    _x_prog->output(_memory, _buf_right);

    /*
    _buf_right.toVector(_p_prog.outputs(), _errors);
    _loss(_errors.squaredNorm());
    if (_stop()) {
      return;
    }
    */

    _x_prep->execute(_memory);

    _x_residual->input(_buf_right, _memory);
    _x_residual->execute(_memory);
    _x_residual->outputVector(_memory, _residuals);

    _linear_solution = _residuals;

    _linear_solver.compute(_hgrad);
    _linear_solution = _linear_solver.solve(_residuals);

    if (!_linear_solution.allFinite()) {
      return -1;
    }

    _nonlinear_solution -= _linear_solution.head(_nonlinear_solution.size());

    return _linear_solution.head(_nonlinear_solution.size()).squaredNorm();
  }

public:
  EqualityConstrainedSolver(const std::shared_ptr<Engine> &engine)
      : Solver(engine) {
    _x_prog = engine->createExecutable();
    _x_prep = engine->createExecutable();
    _x_residual = engine->createExecutable();
    _x_gradient = engine->createExecutable();
    _memory = engine->createMemory();
    _hgrad = MatrixReplacement<Scalar>(_x_gradient, _memory);
  }
};

} // namespace tractor

#endif
