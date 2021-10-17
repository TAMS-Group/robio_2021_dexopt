// (c) 2020-2021 Philipp Ruppel

#if 0

#pragma once

#include <tractor/core/eigen.h>
#include <tractor/core/gradients.h>
#include <tractor/core/solver.h>

namespace tractor {

// Gradient-descent with quadratic step size, equality constraints, and box
// constraints
template <class Scalar> class ProjectedGradientSolver : public Solver {

  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;

  std::shared_ptr<Executable> _x_prog, _x_prep, _x_residual, _x_gradient,
      _x_bprop, _x_fprop;
  std::shared_ptr<Memory> _memory;

  Program _p_prog, _p_prep, _p_residual, _p_gradient, _p_bprop, _p_fprop;
  Buffer _buf_left, _buf_right;

  MatrixReplacement<Scalar> _hgrad;

  Vector _residuals;
  Vector _nonlinear_solution;
  Vector _previous_nonlinear_solution;
  Vector _linear_solution;
  Vector _errors;
  Vector _v_step;

  typedef Eigen::ConjugateGradient<MatrixReplacement<Scalar>,
                                   Eigen::Lower | Eigen::Upper,
                                   Eigen::IdentityPreconditioner>
      LinearSolver;
  LinearSolver _linear_solver;

protected:
  virtual void _compile(const Program &prog) override {
    _p_prog = prog;

    buildGradients(_p_prog, _p_prep, &_p_fprop, &_p_bprop);
    _log("prep", _p_prep);
    _log("fprop", _p_fprop);
    _log("bprop", _p_bprop);
    _x_prog->compile(_p_prog);
    _x_prep->compile(_p_prep);
    _x_bprop->compile(_p_bprop);
    _x_fprop->compile(_p_fprop);

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

    //_x_residual->input(_buf_right, _memory);
    //_x_residual->execute(_memory);
    //_x_residual->outputVector(_memory, _residuals);

    _x_bprop->input(_buf_right, _memory);
    _x_bprop->execute(_memory);
    _x_bprop->outputVector(_memory, _residuals);

    _linear_solution = _residuals;
    _residuals.setZero(_x_gradient->inputBufferSize() / sizeof(Scalar));
    _residuals.head(_linear_solution.size()) = _linear_solution;

    _linear_solution = _residuals;

    //_linear_solver.setMaxIterations(1);
    _linear_solver.compute(_hgrad);
    _linear_solution = _linear_solver.solve(_residuals);

    if (!_linear_solution.allFinite()) {
      return -1;
    }

    _previous_nonlinear_solution = _nonlinear_solution;

    _nonlinear_solution -= _linear_solution.head(_nonlinear_solution.size());

    {
      size_t i = 0;
      for (auto &input : _p_prog.inputs()) {
        if (input.hasLowerBound()) {
          _nonlinear_solution[i] = std::max(
              _nonlinear_solution[i],
              *(const Scalar *)((const uint8_t *)_p_prog.boundData().data() +
                                input.lowerBoundAddress()));
        }
        if (input.hasUpperBound()) {
          _nonlinear_solution[i] = std::min(
              _nonlinear_solution[i],
              *(const Scalar *)((const uint8_t *)_p_prog.boundData().data() +
                                input.upperBoundAddress()));
        }
        i++;
      }
    }

    return (_nonlinear_solution - _previous_nonlinear_solution).squaredNorm();

    /*_linear_solution.noalias() = _hgrad * _residuals;
    double f = _residuals.dot(_residuals) / _residuals.dot(_linear_solution);
    std::cout << "f " << f << std::endl;
    _nonlinear_solution -= _residuals.head(_nonlinear_solution.size()) * f;*/

    // std::cout << "residuals " << _residuals << std::endl;

    /*{
      size_t i = 0;
      for (auto &input : _p_prog.inputs()) {
        if (input.hasLowerBound()) {
          if (_linear_solution[i] > Scalar(0) &&
              _nonlinear_solution[i] <=
                  *(const Scalar *)((const uint8_t *)_p_prog.boundData()
                                        .data() +
                                    input.lowerBoundAddress())) {
            _residuals[i] = 0;
          }
        }
        if (input.hasUpperBound()) {
          if (_linear_solution[i] < Scalar(0) &&
              _nonlinear_solution[i] >=
                  *(const Scalar *)((const uint8_t *)_p_prog.boundData()
                                        .data() +
                                    input.upperBoundAddress())) {
            _residuals[i] = 0;
          }
        }
        i++;
      }
    }

    _linear_solution.noalias() = _hgrad * _residuals;

    // std::cout << "residuals " << _residuals << std::endl;
    // std::cout << "linear_solution " << _linear_solution << std::endl;

    double f = _residuals.dot(_residuals) / _residuals.dot(_linear_solution);
    // std::cout << "f " << f << std::endl;
    if (!std::isfinite(f)) {
      return -1;
    }
    _v_step = _residuals.head(_nonlinear_solution.size()) * -f;
    _nonlinear_solution += _v_step;

    {
      size_t i = 0;
      for (auto &input : _p_prog.inputs()) {
        if (input.hasLowerBound()) {
          _nonlinear_solution[i] = std::max(
              _nonlinear_solution[i],
              *(const Scalar *)((const uint8_t *)_p_prog.boundData().data() +
                                input.lowerBoundAddress()));
        }
        if (input.hasUpperBound()) {
          _nonlinear_solution[i] = std::min(
              _nonlinear_solution[i],
              *(const Scalar *)((const uint8_t *)_p_prog.boundData().data() +
                                input.upperBoundAddress()));
        }
        i++;
      }
    }

    return _v_step.squaredNorm();
    */
  }

public:
  ProjectedGradientSolver(const std::shared_ptr<Engine> &engine)
      : Solver(engine) {
    _x_prog = engine->createExecutable();
    _x_prep = engine->createExecutable();
    _x_residual = engine->createExecutable();
    _x_gradient = engine->createExecutable();
    _x_bprop = engine->createExecutable();
    _x_fprop = engine->createExecutable();
    _memory = engine->createMemory();
    _hgrad = MatrixReplacement<Scalar>(_x_gradient, _memory);
  }
};

} // namespace tractor

#endif
