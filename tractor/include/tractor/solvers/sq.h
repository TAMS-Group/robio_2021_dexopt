// (c) 2020-2021 Philipp Ruppel

#pragma once

#if 1

#include <tractor/solvers/base.h>

namespace tractor {

template <class Scalar,
          class LinearSolver = Eigen::ConjugateGradient<
              MatrixReplacement<Scalar>, Eigen::Lower | Eigen::Upper,
              Eigen::IdentityPreconditioner>>
class LeastSquaresSolver : public SolverBase {
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
  LinearSolver _linear_solver;
  std::shared_ptr<SolverBase::RegularizedMatrixReplacement<Scalar>> _hgrad_p;
  MatrixReplacement<Scalar> _hgrad;
  Vector _line_search_left, _line_search_right;
  Vector _residuals;
  Vector _nonlinear_solution;
  Vector _previous_nonlinear_solution;
  Vector _linear_solution;
  Vector _gradient_temp;

public:
  bool _use_adaptive_regularization = 0;
  Scalar _regularization = 0.0;
  int _max_linear_iterations = -1;
  Scalar _step_scaling = 1.0;

protected:
  virtual void _compile(const Program &prog) override {
    _compileGradients<Scalar>(prog);
  }

  virtual void _input(const Buffer &buffer) override {
    buffer.toVector(_p_prog.inputs(), _nonlinear_solution);
    _previous_nonlinear_solution = _nonlinear_solution;
    if (_use_adaptive_regularization) {
      _regularization = 1.0;
    }
  }

  virtual void _output(Buffer &buffer) override {
    buffer.fromVector(_p_prog.inputs(), _nonlinear_solution);
  }

  virtual void _parameterize(const Buffer &buffer) override {
    _x_prog->parameterize(buffer, _memory);
  }

  double _loss = -1;
  virtual double loss() const override { return _loss; }

  virtual double _step() override {

    std::cout << "sq step" << std::endl;

    // std::cout << "reg " << _regularization << std::endl;

    if (!_nonlinear_solution.allFinite()) {
      throw std::runtime_error("previous solution not finite");
    }

    _hgrad_p->setRegularization(_regularization);

    _x_prog->run(_nonlinear_solution, _memory, _gradient_temp);

    // std::cout << "loss " << _gradient_temp.squaredNorm() << std::endl;
    _loss = _gradient_temp.squaredNorm();

    TRACTOR_CHECK_ALL_FINITE(_gradient_temp);

    _x_prep->execute(_memory);
    _x_bprop->run(_gradient_temp, _memory, _residuals);

    TRACTOR_CHECK_ALL_FINITE(_residuals);

    _linear_solution.resize(_residuals.size());

    _linear_solver.setTolerance(tolerance());
    //_linear_solver.setMaxIterations(_residuals.size());
    //_linear_solver.setMaxIterations(100);

    if (_max_linear_iterations > 0) {
      _linear_solver.setMaxIterations(_max_linear_iterations);
    } else {
      _linear_solver.setMaxIterations(_residuals.size());
    }

    {
      TRACTOR_PROFILER("sq linear");
      _linear_solver.compute(_hgrad);
      _linear_solution = _linear_solver.solve(_residuals);
    }

    TRACTOR_CHECK_ALL_FINITE(_linear_solution);

    _linear_solution.array() = -_linear_solution.array();

#if 0
    {
      TRACTOR_PROFILER("project inequality constraints");
      _x_project->parameterVector(std::array<Scalar, 1>({Scalar(0)}), _memory);
      _x_project->run(_linear_solution, _memory, _linear_solution);
      TRACTOR_CHECK_ALL_FINITE(_linear_solution);
    }
#endif

#if 0
    accumulate(_nonlinear_solution, _linear_solution);
    if (!_nonlinear_solution.allFinite()) {
      throw std::runtime_error("solution not finite");
    }
#endif

#if 1
    //_linear_solution *= 0.5;
    _linear_solution *= _step_scaling;
    accumulate(_nonlinear_solution, _linear_solution);
    TRACTOR_CHECK_ALL_FINITE(_nonlinear_solution);
#endif

#if 0
    {
      TRACTOR_PROFILER("sq nl line search");
      auto f = [&](const Scalar &v) {
        _line_search_left = _nonlinear_solution;
        accumulate(_line_search_left, _linear_solution * v);
        _x_prog->inputVector(_line_search_left, _memory);
        _x_prog->execute(_memory);
        _x_prog->outputVector(_memory, _line_search_right);
        Scalar ret = _line_search_right.squaredNorm();
        return ret;
      };
      if (_use_adaptive_regularization) {
        if (f(1) < f(0.5)) {
          _regularization *= Scalar(0.5);
        } else {
          _regularization *= Scalar(2.0);
          _regularization = std::min(_regularization, Scalar(1.0));
        }
        std::cout << "reg " << _regularization << std::endl;
      }
    }
    accumulate(_nonlinear_solution, _linear_solution);
    if (!_nonlinear_solution.allFinite()) {
      throw std::runtime_error("solution not finite");
    }
#endif

#if 0
    {
      TRACTOR_PROFILER("sq nl line search");
      auto f = [&](const Scalar &v) {

        _line_search_left = _nonlinear_solution;
        accumulate(_line_search_left, _linear_solution * v);
        // applyBounds(_line_search_left);

        _x_prog->inputVector(_line_search_left, _memory);
        _x_prog->execute(_memory);
        _x_prog->outputVector(_memory, _line_search_right);

        Scalar ret = _line_search_right.squaredNorm();
        return ret;
      };

      if (_use_adaptive_regularization) {
        if (f(1) < f(0.5)) {
          _regularization *= Scalar(0.1);
        } else {
          _regularization *= Scalar(10.0);
        }
      }

      Scalar line_search_result =
          minimizeTernary(f, tolerance(), Scalar(0), Scalar(1));
      std::cout << "ls " << line_search_result << std::endl;
      line_search_result *= Scalar(0.9);
      _linear_solution *= line_search_result;
    }

    accumulate(_nonlinear_solution, _linear_solution);
    if (!_nonlinear_solution.allFinite()) {
      throw std::runtime_error("solution not finite");
    }
#endif

#if 0
    {
      TRACTOR_PROFILER("sq nl line search");
      auto f = [&](const Scalar &v) {

        _line_search_left = _nonlinear_solution;
        accumulate(_line_search_left, _linear_solution * v);

        _x_prog->inputVector(_line_search_left, _memory);
        _x_prog->execute(_memory);
        _x_prog->outputVector(_memory, _line_search_right);

        Scalar ret = _line_search_right.squaredNorm();
        if (!std::isfinite(ret)) {
          ret = std::numeric_limits<Scalar>::max();
        }
        return ret;
      };

      Scalar line_search_result = 1;
      while (!(f(line_search_result * 0.5) >= f(line_search_result))) {
        line_search_result *= 0.5;
      }
      line_search_result =
          minimizeTernary(f, line_search_result * 0.05, Scalar(0),
                          Scalar(line_search_result * 2));
      std::cout << "ls " << line_search_result << std::endl;
      // line_search_result *= Scalar(0.9);
      _linear_solution *= line_search_result;
    }

    accumulate(_nonlinear_solution, _linear_solution);
    if (!_nonlinear_solution.allFinite()) {
      throw std::runtime_error("solution not finite");
    }
#endif

#if 0
    {
      Scalar line_search_result = 1;
      while (true) {
        _line_search_left = _nonlinear_solution;
        accumulate(_line_search_left, _linear_solution * line_search_result);
        _x_prog->run(_line_search_left, _memory, _line_search_right);
        //_x_prep->execute(_memory);
        //_x_bprop->run(_line_search_right, _memory, _line_search_right);
        line_search_result *= 0.5;
        if (_line_search_right.allFinite()) {
          break;
        }
        if (line_search_result < tolerance()) {
          line_search_result = 0;
          break;
        }
      }
      while (true) {
        _line_search_left = _nonlinear_solution;
        accumulate(_line_search_left, _linear_solution * line_search_result);
        _x_prog->run(_line_search_left, _memory, _line_search_right);
        //_x_prep->execute(_memory);
        //_x_bprop->run(_line_search_right, _memory, _line_search_right);
        if (_line_search_right.allFinite()) {
          break;
        }
        line_search_result *= 0.5;
        if (line_search_result < tolerance()) {
          line_search_result = 0;
          break;
        }
      }
      std::cout << "line_search_result " << line_search_result << std::endl;
      if (line_search_result > 0) {
        accumulate(_nonlinear_solution, _linear_solution * line_search_result);
      }
    }
#endif

    // applyBounds(_nonlinear_solution);
    /*double step =
        (_previous_nonlinear_solution - _nonlinear_solution).squaredNorm();
    _previous_nonlinear_solution = _nonlinear_solution;
    return step;*/
    return 1;
  }

public:
  LeastSquaresSolver(const std::shared_ptr<Engine> &engine)
      : SolverBase(engine) {
    _hgrad_p =
        std::make_shared<SolverBase::RegularizedMatrixReplacement<Scalar>>();
    _hgrad_p->setExecutable(_x_hprop);
    _hgrad_p->setMemory(_memory);
    _hgrad = MatrixReplacement<Scalar>(_hgrad_p);
  }
};

} // namespace tractor

#endif
