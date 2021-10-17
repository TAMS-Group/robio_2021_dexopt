// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/solvers/base.h>

namespace tractor {

template <class Scalar, bool UsePreconditioner = false>
class PenaltySolver : public SolverBase {

public:
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
  Eigen::MatrixXd _m_fprop, _m_bprop;
  Scalar _initial_barrier_weight = Scalar(1);
  Scalar _min_barrier_weight = Scalar(1e-3);
  Scalar _current_barrier_weight_start = Scalar(0);
  Scalar _current_barrier_weight = Scalar(0);
  Scalar _barrier_decrease = Scalar(0.1);
  Scalar _barrier_increase = Scalar(10);
  Vector _qp_residuals;
  Vector _nonlinear_solution;
  Vector _previous_nonlinear_solution;
  Vector _vec_right;
  Vector _diag_in, _diag_out, _diagonal;
  Vector _line_search_temp_solution;
  Vector _line_search_temp_residuals;
  Vector _line_search_temp_gradients;
  Vector _qp_solution, _step_solution;
  Vector _v_project, _v_barrier_input, _temp_residuals, _v_barrier_gradients,
      _v_barrier_diagonal, _step_residuals, _temp_residuals_2;
  size_t _primal_variable_count = 0;
  size_t _dual_variable_count = 0;
  bool _use_matrices = true;
  Scalar _penalty_weight = 1;
  Scalar _regularization_weight = 1;
  Scalar _objective_weight = 1;
  bool _enable_objectives = false;
  bool _enable_penalties = false;
  bool _enable_regularization = false;
  Scalar _constraint_padding = Scalar(0.01);

  template <class Input, class Output>
  void _fprop(Input &&input, Output &&output) {
    if (_use_matrices) {
      output = _m_fprop * input;
    } else {
      _x_fprop->run(input, _memory, output);
    }
  }

  template <class Input, class Output>
  void _bprop(Input &&input, Output &&output) {
    if (_use_matrices) {
      output = _m_bprop * input;
    } else {
      _x_bprop->run(input, _memory, output);
    }
  }

  Vector _dualprop_temp, _dualprop_temp_2;
  template <class Input, class Output>
  void _dualprop(Input &&input, Output &&output) {

    TRACTOR_CHECK_ALL_FINITE(input);

    output.setZero(_dual_variable_count);

    _fprop(input.head(_primal_variable_count), _dualprop_temp);

    for (size_t i = 0; i < _constraint_indices.size(); i++) {
      size_t j = _constraint_indices[i];
      output[_primal_variable_count + i] = _dualprop_temp[j];
    }

    if (_enable_objectives) {
      for (size_t i = 0; i < _primal_variable_count; i++) {
        if (_priority_list[i] <= 0) {
          _dualprop_temp[i] *= _objective_weight;
        }
      }
    } else {
      for (size_t i = 0; i < _primal_variable_count; i++) {
        if (_priority_list[i] <= 0) {
          _dualprop_temp[i] = Scalar(0);
        }
      }
    }

    for (size_t i = 0; i < _constraint_indices.size(); i++) {
      size_t j = _constraint_indices[i];
      _dualprop_temp[j] += input[_primal_variable_count + i];
    }

    _bprop(_dualprop_temp, output.head(_primal_variable_count));

    if (_enable_penalties) {
      _x_penalty_step->run(input.head(_primal_variable_count), _memory,
                           _dualprop_temp);
      output.head(_primal_variable_count) += _dualprop_temp * _penalty_weight;
    }

    if (_enable_regularization) {
      output.head(_primal_variable_count) +=
          input.head(_primal_variable_count) * _regularization_weight;
    }

    TRACTOR_CHECK_ALL_FINITE(output);
  }

  Vector _dualres_temp;
  template <class Input, class Output>
  void _dualres(Input &&input, Output &&output) {

    TRACTOR_CHECK_ALL_FINITE(input);

    output.setZero(input.size());

    output = -_qp_residuals;

    if (_enable_objectives) {
      for (size_t i = 0; i < _primal_variable_count; i++) {
        if (_priority_list[i] <= 0) {
          output[i] *= _objective_weight;
        }
      }
    } else {
      for (size_t i = 0; i < _primal_variable_count; i++) {
        if (_priority_list[i] <= 0) {
          output[i] = Scalar(0);
        }
      }
    }

    if (_enable_penalties) {

      _x_penalty_init->run(input.head(_primal_variable_count), _memory,
                           _dualres_temp);
      output.head(_primal_variable_count) += _dualres_temp * _penalty_weight;

      _x_penalty_step->run(input.head(_primal_variable_count), _memory,
                           _dualprop_temp);
      output.head(_primal_variable_count) -= _dualprop_temp * _penalty_weight;
    }

    if (_enable_regularization) {
      output.head(_primal_variable_count) +=
          _qp_solution.head(_primal_variable_count) * _regularization_weight;
    }

    TRACTOR_CHECK_ALL_FINITE(output);
  }

  class DualMatrixReplacement : public MatrixReplacement<Scalar>::Impl {
    PenaltySolver *_solver = nullptr;

  public:
    virtual size_t rows() const override {
      return _solver->_dual_variable_count;
    }
    virtual size_t cols() const override {
      return _solver->_dual_variable_count;
    }
    virtual void mul(const Vector &input, Vector &output) const override {
      _solver->_dualprop(input, output);
    }
    virtual const Vector &diagonal() const override {
      throw std::runtime_error("NYI");
    }
    DualMatrixReplacement(PenaltySolver *solver) : _solver(solver) {}
  };

  std::shared_ptr<DualMatrixReplacement> _dual_matrix_replacement;
  MatrixReplacement<Scalar> _hgrad;
  typedef Eigen::ConjugateGradient<MatrixReplacement<Scalar>,
                                   Eigen::Lower | Eigen::Upper,
                                   Eigen::IdentityPreconditioner>
      LinearSolver;
  LinearSolver _linear_solver;

protected:
  virtual void _compile(const Program &prog) override {

    _compileGradients<Scalar>(prog);

    _primal_variable_count = _x_fprop->inputBufferSize() / sizeof(Scalar);
    TRACTOR_LOG_VAR(_primal_variable_count);

    _dual_variable_count = _primal_variable_count + _constraint_indices.size();
    TRACTOR_LOG_VAR(_dual_variable_count);
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

    TRACTOR_PROFILER("step");

    //_linear_solver.setTolerance(tolerance());

    {
      TRACTOR_PROFILER("nl program");
      _x_prog->run(_nonlinear_solution, _memory, _vec_right);
    }

    {
      TRACTOR_PROFILER("prepare");
      _x_prep->execute(_memory);
    }

    {
      TRACTOR_PROFILER("residual");
      _qp_residuals.setZero(_dual_variable_count);
      _x_bprop->run(_vec_right, _memory,
                    _qp_residuals.head(_primal_variable_count));
      for (size_t i = 0; i < _constraint_indices.size(); i++) {
        size_t j = _constraint_indices[i];
        _qp_residuals[_primal_variable_count + i] = _vec_right[j];
      }
    }

    if (_use_matrices) {
      _buildGradientMatrix(_x_fprop, _memory, _m_fprop);
      _buildGradientMatrix(_x_bprop, _memory, _m_bprop);
      if ((_m_fprop - _m_bprop.transpose()).squaredNorm() > 1e-12) {
        throw std::runtime_error("jacobians inconsistent");
      }
    }

    _qp_solution.setZero(_dual_variable_count);
    _step_solution = _qp_solution;

    {
      TRACTOR_PROFILER("solve objectives");
      _enable_objectives = true;
      _enable_penalties = false;
      _enable_regularization = false;
      _dualres(_qp_solution, _step_residuals);
      TRACTOR_CHECK_ALL_FINITE(_step_residuals);
      // _linear_solver.setMaxIterations(1);
      _linear_solver.compute(_hgrad);
      _qp_solution =
          _linear_solver.solveWithGuess(_step_residuals, _qp_solution);
      TRACTOR_CHECK_ALL_FINITE(_qp_solution);
    }

    Scalar padding = _constraint_padding;

    // Scalar padding = 10;

    // for (size_t iteration = 0; iteration < 100; iteration++) {
    for (size_t iteration = 0; 1; iteration++) {
      // std::cout << "iteration " << iteration << std::endl;
      TRACTOR_LOG_VAR(iteration);
      TRACTOR_LOG_VAR(padding);

      _step_solution = _qp_solution;

      _x_penalty_init->parameterVector(std::array<Scalar, 1>({padding}),
                                       _memory);
      // padding *= 2;

      {
        TRACTOR_PROFILER("solve constraints");
        _enable_objectives = false;
        _enable_penalties = true;
        _enable_regularization = true;
        _dualres(_step_solution, _step_residuals);
        TRACTOR_CHECK_ALL_FINITE(_step_residuals);
        _linear_solver.setMaxIterations(_dual_variable_count * 2);
        //_linear_solver.setMaxIterations(1);
        _linear_solver.compute(_hgrad);
        _step_solution =
            _linear_solver.solveWithGuess(_step_residuals, _step_solution);
        TRACTOR_CHECK_ALL_FINITE(_step_solution);
      }

      /*{
        TRACTOR_PROFILER("solve constraints");
        _enable_objectives = false;
        _enable_penalties = true;
        _enable_regularization = false;
        _dualres(_step_solution, _step_residuals);
        TRACTOR_CHECK_ALL_FINITE(_step_residuals);
        _linear_solver.setMaxIterations(1);
        _linear_solver.compute(_hgrad);
        _step_solution =
            _linear_solver.solveWithGuess(_step_residuals, _step_solution);
        TRACTOR_CHECK_ALL_FINITE(_step_solution);
      }

      {
        TRACTOR_PROFILER("solve projection");
        _enable_objectives = false;
        _enable_penalties = false;
        _enable_regularization = true;
        _dualres(_step_solution, _step_residuals);
        TRACTOR_CHECK_ALL_FINITE(_step_residuals);
        _linear_solver.setMaxIterations(_dual_variable_count * 2);
        _linear_solver.compute(_hgrad);
        _step_solution =
            _linear_solver.solveWithGuess(_step_residuals, _step_solution);
        TRACTOR_CHECK_ALL_FINITE(_step_solution);
    }*/

      {
        Scalar line_search_result = 1;
        {
          auto df = [&](const Scalar &v) {
            _line_search_temp_solution =
                _qp_solution + (_step_solution - _qp_solution) * v;
            _dualres(_line_search_temp_solution, _line_search_temp_residuals);
            _dualprop(_line_search_temp_solution, _line_search_temp_gradients);
            Scalar ret =
                (_line_search_temp_residuals - _line_search_temp_gradients)
                    .squaredNorm();
            if (!std::isfinite(ret)) {
              ret = std::numeric_limits<Scalar>::max();
            }
            return ret;
          };
          {
            TRACTOR_PROFILER("qp bisection search");
            line_search_result =
                minimizeTernary(df, tolerance(), Scalar(0), Scalar(100));
            TRACTOR_LOG_VAR(line_search_result);
          }
          // line_search_result *= 0.5;
        }
        _step_solution =
            _qp_solution + (_step_solution - _qp_solution) * line_search_result;

        // if (line_search_result <= tolerance()) {
        //  padding *= 2;
        //}
      }

      /*
      TRACTOR_LOG_VEC(_step_solution);
      TRACTOR_LOG_VAR(tolerance());
      TRACTOR_LOG_VAR(_step_solution.norm());
      bool ok = ((_step_solution - _qp_solution).squaredNorm() <
                 tolerance() * tolerance());
      _qp_solution = _step_solution;
      if (ok) {
        break;
      }
      */

      {
        TRACTOR_PROFILER("project inequality constraints");
        _x_project->parameterVector(
            std::array<Scalar, 1>({_constraint_padding * Scalar(0.5)}),
            _memory);
        //_x_project->parameterVector(
        //    std::array<Scalar, 1>({_constraint_padding}), _memory);
        _x_project->run(_step_solution.head(_primal_variable_count), _memory,
                        _v_project);
        for (size_t i = 0; i < _primal_variable_count; i++) {
          std::cout << i << " " << _p_fprop.input(i).name() << " "
                    << (_step_solution[i] - _v_project[i]) << "         "
                    << (_nonlinear_solution[i] + _step_solution[i]) << " "
                    << (_nonlinear_solution[i] + _v_project[i]) << std::endl;
        }
      }

      _qp_solution = _step_solution;

      {
        Scalar err = (_v_project - _step_solution.head(_primal_variable_count))
                         .squaredNorm();
        TRACTOR_LOG_VAR(err);
        bool ok = (err < tolerance() * tolerance());
        if (ok) {
          break;
        }
      }

      _step_solution.head(_primal_variable_count) = _v_project;
    }

    _previous_nonlinear_solution = _nonlinear_solution;
    accumulate(_nonlinear_solution, _qp_solution);
    return (_nonlinear_solution - _previous_nonlinear_solution).squaredNorm();
  }

public:
  PenaltySolver(const std::shared_ptr<Engine> &engine) : SolverBase(engine) {

    _dual_matrix_replacement = std::make_shared<DualMatrixReplacement>(this);
    _hgrad = MatrixReplacement<Scalar>(_dual_matrix_replacement);
  }
};

} // namespace tractor
