// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/solvers/base.h>

namespace tractor {

template <class Scalar> class InteriorPointSolver : public SolverBase {

public:
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
  Eigen::MatrixXd _m_fprop, _m_bprop;

  Scalar _initial_barrier_weight = Scalar(1);
  Scalar _min_barrier_weight = Scalar(1e-3);
  Scalar _current_barrier_weight_start = Scalar(0);
  Scalar _current_barrier_weight = Scalar(0);

  Scalar _barrier_decrease = Scalar(0.5);
  Scalar _barrier_increase = Scalar(2);

  // Scalar _barrier_decrease = Scalar(0.1);
  // Scalar _barrier_increase = Scalar(10);

  Scalar _constraint_padding = Scalar(0.1);

  Vector _qp_residuals;
  Vector _nonlinear_solution;
  Vector _previous_nonlinear_solution;
  Vector _vec_right;
  Vector _diag_in, _diag_out, _objective_diagonal;
  Vector _line_search_temp_solution;
  Vector _line_search_temp_residuals;
  Vector _line_search_temp_gradients;
  Vector _qp_solution;
  Vector _step_solution;
  Vector _step_solution_test;
  Vector _v_project, _v_barrier_input, _temp_residuals, _v_barrier_gradients,
      _v_barrier_diagonal, _step_residuals, _temp_residuals_2;
  Vector _current_step, _previous_step;

  Scalar _current_regularization = 0;

  size_t _primal_variable_count = 0;
  size_t _dual_variable_count = 0;

  bool _use_matrices = false;
  bool _use_barrier = true;
  bool _use_objectives = true;
  bool _use_constraints = true;
  bool _use_penalty = false;
  bool _in_constraint_phase = false;

  bool _use_preconditioner = true;

#if 0
  Scalar _computeBarrierWeight() const {
    if (_current_barrier_weight < 0) {
      return 0;
    }
    return std::min(Scalar(1), _current_barrier_weight);
  }
  Scalar _computeObjectiveWeight() const {
    if (_in_constraint_phase || !_use_objectives) {
      return 0;
    }
    if (_current_barrier_weight < 0) {
      return 0;
    }
    return std::min(Scalar(1), Scalar(1) / _current_barrier_weight);
  }
#endif

#if 0
  Scalar _computeBarrierWeight() const {
    if (_current_barrier_weight < 0) {
      return 0;
    }
    return _current_barrier_weight;
  }
  Scalar _computeObjectiveWeight() const {
    if (_in_constraint_phase || !_use_objectives) {
      return 0;
    }
    if (_current_barrier_weight < 0) {
      return 0;
    }
    return 0.1;
  }
#endif

#if 0
  Scalar _computeBarrierWeight() const {
    if (_current_barrier_weight < 0) {
      return 0;
    }
    return std::sqrt(_current_barrier_weight);
  }
  Scalar _computeObjectiveWeight() const {
    if (_in_constraint_phase || !_use_objectives) {
      return 0;
    }
    if (_current_barrier_weight < 0) {
      return 0;
    }
    return Scalar(1) / std::sqrt(_current_barrier_weight);
  }
#endif

#if 1
  Scalar _computeBarrierWeight() const {
    if (_current_barrier_weight < 0) {
      return 0;
    }
    if (_current_barrier_weight > Scalar(1)) {
      return std::sqrt(_current_barrier_weight);
    } else {
      return 1;
    }
  }
  Scalar _computeObjectiveWeight() const {
    if (_in_constraint_phase || !_use_objectives) {
      return 0;
    }
    if (_current_barrier_weight < 0) {
      return 0;
    }
    if (_current_barrier_weight > Scalar(1)) {
      return Scalar(1) / std::sqrt(_current_barrier_weight);
    } else {
      return Scalar(1) / _current_barrier_weight;
    }
  }
#endif

  template <class Input, class Output>
  void _fprop(Input &&input, Output &&output) {

    if (_use_matrices) {
      output = _m_fprop * input;
    } else {
      _x_fprop->run(input, _memory, output);
    }

    /*{
      TRACTOR_PROFILER("fprop matrix");
      output = _m_fprop * input;
    }
    {
      TRACTOR_PROFILER("fprop program");
      _x_fprop->run(input, _memory, output);
  }*/
  }

  template <class Input, class Output>
  void _bprop(Input &&input, Output &&output) {

    if (_use_matrices) {
      output = _m_bprop * input;
    } else {
      _x_bprop->run(input, _memory, output);
    }

    /*{
      TRACTOR_PROFILER("bprop matrix");
      output = _m_bprop * input;
    }
    {
      TRACTOR_PROFILER("bprop program");
      _x_bprop->run(input, _memory, output);
  }*/
  }

  Vector _dualprop_temp, _dualprop_temp_2;
  template <class Input, class Output>
  void _dualprop(Input &&input, Output &&output) {

    TRACTOR_CHECK_ALL_FINITE(input);

    Scalar objective_weight = _computeObjectiveWeight();
    Scalar barrier_weight = _computeBarrierWeight();

    output.setZero(_dual_variable_count);

    _fprop(input.head(_primal_variable_count), _dualprop_temp);

    if (_use_constraints) {
      for (size_t i = 0; i < _constraint_indices.size(); i++) {
        size_t j = _constraint_indices[i];
        output[_primal_variable_count + i] = _dualprop_temp[j];
      }
    }

    /*for (size_t i = 0; i < _primal_variable_count; i++) {
      if (_priority_list[i] <= 0) {
        _dualprop_temp[i] *= objective_weight;
      }
  }*/
    _dualprop_temp *= objective_weight;

    if (_use_constraints) {
      for (size_t i = 0; i < _constraint_indices.size(); i++) {
        size_t j = _constraint_indices[i];
        _dualprop_temp[j] += input[_primal_variable_count + i];
      }
    }

    _bprop(_dualprop_temp, output.head(_primal_variable_count));

    /*if (_use_barrier) {
      _x_barrier_step->run(input.head(_primal_variable_count), _memory,
                           _dualprop_temp);
      output.head(_primal_variable_count) += _dualprop_temp * barrier_weight;
  }*/

    if (_use_barrier) {
      _x_barrier_step->run(input.head(_primal_variable_count), _memory,
                           _dualprop_temp);
      // std::cout << "barrier step " << _dualprop_temp << std::endl;
      output.head(_primal_variable_count) += _dualprop_temp * barrier_weight;
      // output.head(_primal_variable_count) +=
      //      _dualprop_temp * (barrier_weight * barrier_weight);
    }

    output.head(_primal_variable_count) +=
        input.head(_primal_variable_count) * _current_regularization;

    // output += input * 1e-9;
  }

  template <class Output> void _dualdiagonal(Output &&output) {

    if (_use_preconditioner) {

      Scalar objective_weight = _computeObjectiveWeight();
      Scalar barrier_weight = _computeBarrierWeight();

      output.setZero(_dual_variable_count);

      output.head(_primal_variable_count) =
          _objective_diagonal * objective_weight;

      if (_use_barrier) {
        _x_barrier_diagonal->execute(_memory);
        _x_barrier_diagonal->outputVector(_memory, _dualprop_temp);
        // std::cout << "barrier diagonal " << _dualprop_temp << std::endl;
        output.head(_primal_variable_count) += _dualprop_temp * barrier_weight;
        // output.head(_primal_variable_count) +=
        //    _dualprop_temp * (barrier_weight * barrier_weight);
      }

      output.head(_primal_variable_count).array() += _current_regularization;

      // std::cout << output << std::endl;

    } else {

      output.setOnes(_dual_variable_count);
    }
  }

  Vector _dualres_temp;
  template <class Input, class Output>
  void _dualres(Input &&input, Output &&output) {

    Scalar objective_weight = _computeObjectiveWeight();
    Scalar barrier_weight = _computeBarrierWeight();

    output = -_qp_residuals;

    /*for (size_t i = 0; i < _primal_variable_count; i++) {
      if (_priority_list[i] <= 0) {
        output[i] *= objective_weight;
      }
  }*/
    output.head(_primal_variable_count) *= objective_weight;

    if (!_use_constraints) {
      for (size_t i = 0; i < _constraint_indices.size(); i++) {
        output[_primal_variable_count + i] = 0;
      }
    }

    if (_use_barrier) {

      _x_barrier_init->run(input.head(_primal_variable_count), _memory,
                           _dualres_temp);
      // std::cout << "barrier gradient " << _dualprop_temp << std::endl;
      // TRACTOR_LOG_VEC(_dualres_temp);
      output.head(_primal_variable_count) -= _dualres_temp * barrier_weight;

      _x_barrier_step->run(input.head(_primal_variable_count), _memory,
                           _dualres_temp);
      // std::cout << "barrier step " << _dualprop_temp << std::endl;
      // TRACTOR_LOG_VEC(_dualres_temp);
      output.head(_primal_variable_count) += _dualres_temp * barrier_weight;
    }

    output.head(_primal_variable_count) +=
        input.head(_primal_variable_count) * _current_regularization;
  }

  struct DualMatrixReplacement : public MatrixReplacement<Scalar>::Impl {
    InteriorPointSolver *_solver = nullptr;
    mutable Vector _diagonal_temp;
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
      _solver->_dualdiagonal(_diagonal_temp);
      return _diagonal_temp;
    }
    DualMatrixReplacement(InteriorPointSolver *solver) : _solver(solver) {}
  };

  struct DiagonalPreconditioner {
    Vector _inv_diag;
    bool _use_preconditioner = false;
    typedef typename Vector::StorageIndex StorageIndex;
    typedef typename Vector::Index Index;
    enum {
      ColsAtCompileTime = Eigen::Dynamic,
      MaxColsAtCompileTime = Eigen::Dynamic
    };
    Index rows() const { return _inv_diag.size(); }
    Index cols() const { return _inv_diag.size(); }
    DiagonalPreconditioner() {}
    DiagonalPreconditioner(const MatrixReplacement<Scalar> &m) {}
    auto &analyzePattern(const MatrixReplacement<Scalar> &m) { return *this; }
    auto &factorize(const MatrixReplacement<Scalar> &m) {
      auto &diagonal = m.diagonal();
      _inv_diag.resize(diagonal.size());
      for (size_t i = 0; i < _inv_diag.size(); i++) {
        Scalar v = diagonal[i];
        if (v != Scalar(0)) {
          v = Scalar(1) / v;
        } else {
          v = Scalar(1);
        }
        _inv_diag[i] = v;
      }
      if (!_inv_diag.allFinite()) {
        throw std::runtime_error("preconditioner not finite");
      }
      return *this;
    }
    auto &compute(const MatrixReplacement<Scalar> &m) { return factorize(m); }
    template <class A, class X> void _solve_impl(const A &a, X &x) const {
      if (_use_preconditioner) {
        x.array() = _inv_diag.array() * a.array();
      } else {
        x = a;
      }
    }
    template <class A> auto solve(const Eigen::MatrixBase<A> &a) const {
      return Eigen::Solve<DiagonalPreconditioner, A>(*this, a.derived());
    }
    Eigen::ComputationInfo info() const { return Eigen::Success; }
  };

  std::shared_ptr<DualMatrixReplacement> _dual_matrix_replacement;
  MatrixReplacement<Scalar> _hgrad;
  typedef Eigen::ConjugateGradient<MatrixReplacement<Scalar>,
                                   Eigen::Lower | Eigen::Upper,
                                   DiagonalPreconditioner>
      LinearSolver;
  LinearSolver _linear_solver;

protected:
  virtual void _compile(const Program &prog) override {

    _compileGradients<Scalar>(prog);

    _primal_variable_count = _x_fprop->inputBufferSize() / sizeof(Scalar);
    TRACTOR_LOG_VAR(_primal_variable_count);

    _dual_variable_count = _primal_variable_count + _constraint_indices.size();
    TRACTOR_LOG_VAR(_dual_variable_count);

    std::cout << _p_accu << std::endl;
    // throw 0;
  }

  virtual void _input(const Buffer &buffer) override {
    buffer.toVector(_p_prog.inputs(), _nonlinear_solution);
    _current_barrier_weight_start = _initial_barrier_weight;
  }

  virtual void _output(Buffer &buffer) override {
    buffer.fromVector(_p_prog.inputs(), _nonlinear_solution);
  }

  virtual void _parameterize(const Buffer &buffer) override {
    _x_prog->parameterize(buffer, _memory);
  }

  virtual double _step() override {

    TRACTOR_PROFILER("step");

    _linear_solver.setTolerance(tolerance() * 0.1);

    _current_regularization = 0;

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

    if (_use_preconditioner) {
      _diag_in.setZero(_primal_variable_count);
      _objective_diagonal.setZero(_primal_variable_count);
      /*
      {
        TRACTOR_PROFILER("preconditioning1");
        for (size_t i = 0; i < _primal_variable_count; i++) {
          _diag_in[i] = Scalar(1);
          _x_hprop->run(_diag_in, _memory, _diag_out);
          _objective_diagonal[i] = _diag_out[i];
          _diag_in[i] = Scalar(0);
        }
      }
      */
      {
        TRACTOR_PROFILER("preconditioning2");
        for (size_t i = 0; i < _primal_variable_count; i++) {
          _diag_in[i] = Scalar(1);
          _x_fprop->run(_diag_in, _memory, _diag_out);
          Scalar v = _diag_out.dot(_diag_out);
          // std::cout << v << " " << _objective_diagonal[i] << std::endl;
          _objective_diagonal[i] = v;
          _diag_in[i] = Scalar(0);
        }
      }
    } else {
      _objective_diagonal.setZero(_qp_residuals.size());
    }

    _qp_solution.setZero(_qp_residuals.size());
    _step_solution = _qp_solution;
    _previous_step = _qp_solution;

    /*
    {
      TRACTOR_PROFILER("project inequality constraints");
      _x_project->parameterVector(std::array<Scalar, 1>({_constraint_padding}),
                                  _memory);
      _x_project->run(_step_solution.head(_primal_variable_count), _memory,
                      _step_solution.head(_primal_variable_count));
    }
    */

#if 0
    {
      _in_constraint_phase = true;
      while (true) {
        _step_solution = _qp_solution;
        _dualres(_step_solution, _step_residuals);
        TRACTOR_CHECK_ALL_FINITE(_step_residuals);
        /*{
          TRACTOR_PROFILER("project inequality constraints");
          _x_project->parameterVector(
              std::array<Scalar, 1>({_constraint_padding}), _memory);
          _x_project->run(_step_solution.head(_primal_variable_count), _memory,
                          _step_solution.head(_primal_variable_count));
        }*/
        {
          TRACTOR_PROFILER("phase one");
          _linear_solver.compute(_hgrad);
          _step_solution =
              _linear_solver.solveWithGuess(_step_residuals, _step_solution);
          TRACTOR_CHECK_ALL_FINITE(_step_solution);
        }

        /*
        Scalar err = (_step_solution - _qp_solution).squaredNorm();
        // Scalar err = _step_residuals.squaredNorm();
        TRACTOR_LOG_VAR(err);
        bool ok = (err <= tolerance() * tolerance());
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
          _x_project->run(_step_solution.head(_primal_variable_count), _memory,
                          _v_project);
          /*for (size_t i = 0; i < _primal_variable_count; i++) {
            std::cout << i << " " << _p_fprop.input(i).name() << " "
                      << (_step_solution[i] - _v_project[i]) << "         "
                      << (_nonlinear_solution[i] + _step_solution[i]) << " "
                      << (_nonlinear_solution[i] + _v_project[i]) << std::endl;
          }*/
        }

        _qp_solution = _step_solution;

        {
          Scalar err =
              (_v_project - _step_solution.head(_primal_variable_count))
                  .squaredNorm();
          TRACTOR_LOG_VAR(err);
          bool ok = (err < tolerance() * tolerance());
          if (ok) {
            break;
          }
        }
      }
      _in_constraint_phase = false;
    }
#endif

#if 0
    for (size_t i = 0; i < _primal_variable_count; i++) {
      std::cout << i << " " << _p_prog.input(i).name() << " " << _qp_solution[i]
                << " " << _step_solution[i] << " "
                << (_nonlinear_solution[i] + _qp_solution[i]) << " "
                << (_nonlinear_solution[i] + _step_solution[i]) << std::endl;
    }
#endif

#if 0
    while (true) {
      TRACTOR_LOG_VAR(_constraint_padding);
      _step_solution = _qp_solution;
      {
        TRACTOR_PROFILER("project inequality constraints");
        _x_project->parameterVector(
            std::array<Scalar, 1>({_constraint_padding}), _memory);
        _x_project->run(_step_solution.head(_primal_variable_count), _memory,
                        _step_solution.head(_primal_variable_count));
      }
      {
        TRACTOR_PROFILER("project equality constraints");
        _in_constraint_phase = true;
        _dualres(_step_solution, _step_residuals);
        _linear_solver.compute(_hgrad);
        _step_solution =
            _linear_solver.solveWithGuess(_step_residuals, _step_solution);
        TRACTOR_CHECK_ALL_FINITE(_step_solution);
      }
      _qp_solution = _step_solution;
      {
        TRACTOR_PROFILER("project inequality constraints");
        _x_project->parameterVector(
            std::array<Scalar, 1>({_constraint_padding * 0.5}), _memory);
        _x_project->run(_step_solution.head(_primal_variable_count), _memory,
                        _step_solution.head(_primal_variable_count));
      }
      {
        Scalar err = (_qp_solution - _step_solution).squaredNorm();
        TRACTOR_LOG_VAR(err);
        // if (err <= tolerance() * tolerance()) {
        if (err <= 0.01) {
          break;
        }
      }
    }
#endif

#if 1
    {
      TRACTOR_PROFILER("project inequality constraints");
      _x_project->parameterVector(std::array<Scalar, 1>({_constraint_padding}),
                                  _memory);
      _x_project->run(_qp_solution.head(_primal_variable_count), _memory,
                      _qp_solution.head(_primal_variable_count));
      TRACTOR_CHECK_ALL_FINITE(_qp_solution);
    }
#endif

    _current_barrier_weight = _current_barrier_weight_start;

    size_t iteration_count = 0;

    _in_constraint_phase = false;

    if (1) {
      while (_current_barrier_weight >= _min_barrier_weight * Scalar(0.99)) {

        TRACTOR_LOG_VAR(iteration_count);
        TRACTOR_LOG_VAR(_current_barrier_weight);

        TRACTOR_CHECK_FINITE(_current_barrier_weight);

        /*if (_expired()) {
          return -1;
      }*/

        /*
        if (iteration_count == 2) {
          _current_regularization = tolerance();
        }
        */

        _step_solution = _qp_solution;

        /*
        _in_constraint_phase = true;
        _dualres(_qp_solution, _step_residuals);
        TRACTOR_CHECK_ALL_FINITE(_step_residuals);
        {
          TRACTOR_PROFILER("solve linear");
          _linear_solver.compute(_hgrad);
          _step_solution =
              _linear_solver.solveWithGuess(_step_residuals, _step_solution);
          TRACTOR_CHECK_ALL_FINITE(_step_solution);
        }
        */

        _in_constraint_phase = false;
        _dualres(_step_solution, _step_residuals);
        if (!_step_residuals.allFinite()) {

          std::cout << std::endl;

          {
            auto it_nonlinear = _p_prog.inputs().begin();
            auto it_linear = _p_fprop.inputs().begin();
            for (size_t i = 0; i < _p_prog.inputs().size(); i++) {
              auto &nonlinear_input = *it_nonlinear;
              auto &linear_input = *it_linear;
              std::cout << "nl " << i << " " << nonlinear_input.name() << " ";
              for (size_t j = 0; j < nonlinear_input.size() / sizeof(Scalar);
                   j++) {
                std::cout << " "
                          << _nonlinear_solution[nonlinear_input.offset() /
                                                     sizeof(Scalar) +
                                                 j];
              }
              std::cout << std::endl;
              ++it_nonlinear;
              ++it_linear;
            }

            std::cout << std::endl;
          }

          {
            auto it_nonlinear = _p_prog.inputs().begin();
            auto it_linear = _p_fprop.inputs().begin();
            for (size_t i = 0; i < _p_prog.inputs().size(); i++) {
              auto &nonlinear_input = *it_nonlinear;
              auto &linear_input = *it_linear;
              std::cout << "solution " << i << " " << nonlinear_input.name()
                        << " ";
              for (size_t j = 0; j < linear_input.size() / sizeof(Scalar);
                   j++) {
                std::cout
                    << " "
                    << _step_solution[linear_input.offset() / sizeof(Scalar) +
                                      j];
              }
              std::cout << std::endl;
              ++it_nonlinear;
              ++it_linear;
            }
            std::cout << std::endl;
          }

          {
            auto it_nonlinear = _p_prog.inputs().begin();
            auto it_linear = _p_fprop.inputs().begin();
            for (size_t i = 0; i < _p_prog.inputs().size(); i++) {
              auto &nonlinear_input = *it_nonlinear;
              auto &linear_input = *it_linear;
              std::cout << "residual " << i << " " << nonlinear_input.name()
                        << " ";
              for (size_t j = 0; j < linear_input.size() / sizeof(Scalar);
                   j++) {
                std::cout
                    << " "
                    << _step_residuals[linear_input.offset() / sizeof(Scalar) +
                                       j];
              }
              std::cout << std::endl;
              ++it_nonlinear;
              ++it_linear;
            }
            std::cout << std::endl;
          }
        }
        TRACTOR_CHECK_ALL_FINITE(_step_residuals);
        TRACTOR_CHECK_ALL_FINITE(_qp_solution);
        {
          TRACTOR_PROFILER("solve linear");
          _linear_solver.compute(_hgrad);
          _step_solution =
              _linear_solver.solveWithGuess(_step_residuals, _qp_solution);
          TRACTOR_CHECK_ALL_FINITE(_step_solution);
        }

        Scalar line_search_result = 1;

        if (iteration_count == 0) {

          _x_barrier_init->run(_step_solution.head(_primal_variable_count),
                               _memory, _temp_residuals);

          _current_barrier_weight *= _barrier_increase;

          if (_temp_residuals.allFinite()) {
            iteration_count++;
          }

          continue;

        } else if (iteration_count == 1) {

          _x_barrier_init->run(_step_solution.head(_primal_variable_count),
                               _memory, _temp_residuals);

          if (!_temp_residuals.allFinite()) {
            _current_barrier_weight *= _barrier_increase;
            continue;
          }

        } else {

          if (0) {
            while (true) {
              _v_barrier_input =
                  _qp_solution.head(_primal_variable_count) +
                  (_step_solution - _qp_solution).head(_primal_variable_count) *
                      line_search_result;
              _x_barrier_init->run(_v_barrier_input, _memory, _temp_residuals);
              line_search_result *= 0.9;
              if (_temp_residuals.allFinite()) {
                break;
              }
            }
            // line_search_result *= 0.1;
          }

          if (0) {
            auto check = [&](const Scalar &f) {
              _v_barrier_input =
                  _qp_solution.head(_primal_variable_count) +
                  (_step_solution - _qp_solution).head(_primal_variable_count) *
                      f;
              _x_barrier_init->run(_v_barrier_input, _memory, _temp_residuals);
              return _temp_residuals.allFinite();
            };
            line_search_result = 1;
            while (true) {
              if (check(line_search_result)) {
                break;
              }
              line_search_result *= 0.5;
            }
            if (line_search_result < 1 || !check(2)) {
              line_search_result *= 0.5;
            }
            // line_search_result *= 0.1;
          }

          if (0) {
            auto df = [&](const Scalar &v) {
              _v_barrier_input =
                  _qp_solution.head(_primal_variable_count) +
                  (_step_solution - _qp_solution).head(_primal_variable_count) *
                      line_search_result;
              _x_barrier_init->run(_v_barrier_input, _memory, _temp_residuals);
              /*
              if (_temp_residuals.allFinite()) {
                return 10 - v;
              }
              return -std::numeric_limits<Scalar>::max();
              */
              if (_temp_residuals.allFinite()) {
                return v - 10;
              }
              return Scalar(10);
            };
            line_search_result =
                rootBisect(df, tolerance(), Scalar(0), Scalar(2));
            line_search_result *= 0.5;
          }

          if (0) {
            auto df = [&](const Scalar &v) {
              _line_search_temp_solution =
                  _qp_solution + (_step_solution - _qp_solution) * v;
              _dualres(_line_search_temp_solution, _line_search_temp_residuals);
              _dualprop(_line_search_temp_solution,
                        _line_search_temp_gradients);
              Scalar ret =
                  (_line_search_temp_residuals - _line_search_temp_gradients)
                      .squaredNorm();
              if (!std::isfinite(ret)) {
                ret = std::numeric_limits<Scalar>::max();
              }
              return ret;
            };
            if (1) {
              TRACTOR_PROFILER("qp line search");
              line_search_result =
                  minimizeTernary(df, tolerance(), Scalar(0), Scalar(1));
              // if (line_search_result < tolerance()) {
              //    line_search_result = 0;
              //}
            }
          }

          if (0) {
            auto df = [&](const Scalar &v) {
              _line_search_temp_solution =
                  _qp_solution + (_step_solution - _qp_solution) * v;
              _dualres(_line_search_temp_solution, _line_search_temp_residuals);
              _dualprop(_line_search_temp_solution,
                        _line_search_temp_gradients);
              return Scalar((_qp_solution - _step_solution)
                                .dot(_line_search_temp_residuals -
                                     _line_search_temp_gradients));
            };
            if (1) {
              TRACTOR_PROFILER("qp bisection search");
              line_search_result =
                  rootBisect(df, tolerance(), Scalar(0), Scalar(1));
              std::cout << "line_search_result " << line_search_result
                        << std::endl;
            }
            if (1) {
              for (double p = 0.0; p <= 1.00001; p += 0.1) {
                std::cout << df(p) << " ";
              }
              std::cout << std::endl;
            }
          }

          if (0) {
            auto df = [&](const Scalar &v) {
              _line_search_temp_solution =
                  _qp_solution + (_step_solution - _qp_solution) * v;
              _dualres(_line_search_temp_solution, _line_search_temp_residuals);
              return Scalar((_step_solution - _qp_solution)
                                .head(_primal_variable_count)
                                .dot(_line_search_temp_residuals.head(
                                    _primal_variable_count)));
            };
            if (1) {
              TRACTOR_PROFILER("qp bisection search");
              line_search_result =
                  rootBisect(df, tolerance(), Scalar(0), Scalar(1));
              std::cout << "line_search_result " << line_search_result
                        << std::endl;
            }
            if (1) {
              for (double p = 0.0; p <= 1.00001; p += 0.1) {
                std::cout << df(p) << " ";
              }
              std::cout << std::endl;
            }
          }

          if (0) {
            auto df = [&](const Scalar &v) {
              Scalar objective_weight = _computeObjectiveWeight();
              Scalar barrier_weight = _computeBarrierWeight();

              _line_search_temp_solution =
                  _qp_solution + (_step_solution - _qp_solution) * v;

              Scalar ret = 0;

              ret += (_line_search_temp_solution - _qp_residuals)
                         .head(_primal_variable_count)
                         .dot((_step_solution - _qp_solution)
                                  .head(_primal_variable_count)) *
                     objective_weight;

              if (_use_barrier) {
                _x_barrier_init->run(
                    _line_search_temp_solution.head(_primal_variable_count),
                    _memory, _dualres_temp);
                ret += _dualres_temp.dot((_step_solution - _qp_solution)
                                             .head(_primal_variable_count)) *
                       barrier_weight;
              }

              return -ret;
            };
            if (1) {
              TRACTOR_PROFILER("qp bisection search");
              line_search_result =
                  rootBisect(df, tolerance(), Scalar(0), Scalar(1));
              std::cout << "line_search_result " << line_search_result
                        << std::endl;
            }
            if (1) {
              for (double p = 0.0; p <= 1.00001; p += 0.1) {
                std::cout << df(p) << " ";
              }
              std::cout << std::endl;
            }
          }

          if (1) {
            auto df = [&](const Scalar &v) {

              /*Scalar objective_weight = _computeObjectiveWeight();
              Scalar barrier_weight = _computeBarrierWeight();

              _line_search_temp_solution =
                  _qp_solution + (_step_solution - _qp_solution) * v;

              Scalar ret = 0;

              ret += (_line_search_temp_solution - _qp_residuals)
                         .head(_primal_variable_count)
                         .dot((_step_solution - _qp_solution)
                                  .head(_primal_variable_count)) *
                     objective_weight;

              if (_use_barrier) {
                _x_barrier_init->run(
                    _line_search_temp_solution.head(_primal_variable_count),
                    _memory, _dualres_temp);
                ret += _dualres_temp.dot((_step_solution - _qp_solution)
                                             .head(_primal_variable_count)) *
                       barrier_weight;
              }

              return ret * ret;
              */

              _line_search_temp_solution =
                  _qp_solution + (_step_solution - _qp_solution) * v;
              _dualres(_line_search_temp_solution, _line_search_temp_residuals);
              _dualprop(_line_search_temp_solution,
                        _line_search_temp_gradients);
              Scalar ret = Scalar((_qp_solution - _step_solution)
                                      .head(_primal_variable_count)
                                      .dot((_line_search_temp_residuals -
                                            _line_search_temp_gradients)
                                               .head(_primal_variable_count)));

              /*
_line_search_temp_solution =
_qp_solution + (_step_solution - _qp_solution) * v;
_dualres(_line_search_temp_solution, _line_search_temp_residuals);
Scalar ret = Scalar((_qp_solution - _step_solution)
     .head(_primal_variable_count)
     .dot((_line_search_temp_residuals)
              .head(_primal_variable_count)));
*/

              // return std::abs(ret);
              return ret;
            };
            if (1) {
              TRACTOR_PROFILER("qp line search");
              // line_search_result =
              // minimizeTernary(df, tolerance(), Scalar(0), Scalar(1));
              //  minimizeTernary(df, 0.01, Scalar(0), Scalar(1)) * 0.99;
              // line_search_result = std::max(
              //      Scalar(0), rootBisect(df, 0.1, Scalar(0), Scalar(1)) -
              //      0.1);

              // line_search_result =
              //      std::max(Scalar(0),
              //               rootBisect(df, 0.001, Scalar(0), Scalar(1)) -
              //               0.001);

              // line_search_result = std::max(
              //      Scalar(0), rootBisect(df, 0.01, Scalar(0), Scalar(1)) -
              //      0.01);

              {
                Scalar x = 1;
                while (!(df(x) < 0)) {
                  x *= 0.5;
                  if (x >= tolerance()) {
                    continue;
                  } else {
                    x = 0;
                    break;
                  }
                }
                if (x > 0) {
                  x = rootBisect(df, x * 0.1, Scalar(0), x * 2);
                }
                line_search_result = x;
              }

              // line_search_result = rootBisect(df, 0.01, Scalar(0),
              // Scalar(1));
              std::cout << "line_search_result " << line_search_result
                        << std::endl;
            }
            if (0) {
              for (double p = 0.0; p <= 1.00001; p += 0.1) {
                std::cout << df(p) << " ";
              }
              std::cout << std::endl;
            }
          }
        }

        TRACTOR_LOG_VAR(line_search_result);

        _current_step = _step_solution - _qp_solution;

        /*
        double step_dot = _current_step.head(_primal_variable_count)
                              .dot(_previous_step.head(_primal_variable_count));
        TRACTOR_LOG_VAR(step_dot);
        TRACTOR_LOG_VAR(_current_regularization);
        if (step_dot < 0) {
          _current_regularization *= Scalar(3);
        } else {
          _current_regularization =
              std::max(tolerance(), _current_regularization * Scalar(0.5));
        }
        _previous_step = _current_step;
        */

        _current_step *= line_search_result;

        _step_solution = _qp_solution + _current_step;

        Scalar step_size_sq =
            _current_step.head(_primal_variable_count).squaredNorm();
        Scalar step_size = sqrt(step_size_sq);
        TRACTOR_LOG_VAR(step_size);

        _qp_solution = _step_solution;

        iteration_count++;

        Scalar tol = tolerance() *
                     std::max(Scalar(1), std::max(_computeBarrierWeight(),
                                                  _computeObjectiveWeight()));
        if (step_size_sq <= tol * tol /*||
            line_search_result <= tolerance()*/) {
          _current_barrier_weight *= _barrier_decrease;
        }
      }
    }

    TRACTOR_CHECK_ALL_FINITE(_qp_solution);

    _previous_nonlinear_solution = _nonlinear_solution;
    accumulate(_nonlinear_solution, _qp_solution);

    TRACTOR_CHECK_ALL_FINITE(_nonlinear_solution);

    std::cout << "finished" << std::endl;

    return (_nonlinear_solution - _previous_nonlinear_solution).squaredNorm();
  }

public:
  InteriorPointSolver(const std::shared_ptr<Engine> &engine)
      : SolverBase(engine) {

    _dual_matrix_replacement = std::make_shared<DualMatrixReplacement>(this);
    _hgrad = MatrixReplacement<Scalar>(_dual_matrix_replacement);
  }
};

} // namespace tractor
