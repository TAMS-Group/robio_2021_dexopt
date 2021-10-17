// (c) 2020-2021 Philipp Ruppel

#include <tractor/core/solver.h>

#include <tractor/core/engine.h>

#include <chrono>

namespace tractor {

Solver::Solver(const std::shared_ptr<Engine> &engine) : _engine(engine) {
  //_executable = engine->createExecutable();
}

void Solver::_log(const char *label, const Program &prog) {
  // std::cout << "solver program " << label << " " << typeid(*this).name() <<
  // "\n"
  //            << prog << std::endl;
}

void Solver::compile(const Program &prog) {
  //_log("prog", prog);
  _inputs.assign(&*prog.inputs().begin(), &*prog.inputs().end());
  _parameters.assign(&*prog.parameters().begin(), &*prog.parameters().end());
  //_executable->compile(prog);
  _compile(prog);
  _compiled = true;
}

void Solver::parameterize(const Buffer &buffer) {
  _checkCompiled();
  _parameterize(buffer);
}

void Solver::input(const Buffer &buffer) {
  _checkCompiled();
  _input(buffer);
}

void Solver::output(Buffer &buffer) {
  _checkCompiled();
  _output(buffer);
}

void Solver::parameterize() {
  _checkCompiled();

  _buffer.gather(_parameters);
  _parameterize(_buffer);
}

void Solver::gather() {
  _checkCompiled();

  _buffer.gather(_parameters);
  _parameterize(_buffer);

  _buffer.gather(_inputs);
  _input(_buffer);
}

void Solver::scatter() {
  _checkCompiled();
  _output(_buffer);
  _buffer.scatter(_inputs);
}

void Solver::step() {
  _start_time = std::chrono::steady_clock::now();
  _first_step = true;
  _step();
}

bool Solver::_expired() const {
  return (!_first_step || _hard_timeout) && _timeout > 0 &&
         (std::chrono::steady_clock::now() >
          _start_time + std::chrono::duration<double>(_timeout));
}

void Solver::solve() {
  _start_time = std::chrono::steady_clock::now();
  _first_step = true;
  while (true) {
    double step = _step();
    _first_step = false;
    /*if (step < _tolerance * _tolerance) {
      std::cout << "converged" << std::endl;
      break;
  }*/
    /*auto t = std::chrono::steady_clock::now();
    std::cout << "time "
              << std::chrono::duration<double>(t - _start_time).count()
              << std::endl;
    if (_timeout >= 0 &&
        t > _start_time + std::chrono::duration<double>(_timeout)) {
      std::cout << "timeout" << std::endl;
      break;
  }*/
    if (_expired()) {
      break;
    }
  }
}

} // namespace tractor
