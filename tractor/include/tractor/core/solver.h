// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/core/program.h>

#include <memory>

#include <chrono>

namespace tractor {

class Engine;
class Executable;

class Solver {
  std::vector<Program::Input> _inputs;
  std::vector<Program::Parameter> _parameters;
  Buffer _buffer;
  double _tolerance = 1e-9;
  double _timeout = -1;
  // double _loss = -1;
  bool _compiled = false;
  void _checkCompiled() const {
    if (!_compiled) {
      throw std::runtime_error("call compile(...) before use");
    }
  }
  std::chrono::steady_clock::time_point _start_time;
  bool _first_step = false;
  bool _hard_timeout = false;

protected:
  std::shared_ptr<Engine> _engine;
  virtual void _compile(const Program &prog) = 0;
  virtual void _parameterize(const Buffer &buffer) = 0;
  virtual void _input(const Buffer &buffer) = 0;
  virtual void _output(Buffer &buffer) = 0;
  virtual double _step() = 0;
  void _log(const char *label, const Program &prog);
  // void _loss(double loss) { _current_loss = loss; }
  // bool _stop() const { return (_current_loss < _stop_tolerance); }
  bool _expired() const;

public:
  Solver(const std::shared_ptr<Engine> &engine);
  Solver(const Solver &other) = delete;
  Solver &operator=(const Solver &other) = delete;
  void compile(const Program &prog);
  void parameterize(const Buffer &buffer);
  void parameterize();
  void input(const Buffer &buffer);
  void output(Buffer &buffer);
  void gather();
  void scatter();
  void step();
  void solve();
  double tolerance() const { return _tolerance; }
  // double loss() const { return _loss; }
  void setTolerance(double v) { _tolerance = v; }
  void setTimeout(double v, bool hard) {
    _timeout = v;
    _hard_timeout = hard;
  }
  // double timeout() const { return _timeout; }
  void clearTimeout() { _timeout = 0; }
  virtual double loss() const { return -1; }
};

} // namespace tractor
