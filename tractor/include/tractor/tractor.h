// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/collision/query.h>
#include <tractor/collision/robot.h>

#include <tractor/core/constraints.h>
#include <tractor/core/eigen.h>
#include <tractor/core/engine.h>
#include <tractor/core/gradients.h>
#include <tractor/core/operator.h>
#include <tractor/core/profiler.h>
#include <tractor/core/var.h>

#include <tractor/solvers/ec.h>
#include <tractor/solvers/gd.h>
#include <tractor/solvers/ip.h>
#include <tractor/solvers/pgd.h>
#include <tractor/solvers/ps.h>
#include <tractor/solvers/sq.h>

#include <tractor/engines/jit.h>
#include <tractor/engines/loop.h>
#include <tractor/engines/simple.h>
#include <tractor/engines/test.h>

#include <tractor/robot/robot.h>

#include <tractor/geometry/eigen.h>
#include <tractor/geometry/fast.h>

#include <tractor/test/test.h>
