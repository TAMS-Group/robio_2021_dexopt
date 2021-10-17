// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/core/operator.h>

#include <unordered_set>

namespace tractor {

void buildGradients(const Program &src, Program &prep, Program *fprop,
                    Program *bprop, Program *hessian = nullptr,
                    Program *accumulate = nullptr);

/*
void buildDual(const Program &prog, const Program &fprop, const Program &bprop,
               const TypeInfo &parameter_type, Program &gradients,
               Program &residual);
*/

void buildConstraints(const Program &prog, const Program &fprop,
                      const Program &bprop, const Program &hprop,
                      const TypeInfo &padding_type, Program *proj,
                      Program *barrier_init, Program *barrier_step,
                      Program *barrier_diagonal, Program *penalty_init,
                      Program *penalty_step, Program *penalty_diagonal);

} // namespace tractor
