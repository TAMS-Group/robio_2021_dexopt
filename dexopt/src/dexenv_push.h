// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "dexenv_grasp.h"

namespace tractor {

template <class ValueSingle, class ValueBatch>
struct DexEnvPush : tractor::DexEnvGrasp<ValueSingle, ValueBatch> {

  typedef tractor::Var<ValueSingle> ScalarSingle;
  typedef tractor::GeometryFast<ScalarSingle> GeometrySingle;

  typedef tractor::Var<ValueBatch> ScalarBatch;
  typedef tractor::GeometryFast<ScalarBatch> GeometryBatch;

  DexEnvPush() { this->_info.name = "push"; }

  virtual void
  goals(tractor::DexLearn<ValueSingle, ValueBatch> &dexlearn) override {

    dexlearn.addGoal(std::allocate_shared<tractor::MoveGoal<GeometryBatch>>(
        tractor::AlignedStdAlloc<tractor::MoveGoal<GeometryBatch>>(), "object",
        dexlearn.frameCount() - 1,
        GeometryBatch::pack(ValueBatch(0.0), ValueBatch(0.1), ValueBatch(0.0)),
        ValueBatch(2)));

    dexlearn.addGoal(
        std::allocate_shared<tractor::RelativeOrientationGoal<GeometryBatch>>(
            tractor::AlignedStdAlloc<
                tractor::RelativeOrientationGoal<GeometryBatch>>(),
            "object",
            GeometryBatch::pack(ValueBatch(0.0), ValueBatch(0.0),
                                ValueBatch(0.0)),
            ValueBatch(1)));
  }
};

} // namespace tractor
