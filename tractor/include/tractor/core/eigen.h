// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/core/engine.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace tractor {
template <class Scalar> class MatrixReplacement;

// template <class T> using AlignedStdAlloc = Eigen::aligned_allocator<T>;
// template <class T> using AlignedStdVector = std::vector<T,
// AlignedStdAlloc<T>>;

} // namespace tractor

namespace Eigen {
namespace internal {
template <class Scalar>
struct traits<tractor::MatrixReplacement<Scalar>>
    : public Eigen::internal::traits<Eigen::SparseMatrix<Scalar>> {};
} // namespace internal
} // namespace Eigen

namespace tractor {
template <class ScalarType>
class MatrixReplacement
    : public Eigen::EigenBase<MatrixReplacement<ScalarType>> {

public:
  typedef ScalarType Scalar;
  typedef ScalarType RealScalar;
  typedef size_t StorageIndex;
  typedef size_t Index;
  typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> Vector;

  struct Impl {
    typedef MatrixReplacement<ScalarType>::Vector Vector;
    virtual size_t rows() const = 0;
    virtual size_t cols() const = 0;
    virtual void mul(const Vector &input, Vector &output) const = 0;
    virtual const Vector &diagonal() const {
      throw std::runtime_error("diagonal not implemented");
    }
  };

  class ExecutableImpl : public Impl {
  protected:
    std::shared_ptr<Executable> _executable;
    std::shared_ptr<Memory> _memory;

  public:
    ExecutableImpl() {}
    ExecutableImpl(const std::shared_ptr<Executable> &executable,
                   const std::shared_ptr<Memory> &memory)
        : _executable(executable), _memory(memory) {}

    virtual Index rows() const override {
      return _executable->outputBufferSize() / sizeof(ScalarType);
    }
    virtual Index cols() const override {
      return _executable->inputBufferSize() / sizeof(ScalarType);
    }

    /*
    virtual Index rows() const override { return _executable->inputs().size(); }
    virtual Index cols() const override {
      return _executable->outputs().size();
    }
    */
    virtual void mul(const Vector &input, Vector &output) const override {
      _executable->inputVector(input, _memory);
      _executable->execute(_memory);
      _executable->outputVector(_memory, output);
    }
    void setMemory(const std::shared_ptr<Memory> &memory) { _memory = memory; }
    void setExecutable(const std::shared_ptr<Executable> &executable) {
      _executable = executable;
    }
  };

private:
  std::shared_ptr<Impl> _impl;

public:
  enum {
    ColsAtCompileTime = Eigen::Dynamic,
    MaxColsAtCompileTime = Eigen::Dynamic,
    IsRowMajor = false
  };
  Index rows() const { return _impl->rows(); }
  Index cols() const { return _impl->cols(); }
  MatrixReplacement() {}
  MatrixReplacement(const std::shared_ptr<Executable> &executable,
                    const std::shared_ptr<Memory> &memory)
      : _impl(std::make_shared<ExecutableImpl>(executable, memory)) {}
  MatrixReplacement(const std::shared_ptr<Impl> &impl) : _impl(impl) {}
  template <class Rhs>
  Eigen::Product<MatrixReplacement, Rhs, Eigen::AliasFreeProduct>
  operator*(const Eigen::MatrixBase<Rhs> &x) const {
    return Eigen::Product<MatrixReplacement, Rhs, Eigen::AliasFreeProduct>(
        *this, x.derived());
  }
  void mul(const Vector &input, Vector &output) const {
    _impl->mul(input, output);
  }
  const Vector &diagonal() const { return _impl->diagonal(); }
};
} // namespace tractor

namespace Eigen {
namespace internal {
template <class ScalarType, typename Rhs>
struct generic_product_impl<tractor::MatrixReplacement<ScalarType>, Rhs,
                            SparseShape, DenseShape, GemvProduct>
    : generic_product_impl_base<
          tractor::MatrixReplacement<ScalarType>, Rhs,
          generic_product_impl<tractor::MatrixReplacement<ScalarType>, Rhs>> {
  typedef ScalarType Scalar;
  static void scaleAndAddTo(Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> &dst,
                            const tractor::MatrixReplacement<ScalarType> &lhs,
                            const Rhs &rhs, const Scalar &alpha) {
    if (alpha != 1.0) {
      throw std::runtime_error("scaling not implemented");
    }
    lhs.mul(rhs, dst);
  }
};
} // namespace internal
} // namespace Eigen

namespace Eigen {
template <class T>
struct NumTraits<tractor::Var<T>> : GenericNumTraits<tractor::Var<T>> {
  typedef tractor::Var<T> Real;
  typedef tractor::Var<T> NonInteger;
  typedef tractor::Var<T> Nested;
  static inline Real epsilon() { return 0; }
  static inline Real dummy_precision() { return 0; }
  static inline int digits10() { return 0; }
  enum {
    IsComplex = 0,
    IsInteger = 0,
    IsSigned = 1,
    RequireInitialization = 1,
    ReadCost = 1,
    AddCost = 3,
    MulCost = 3
  };
};

} // namespace Eigen

namespace tractor {
template <class T> const Var<T> &conj(const Var<T> &x) { return x; }
template <class T> const Var<T> &real(const Var<T> &x) { return x; }
template <class T> Var<T> imag(const Var<T> &x) { return Var<T>(T(0)); }
template <class T> Var<T> abs2(const Var<T> &x) { return x * x; }
} // namespace tractor
