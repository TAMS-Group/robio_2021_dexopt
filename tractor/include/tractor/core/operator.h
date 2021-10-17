// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/core/batch.h>
#include <tractor/core/recorder.h>

#include <cmath>
#include <memory>
#include <sstream>
#include <tuple>

namespace tractor {

typedef void (*LoopFunction)(const void *base, const uintptr_t *offsets,
                             size_t iterations);

typedef void (*OpFunction)(const void *base, const uintptr_t *offsets);

struct OperatorFunctions {
  LoopFunction loop = nullptr;
  OpFunction indirect = nullptr;
  const void *direct = nullptr;
};

template <class Functor> class RawArgumentTuple {
  template <class Ret, class... Args>
  static std::tuple<Args...> *getArgumentTuple(Ret (*)(Args...)) {
    return nullptr;
  }

public:
  typedef typename std::decay<decltype(
      *getArgumentTuple(*(Functor *)nullptr))>::type Type;
};

template <class Functor> class ArgumentValueTuple {
  template <class Ret, class... Args>
  static std::tuple<typename std::decay<Args>::type...> *
  getArgumentTuple(Ret (*)(Args...)) {
    return nullptr;
  }

public:
  typedef typename std::decay<decltype(
      *getArgumentTuple(*(Functor *)nullptr))>::type Type;
};

template <class Functor> class ReturnType {
  template <class Ret, class... Args>
  static Ret *getReturnType(Ret (*)(Args...)) {
    return nullptr;
  }

public:
  typedef typename std::decay<typename std::remove_pointer<decltype(
      getReturnType(*(Functor *)nullptr))>::type>::type Type;
};

template <class T> struct IsVar { static constexpr bool value = false; };
template <class T> struct IsVar<Var<T>> { static constexpr bool value = true; };

template <class... TT> struct AnyVar {};
template <class T, class... TT> struct AnyVar<T, TT...> {
  static constexpr bool value =
      (IsVar<typename std::decay<T>::type>::value || AnyVar<TT...>::value);
};
template <> struct AnyVar<> { static constexpr bool value = false; };

class OperatorModeMap {
  std::vector<const Operator *> _ops;

public:
  static size_t index(const std::type_index &type);
  inline auto at(size_t i) const { return i < _ops.size() ? _ops[i] : nullptr; }
  void put(size_t i, const Operator *op) {
    _ops.resize(std::max(_ops.size(), i + 1), nullptr);
    _ops[i] = op;
  }
};

class Operator {
public:
  class Argument {
    size_t _size = 0;
    bool _is_const = false;
    TypeInfo _type;

  public:
    template <class T> static Argument make() {
      Argument ret;
      ret._size = sizeof(typename std::decay<T>::type);
      ret._is_const =
          std::is_convertible<const typename std::decay<T>::type &, T>::value;
      ret._type = TypeInfo::get<T>();
      return ret;
    }
    size_t size() const { return _type.size(); }
    bool isInput() const { return _is_const; }
    bool isOutput() const { return !_is_const; }
    const std::type_index &type() const { return _type.type(); }
    const TypeInfo &typeInfo() const { return _type; }
  };

private:
  std::string _name;
  std::type_index _mode;
  std::type_index _op;
  const OperatorModeMap *_map = nullptr;

protected:
  OperatorFunctions _functions;
  size_t _argument_count = 0;
  std::vector<Argument> _arguments;
  static const Operator *tryFind(const std::type_index &mode,
                                 const std::type_index &group);
  Operator(const std::string &name, const std::type_info &mode,
           const std::type_info &op, const std::type_info &group);

public:
  Operator(const Operator &) = delete;
  ~Operator();
  Operator &operator=(const Operator &) = delete;
  template <class T> inline bool isMode() const { return _mode == typeid(T *); }
  inline const std::string &name() const { return _name; }
  // inline LoopFunction loopFunction() const { return _loop_function; }
  // inline OpFunction indirectFunction() const { return _indirect_function; }
  inline OperatorFunctions functions() const { return _functions; }
  inline size_t argumentCount() const { return _argument_count; }
  inline size_t argumentSize(size_t i) const { return _arguments[i].size(); }
  inline auto arguments() const { return ArrayRef<const Argument>(_arguments); }
  inline const Argument &arg(size_t i) const { return _arguments[i]; }
  template <class T> inline const Operator *tryFindVariant() const {
    size_t index = OperatorModeMap::index(typeid(T *));
    auto *op = _map->at(index);
    return op;
  }
  template <class T> inline const Operator *variant() const {
    auto *op = tryFindVariant<T>();
    if (!op) {
      throw std::runtime_error(std::string() + "variant not found: " +
                               typeid(T *).name() + " " + name());
    }
    return op;
  }
  static const Operator *tryFind(const std::string &name);
  static const Operator *find(const std::string &name) {
    auto *op = tryFind(name);
    if (!op) {
      throw std::runtime_error(std::string() + "operator not found: " + name);
    }
    return op;
  }
  static const Operator *
  tryFind(const std::type_index &mode, const std::type_index &op,
          const std::initializer_list<std::type_index> &args);
  template <class Mode, class Op, class... Args>
  static const Operator *tryFind(const Args &... args) {
    return tryFind(typeid(Mode *), typeid(Op *), {args...});
  }
  template <class Mode, class Op, class... Args>
  static const Operator *find(const Args &... args) {
    auto *ret = tryFind<Mode, Op>(args...);
    if (!ret) {
      std::stringstream s;
      s << "operator not found: " << typeid(Mode *).name() << " "
        << typeid(Op *).name();
      for (auto &arg : {args...}) {
        s << " " << arg.name();
      }
      throw std::runtime_error(s.str());
    }
    return ret;
  }
  template <class Op> bool is() const { return _op == typeid(Op *); }
  static std::vector<const Operator *> all();
};

#ifdef TRACTOR_IMPLEMENT_OPS

template <class Impl, class Mode, class Op, class Group>
class OperatorImpl : public Operator {
  template <class... Args> struct Init {
    template <class Ret, size_t... Indices> struct Looper {
      static void loop(const void *base, const uintptr_t *offsets,
                       size_t iterations) {
        for (size_t i = 0; i < iterations; i++) {
          *(Ret *)(void *)((uint8_t *)base + offsets[sizeof...(Indices)]) =
              Impl::call(
                  *(typename std::decay<Args>::type
                        *)(void *)((uint8_t *)base + offsets[Indices])...);
          offsets += sizeof...(Indices) + 1;
        }
      }
      static void indirect(const void *base, const uintptr_t *offsets) {
        *(Ret *)(void *)((uint8_t *)base + offsets[sizeof...(Indices)]) =
            Impl::call(*(typename std::decay<Args>::type
                             *)(void *)((uint8_t *)base + offsets[Indices])...);
      }
      static void direct(typename std::decay<Args>::type *... args, Ret *ret) {
        *ret = Impl::call(*args...);
      }
      static std::vector<Argument> arguments() {
        // auto x = {(std::cout << typeid(Args &).name() << std::endl, 0)...};
        return {Argument::make<Args>()..., Argument::make<Ret &>()};
      }
    };
    template <size_t... Indices> struct Looper<void, Indices...> {
      static void loop(const void *base, const uintptr_t *offsets,
                       size_t iterations) {
        for (size_t i = 0; i < iterations; i++) {
          Impl::call(*(
              typename std::decay<Args>::type *)(void *)((uint8_t *)base +
                                                         offsets[Indices])...);
          offsets += sizeof...(Indices);
        }
      }
      static void indirect(const void *base, const uintptr_t *offsets) {
        Impl::call(
            *(typename std::decay<Args>::type *)(void *)((uint8_t *)base +
                                                         offsets[Indices])...);
      }
      static void direct(typename std::decay<Args>::type *... args) {
        Impl::call(*args...);
      }
      static std::vector<Argument> arguments() {
        return {Argument::make<Args>()...};
      }
    };
  };
  typedef typename ReturnType<decltype(&Impl::call)>::Type Return;
  template <size_t... Indices, class... Args>
  void init(const std::integer_sequence<size_t, Indices...> &indices,
            std::tuple<Args...> *) {
    typedef Init<Args...> _Init;
    typedef typename _Init::template Looper<Return, Indices...> _Loop;
    _functions.loop = &_Loop::loop;
    _functions.indirect = &_Loop::indirect;
    _functions.direct = reinterpret_cast<const void *>(&_Loop::direct);
    _arguments = _Loop::arguments();
  }
  typedef typename RawArgumentTuple<decltype(&Impl::call)>::Type ArgumentTuple;

public:
  OperatorImpl(const std::string &name)
      : Operator(name, typeid(Mode *), typeid(Op *), typeid(Group *)) {
    constexpr size_t argument_count = std::tuple_size<ArgumentTuple>::value;
    _argument_count =
        argument_count + (std::is_same<Return, void>::value ? 0 : 1);
    init(std::make_index_sequence<argument_count>(), (ArgumentTuple *)nullptr);
  }
  static const Operator *instance(const char *name) {
    // std::cout << name << std::endl;
    static const Operator *instance = [name]() {
      auto *instance = tryFind(typeid(Mode *), typeid(Group *));
      if (!instance) {
        instance = new OperatorImpl(name);
        // std::cout << "new op " << name << std::endl;
      } else {
        // std::cout << "op already exists " << name << std::endl;
      }
      return instance;
    }();
    return instance;
  }
};

#endif

template <class T> class Var;

template <class T> struct OverloadSelector {
  // template <class U> operator const U &() { return *(const U *)nullptr; }
  template <class U,
            std::enable_if_t<std::is_convertible<T, U>::value, int> Z = 0>
  operator const U &() {
    return *(const U *)nullptr;
  }
};
template <class T> struct OverloadSelector<const Var<T> &> {
  operator const T &() { return *(const T *)nullptr; }
};
template <class T> struct OverloadSelector<Var<T> &> {
  operator T &() { return *(T *)nullptr; }
};
template <class T> struct OverloadSelector<Var<T>> {
  operator const T &() { return *(const T *)nullptr; }
};

template <class T> struct ArgumentConverter {
  static inline const T &map(const Var<T> &v) { return v.value(); }
  static inline T &map(Var<T> &v) { return v.value(); }
};

template <class Ret, class Op> struct Caller {
  template <class... Args> static inline Var<Ret> call2(Args &... args) {
    Var<Ret> ret;
    ret.value() = Op::call(args...);
    recordOperation(Op::instance(), &args..., &ret.value());
    return std::move(ret);
  }
  template <class... ImplArgs, class... Args>
  static inline Var<Ret> call(std::tuple<ImplArgs...> *, Args &... args) {
    return std::move(call2(ArgumentConverter<ImplArgs>::map(args)...));
  }
};
template <class Op> struct Caller<void, Op> {
  template <class... Args> static inline void call2(Args &... args) {
    Op::call(args...);
    recordOperation(Op::instance(), &args...);
  }
  template <class... ImplArgs, class... Args>
  static inline void call(std::tuple<ImplArgs...> *, Args &... args) {
    call2(ArgumentConverter<ImplArgs>::map(args)...);
  }
};

#define TRACTOR_STRINGIFY(x) #x

#ifdef TRACTOR_IMPLEMENT_OPS

#define TRACTOR_OP_TYPED(mode, prefix, name, args, impl, scalar, postfix)      \
                                                                               \
  class op_##name;                                                             \
  struct op_##prefix##name##_##postfix##_impl_1 {                              \
    typedef scalar T;                                                          \
    typedef BatchScalar<scalar>::Type S;                                       \
    static inline auto call args impl;                                         \
  };                                                                           \
                                                                               \
  struct scalar##postfix##_group;                                              \
                                                                               \
  const Operator *op_##prefix##name##_##postfix##_inst =                       \
      OperatorImpl<op_##prefix##name##_##postfix##_impl_1, mode, op_##name,    \
                   std::tuple<op_##name *, scalar##postfix##_group *>>::       \
          instance(TRACTOR_STRINGIFY(prefix##name##_##postfix));               \
                                                                               \
  struct op_##prefix##name##_##postfix##_impl_2                                \
      : op_##prefix##name##_##postfix##_impl_1 {                               \
    static decltype(op_##prefix##name##_##postfix##_inst) instance();          \
  };                                                                           \
  decltype(op_##prefix##name##_##postfix##_inst)                               \
      op_##prefix##name##_##postfix##_impl_2::instance() {                     \
    return op_##prefix##name##_##postfix##_inst;                               \
  }                                                                            \
                                                                               \
  namespace op_##prefix##name##_##postfix##_ns {                               \
    typedef scalar T;                                                          \
    typedef BatchScalar<scalar>::Type S;                                       \
    static op_##prefix##name##_##postfix##_impl_2                              \
        *op_##prefix##name##_overload args;                                    \
  }                                                                            \
  using op_##prefix##name##_##postfix##_ns::op_##prefix##name##_overload;

#else

#define TRACTOR_OP_TYPED(mode, prefix, name, args, impl, scalar, postfix)      \
                                                                               \
  class op_##name;                                                             \
  struct op_##prefix##name##_##postfix##_impl_1 {                              \
    typedef scalar T;                                                          \
    typedef BatchScalar<scalar>::Type S;                                       \
    static inline auto call args impl;                                         \
  };                                                                           \
                                                                               \
  struct scalar##postfix##_group;                                              \
                                                                               \
  extern const Operator *op_##prefix##name##_##postfix##_inst;                 \
                                                                               \
  struct op_##prefix##name##_##postfix##_impl_2                                \
      : op_##prefix##name##_##postfix##_impl_1 {                               \
    static decltype(op_##prefix##name##_##postfix##_inst) instance();          \
  };                                                                           \
                                                                               \
  namespace op_##prefix##name##_##postfix##_ns {                               \
    typedef scalar T;                                                          \
    typedef BatchScalar<scalar>::Type S;                                       \
    static op_##prefix##name##_##postfix##_impl_2                              \
        *op_##prefix##name##_overload args;                                    \
  }                                                                            \
  using op_##prefix##name##_##postfix##_ns::op_##prefix##name##_overload;

#endif

// #define TRACTOR_OP_IMPL(mode, prefix, name, args, impl, postfix)               \
//   TRACTOR_OP_TYPED(mode, prefix, name, args, impl, float, postfix##f)          \
//   TRACTOR_OP_TYPED(mode, prefix, name, args, impl, double, postfix##d)         \
//   TRACTOR_OP_TYPED(mode, prefix, name, args, impl, uint64_t, postfix##i)       \
//   TRACTOR_OP_TYPED(mode, prefix, name, args, impl, Batch4f, postfix##4f)       \
//   TRACTOR_OP_TYPED(mode, prefix, name, args, impl, Batch4d, postfix##4d)       \
//   TRACTOR_OP_TYPED(mode, prefix, name, args, impl, Batch8f, postfix##8f)       \
//   TRACTOR_OP_TYPED(mode, prefix, name, args, impl, Batch8d, postfix##8d)

#define TRACTOR_OP_IMPL(mode, prefix, name, args, impl, postfix)               \
  TRACTOR_OP_TYPED(mode, prefix, name, args, impl, float, postfix##f)          \
  TRACTOR_OP_TYPED(mode, prefix, name, args, impl, double, postfix##d)         \
  TRACTOR_OP_TYPED(mode, prefix, name, args, impl, uint64_t, postfix##i)       \
  TRACTOR_OP_TYPED(mode, prefix, name, args, impl, Batch4f, postfix##4f)       \
  TRACTOR_OP_TYPED(mode, prefix, name, args, impl, Batch4d, postfix##4d)       \
  TRACTOR_OP_TYPED(mode, prefix, name, args, impl, Batch8d, postfix##8d)       \
  TRACTOR_OP_TYPED(mode, prefix, name, args, impl, Batch16d, postfix##16d)

#define TRACTOR_VAR_OP(name)                                                   \
  template <class... Args,                                                     \
            std::enable_if_t<AnyVar<Args...>::value, int> X = 0,               \
            class Impl = typename std::decay<decltype(                         \
                *op_##name##_overload(OverloadSelector<Args>()...))>::type,    \
            class ImplArgs =                                                   \
                typename ArgumentValueTuple<decltype(&Impl::call)>::Type,      \
            class Ret = decltype(Impl::call(OverloadSelector<Args>()...)),     \
            decltype(Impl::call(OverloadSelector<Args>()...)) *Y = nullptr>    \
  inline auto name(Args &&... args) {                                          \
    return Caller<Ret, Impl>::call((ImplArgs *)nullptr, args...);              \
  }

#define TRACTOR_OP(name, args, impl)                                           \
  TRACTOR_OP_IMPL(compute, , name, args, impl, )                               \
  TRACTOR_VAR_OP(name)

#define TRACTOR_D(mode, name, args, impl)                                      \
  TRACTOR_OP_IMPL(mode, mode##_, name, args, impl, )

#define TRACTOR_OP_T(postfix, name, args, impl)                                \
  TRACTOR_OP_IMPL(compute, , name, args, impl, postfix##_)                     \
  // TRACTOR_VAR_OP(name)

#define TRACTOR_D_T(mode, postfix, name, args, impl)                           \
  TRACTOR_OP_IMPL(mode, mode##_, name, args, impl, postfix##_)

/*
  #define TRACTOR_D_LOOP(mode, name, args, impl) \
  TRACTOR_D(mode, name, args, { typedef S T; makeBatchLoop<F>(
      [&] args impl).run(
      a, lo, hi, da, padding, dx);
   })
*/

#define TRACTOR_D_LOOP(mode, name, args, args2, impl)                          \
  TRACTOR_D(mode, name, args, {                                                \
    typedef S T;                                                               \
    auto f = [] args {                                                         \
      typedef S T;                                                             \
      impl                                                                     \
    };                                                                         \
    makeBatchLoop(f).run args2;                                                \
  })

template <class T, class Impl = typename std::decay<decltype(
                       *op_move_overload(OverloadSelector<Var<T>>()))>::type>
inline void Recorder_move_impl(Recorder *rec, const T *from, T *to) {
  rec->op(Impl::instance(), from, to);
}

template <class T> void Recorder::move(const T *from, T *to) {
  Recorder_move_impl(this, from, to);
}

} // namespace tractor
