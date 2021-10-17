// (c) 2020-2021 Philipp Ruppel

#include <tractor/engines/jit.h>

#include <tractor/core/profiler.h>

#include <sys/mman.h>

namespace tractor {

/*

fnc = (OpFunction)(void*)(uintptr_t)0x124124123300;
        fnc(memory, offsets + 0x15650000/8);

typedef void (*OpFunction)(const void *base, const uintptr_t *offsets);

void bla(const void* memory, const uintptr_t *offsets) {
asm(
"mov %rsi,%rcx\n"
"lea 0x11223344(%rcx),%rsi\n"
"mov $0x11223344556677,%rax\n"
"call *%rax\n"
);

asm(
"mov %rsi,%rcx\n"
"nop\n"
"lea 0x11223344(%rcx),%rsi\n"
"call *0x11223344(%rip)\n"
"nop\n"
"ret\n"
"nop\n"
"nop\n"
"nop\n"
"nop\n"
);
}


void bla(const void* memory) {
asm(
"mov $0x22334455667788,%rsi\n"
"mov $0x11223344556677,%rax\n"
"call *%rax\n"
"nop\n"
"ret\n"
"nop\n"
"nop\n"
"nop\n"
"nop\n"
);

asm(
"mov %rdi,%r12\n"
"mov $0x22334455667788,%rsi\n"
"mov $0x11223344556677,%rax\n"
"mov %r12,%rdi\n"
"call *%rax\n"
"nop\n"
"ret\n"
"nop\n"
"nop\n"
"nop\n"
"nop\n"
);

asm(
"mov %rdi,%r12\n"
"mov $0x22334455667788,%rsi\n"
"mov %r12,%rdi\n"
"call *0x11223344(%rip)\n"
"nop\n"
"ret\n"
"nop\n"
"nop\n"
"nop\n"
"nop\n"
);


*/

class JITCode {
  std::vector<uint8_t> _code;

public:
  template <class T> JITCode &operator<<(const T &v) {
    size_t offset = _code.size();
    _code.resize(offset + sizeof(T));
    std::memcpy(_code.data() + offset, &v, sizeof(T));
    return *this;
  }
  template <class S> JITCode &write(const S &a) {
    (*this) << a;
    return *this;
  }
  template <class S, class... T> JITCode &write(const S &a, const T &... v) {
    write(a);
    write(v...);
    return *this;
  }
  auto *data() const { return _code.data(); }
  auto size() const { return _code.size(); }
};

void JITEngine::JITProgramBase::_free() {
  if (_data) {
    std::cout << "free jit memory " << _data << std::endl;
    munmap(_data, _size);
    _data = nullptr;
    _size = 0;
  }
}

JITEngine::JITProgramBase::JITProgramBase() {}

JITEngine::JITProgramBase::~JITProgramBase() { _free(); }

void JITEngine::JITProgramBase::alloc(size_t size) {
  _free();

  // const void *hint = (const void *)&JITEngine::JITProgramBase::alloc;

  _data = mmap(nullptr, size, PROT_READ | PROT_WRITE,
               MAP_ANONYMOUS | MAP_PRIVATE, 0, 0);

  /*
  {
    // auto hint = (intptr_t)(void *)&JITEngine::JITProgramBase::alloc;
    // hint = ((hint >> 16) << 16);
    intptr_t hint = 64 * 1024;
    while (!_data) {
      std::cout << "hint" << (void *)hint << std::endl;
      _data = mmap((void *)hint, size, PROT_READ | PROT_WRITE | PROT_EXEC,
                   MAP_ANONYMOUS | MAP_PRIVATE | MAP_FIXED, 0, 0);
      if ((int64_t)_data <= 0) {
        _data = nullptr;
      }
      hint += 64 * 1024;
      std::cout << "jit memory allocated " << _data << std::endl;
    }
  }
  */

  /*
  intptr_t hint = 64 * 1024;
  std::cout << "jit memory hint" << (void *)hint << std::endl;
  _data = mmap((void *)hint, size, PROT_READ | PROT_WRITE | PROT_EXEC,
               MAP_ANONYMOUS | MAP_PRIVATE, 0, 0);
  std::cout << "jit memory allocated " << _data << std::endl;
  */

  _size = size;

  if ((int64_t)_data <= 0) {
    throw std::runtime_error("failed to allocate jit memory");
  }

  if (mprotect(_data, _size, PROT_READ | PROT_WRITE) != 0) {
    throw std::runtime_error("failed to make jit memory writable");
  }
}

void JITEngine::JITProgramBase::write(const void *data, size_t size) {
  if (size > _size) {
    throw std::runtime_error("allocated jit buffer is too small");
  }
  if (mprotect(_data, _size, PROT_READ | PROT_WRITE) != 0) {
    throw std::runtime_error("failed to make jit memory writable");
  }
  std::memcpy(_data, data, size);
  if (mprotect(_data, _size, PROT_EXEC) != 0) {
    throw std::runtime_error("failed to make jit memory executable");
  }
}

JITEngine::ExecutableImpl::ExecutableImpl() {}

JITEngine::ExecutableImpl::~ExecutableImpl() {}

void JITEngine::ExecutableImpl::_compile(const Program &program) {

  // std::cout << program << std::endl;

  uintptr_t function_base = -1;
  size_t program_size = 0;
  _arguments.clear();
  for (auto &instp : program.instructions()) {
    program_size++;
    for (auto &arg : instp.args()) {
      _arguments.push_back(arg);
      program_size++;
    }
    function_base =
        std::min(function_base, (uintptr_t)instp.op()->functions().indirect);
  }

  {
    JITCode code;

    _jit_function.alloc(program_size * 16 + 256);

    const auto *argptr = _arguments.data();
    uint64_t argcntr = 0;

    code.write(uint8_t(0x53));                // push %rbx
    code.write(uint8_t(0x41), uint8_t(0x54)); // push %r12
    code.write(uint8_t(0x41), uint8_t(0x55)); // push %r13
    code.write(uint8_t(0x41), uint8_t(0x56)); // push %r14
    code.write(uint8_t(0x41), uint8_t(0x57)); // push %r15

    code.write(uint8_t(0x49), uint8_t(0x89), uint8_t(0xfc)); // mov %rdi,%r12

    code.write(uint8_t(0x49), uint8_t(0xbe),
               uint64_t(function_base)); // mov function_base,%r14

    code.write(uint8_t(0x49), uint8_t(0xbf),
               uint64_t(argptr)); // mov argptr,%r15

    for (auto &inst : program.instructions()) {

      // if (inst.argumentCount() <= 6)
      if (true)
      // if (false)
      //
      {

        uint64_t opfun = uint64_t(uintptr_t(inst.op()->functions().direct));
        code.write(uint8_t(0x48), uint8_t(0xb8), opfun); // mov function,%rax

        for (ssize_t iarg = inst.argumentCount() - 1; iarg > 5; iarg--) {

          // lea 0x11223344(%r12),%rdi
          code.write(uint8_t(0x49), uint8_t(0x8d), uint8_t(0xbc), uint8_t(0x24),
                     uint32_t(inst.arg(iarg)));

          // push %rdi
          code.write(uint8_t(0x57));
        }

        if (inst.argumentCount() > 0) {
          code.write(uint8_t(0x49), uint8_t(0x8d), uint8_t(0xbc), uint8_t(0x24),
                     uint32_t(inst.arg(0)));
        }

        if (inst.argumentCount() > 1) {
          code.write(uint8_t(0x49), uint8_t(0x8d), uint8_t(0xb4), uint8_t(0x24),
                     uint32_t(inst.arg(1)));
        }

        if (inst.argumentCount() > 2) {
          code.write(uint8_t(0x49), uint8_t(0x8d), uint8_t(0x94), uint8_t(0x24),
                     uint32_t(inst.arg(2)));
        }

        if (inst.argumentCount() > 3) {
          code.write(uint8_t(0x49), uint8_t(0x8d), uint8_t(0x8c), uint8_t(0x24),
                     uint32_t(inst.arg(3)));
        }

        if (inst.argumentCount() > 4) {
          code.write(uint8_t(0x4d), uint8_t(0x8d), uint8_t(0x84), uint8_t(0x24),
                     uint32_t(inst.arg(4)));
        }

        if (inst.argumentCount() > 5) {
          code.write(uint8_t(0x4d), uint8_t(0x8d), uint8_t(0x8c), uint8_t(0x24),
                     uint32_t(inst.arg(5)));
        }

        // call *%rax
        code.write(uint8_t(0xff), uint8_t(0xd0));

        for (ssize_t iarg = inst.argumentCount() - 1; iarg > 5; iarg--) {

          // pop
          code.write(uint8_t(0x5f));
        }

      } else {

        uint64_t opfun = uint64_t(uintptr_t(inst.op()->functions().indirect));
        code.write(uint8_t(0x48), uint8_t(0xb8), opfun); // mov function,%rax

        // code.write(uint8_t(0x48), uint8_t(0xbe),
        //             uint64_t(argptr)); // mov argptr,%rsi -> second arg

        code.write(uint8_t(0x49), uint8_t(0x8d), uint8_t(0xb7),
                   uint32_t(argcntr * 8)); // lea 0x11223344(%r15),%rsi

        // code.write(uint8_t(0x4c), uint8_t(0x89), uint8_t(0xfe)); // mov
        // %r15,%rsi

        code.write(uint8_t(0x4c), uint8_t(0x89),
                   uint8_t(0xe7)); // mov %r12,%rdi

        code.write(uint8_t(0xff), uint8_t(0xd0)); // call *%rax

        // code.write(uint8_t(0x4c), uint8_t(0x03), uint8_t(0x3c),
        // uint8_t(0x25),
        //             uint32_t(inst.argumentCount() * 8)); // add
        //             0x112233,%r15
      }

      argptr += inst.argumentCount();
      argcntr += inst.argumentCount();
    }

    code.write(uint8_t(0x41), uint8_t(0x5f)); // pop %r15
    code.write(uint8_t(0x41), uint8_t(0x5e)); // pop %r14
    code.write(uint8_t(0x41), uint8_t(0x5d)); // pop %r13
    code.write(uint8_t(0x41), uint8_t(0x5c)); // pop %r12
    code.write(uint8_t(0x5b));                // pop %rbx

    code.write(0xc3); // ret

    for (size_t i = 0; i < 32; i++) {
      code.write(0x90); // nop
    }

    _jit_function.write(code.data(), code.size());
  }
}

void JITEngine::ExecutableImpl::_execute(
    const std::shared_ptr<Memory> &memory) const {

  // std::cout << "this " << this << std::endl;
  // std::cout << "memory " << memory.get() << std::endl;
  auto &temp = *(MemoryImpl *)(memory.get());
  // std::cout << "temp " << &temp << std::endl;
  // std::cout << "_memory_size " << _memory_size << std::endl;
  // std::cout << "temp.size() " << temp.size() << std::endl;
  temp.resize(std::max(temp.size(), _memory_size));
  // std::cout << "x" << __LINE__ << std::endl;

  {
    TRACTOR_PROFILER("load constants");
    for (auto &port : _constants) {
      std::memcpy((uint8_t *)temp.data() + port.address(),
                  _const_data.data() + port.offset(), port.size());
    }
  }
  // std::cout << "x" << __LINE__ << std::endl;

  // std::cout << "arg base " << _arguments.data() << std::endl;
  // std::cout << "mem base " << temp.data() << std::endl;

  {
    TRACTOR_PROFILER("jitfnc");
    _jit_function.call(temp.data());
  }
}

} // namespace tractor
