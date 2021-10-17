// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

namespace tractor {

template <class T> class SimpleIterator {
  T *_ptr = nullptr;

public:
  SimpleIterator(T *ptr) : _ptr(ptr) {}
  inline SimpleIterator &operator++() {
    ++_ptr;
    return *this;
  }
  inline SimpleIterator operator++(int) {
    SimpleIterator ret = *this;
    ++_ptr;
    return std::move(ret);
  }
  inline T &operator*() const { return *_ptr; }
  inline T *operator->() const { return _ptr; }
  inline bool operator==(const SimpleIterator &other) const {
    return _ptr == other._ptr;
  }
  inline bool operator!=(const SimpleIterator &other) const {
    return _ptr != other._ptr;
  }
};

template <class T, class Iterator = SimpleIterator<T>> class ArrayRef {
  const Iterator _begin;
  const Iterator _end;

public:
  ArrayRef(const Iterator &begin, const Iterator &end)
      : _begin(Iterator(begin)), _end(Iterator(end)) {}
  template <class Container>
  ArrayRef(Container &container)
      : _begin(Iterator(container.data())),
        _end(Iterator(container.data() + container.size())) {}
  ArrayRef(ArrayRef &&other) noexcept
      : _begin(other.begin()), _end(other.end()) {}
  ArrayRef(const ArrayRef &) = delete;
  ArrayRef &operator=(const ArrayRef &);
  const Iterator &begin() const { return _begin; }
  const Iterator &end() const { return _end; }
  bool empty() const { return _begin == _end; }
};

template <class T> class ArrayRef<T, SimpleIterator<T>> {
  typedef SimpleIterator<T> Iterator;
  const Iterator _begin;
  const Iterator _end;

public:
  ArrayRef(const Iterator &begin, const Iterator &end)
      : _begin(Iterator(begin)), _end(Iterator(end)) {}
  template <class Container>
  ArrayRef(Container &container)
      : _begin(Iterator(container.data())),
        _end(Iterator(container.data() + container.size())) {}
  const Iterator &begin() const { return _begin; }
  const Iterator &end() const { return _end; }
  size_t size() const { return &*_end - &*_begin; }
  bool empty() const { return _begin == _end; }
};

} // namespace tractor
