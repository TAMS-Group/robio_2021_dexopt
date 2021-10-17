// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <typeinfo>
#include <vector>

namespace tractor {

class Buffer {
  std::vector<uint8_t> _data;

public:
  template <class Begin, class End>
  void assign(const Begin &begin, const End &end) {
    _data.assign(begin, end);
  }

  void append(const Buffer &b) {
    size_t p = _data.size();
    _data.resize(p + b.size());
    std::memcpy(_data.data() + p, b.data(), b.size());
  }

  void clear() { _data.clear(); }

  inline const void *data() const { return _data.data(); }
  inline void *data() { return _data.data(); }

  inline size_t size() const { return _data.size(); }
  void resize(size_t s) { _data.resize(s); }

  void zero(size_t s) {
    _data.clear();
    _data.resize(s, 0);
  }
  void zero() { std::memset(_data.data(), 0, _data.size()); }

  template <class T, class PType> inline T &at(const PType &port) {
    if (port.type() != typeid(typename std::decay<T>::type)) {
      throw std::runtime_error("type mismatch");
    }
    if (port.offset() + port.size() < _data.size()) {
      _data.resize(port.offset() + port.size());
    }
    return *(T *)(void *)(_data.data() + port.offset());
  }

  template <class T, class PType> inline const T &at(const PType &port) const {
    if (port.type() != typeid(typename std::decay<T>::type)) {
      throw std::runtime_error("type mismatch");
    }
    return *(const T *)(const void *)(_data.data() + port.offset());
  }

  template <class PContainer> void gather(const PContainer &container) {
    size_t size = 0;
    for (const auto &port : container) {
      size = std::max(size, port.offset() + port.size());
    }
    _data.resize(std::max(_data.size(), size));
    for (const auto &port : container) {
      if (port.binding()) {
        std::memcpy(_data.data() + port.offset(), (const void *)port.binding(),
                    port.size());
      }
    }
  }

  template <class PContainer> void scatter(const PContainer &container) const {
    for (const auto &port : container) {
      if (port.binding()) {
        // std::cout << "scatter " << _data.size() << " " << port.offset() << "
        // "
        //          << (void *)port.binding() << std::endl;
        std::memcpy((void *)port.binding(), _data.data() + port.offset(),
                    port.size());
      }
    }
  }

  /*
  template <class PContainer, class Vector>
  void toVector(const PContainer &ports, Vector &vector) const {
    vector.resize(ports.size());
    size_t i = 0;
    for (auto &port : ports) {
      auto &v = vector[i];
      v = at<typename std::decay<decltype(v)>::type>(port);
      i++;
    }
  }

  template <class PContainer, class Vector>
  void fromVector(const PContainer &ports, Vector &vector) {
    size_t size = 0;
    for (const auto &port : ports) {
      size = std::max(size, port.offset() + port.size());
    }
    _data.resize(std::max(_data.size(), size));
    size_t i = 0;
    for (auto &port : ports) {
      auto &v = vector[i];
      at<typename std::decay<decltype(v)>::type>(port) = v;
      i++;
    }
  }
  */

  template <class PContainer, class Vector>
  void toVector(const PContainer &ports, Vector &&vector) const {
    typedef typename std::decay<decltype(vector[0])>::type Scalar;
    {
      size_t element_count = 0;
      for (auto &port : ports) {
        if (port.size() % sizeof(Scalar) != 0) {
          throw std::runtime_error("element types incompatible");
        }
        element_count += port.size() / sizeof(Scalar);
      }
      vector.resize(element_count);
    }
    {
      size_t i_vector = 0;
      for (auto &port : ports) {
        if (port.size() % sizeof(Scalar) != 0) {
          throw std::runtime_error("element types incompatible");
        }
        const Scalar *port_data =
            (const Scalar *)(void *)(_data.data() + port.offset());
        for (size_t i_element = 0; i_element < port.size() / sizeof(Scalar);
             i_element++) {
          vector[i_vector] = port_data[i_element];
          i_vector++;
        }
      }
    }
  }

  template <class PContainer, class Vector>
  void fromVector(const PContainer &ports, Vector &&vector) {
    typedef typename std::decay<decltype(vector[0])>::type Scalar;
    {
      size_t size = 0;
      for (const auto &port : ports) {
        size = std::max(size, port.offset() + port.size());
      }
      _data.resize(std::max(_data.size(), size));
    }
    {
      size_t size = 0;
      for (const auto &port : ports) {
        if (port.size() % sizeof(Scalar) != 0) {
          throw std::runtime_error("element types incompatible");
        }
        size += port.size() / sizeof(Scalar);
      }
      if (vector.size() != size) {
        std::cerr << "incompatible vector size " << vector.size() << " " << size
                  << std::endl;
        throw std::runtime_error("incompatible vector size");
      }
    }
    {
      size_t i_vector = 0;
      for (auto &port : ports) {
        if (port.size() % sizeof(Scalar) != 0) {
          throw std::runtime_error("element types incompatible");
        }
        Scalar *port_data = (Scalar *)(void *)(_data.data() + port.offset());
        for (size_t i_element = 0; i_element < port.size() / sizeof(Scalar);
             i_element++) {
          port_data[i_element] = vector[i_vector];
          i_vector++;
        }
      }
    }
  }

  template <class Vector> void fromVectorDense(Vector &vector) {
    typedef typename std::decay<decltype(vector[0])>::type Scalar;
    _data.resize(vector.size() * sizeof(vector[0]));
    // std::memcpy(_data.data(), vector.data(), _data.size());
    Scalar *data = (Scalar *)_data.data();
    for (size_t i = 0; i < vector.size(); i++) {
      data[i] = vector[i];
    }
  }
};

} // namespace tractor
