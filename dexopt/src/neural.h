// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/tractor.h>

#include "ops.h"
#include "tensor.h"

#include <fstream>
#include <random>

namespace tractor {

enum class Activation {
  Linear,
  TanH,
  ReLU,
};

template <class T>
Tensor<T> applyActivation(const Tensor<T> &input_tensor,
                          const Activation &activation) {
  Tensor<T> tensor = input_tensor;
  size_t tensor_size = tensor.size();
  switch (activation) {
  case Activation::Linear:
    break;
  case Activation::TanH:
    for (size_t i = 0; i < tensor_size; i++) {
      tensor[i] = tanh(tensor[i]);
    }
    break;
  case Activation::ReLU:
    for (size_t i = 0; i < tensor_size; i++) {
      tensor[i] = relu(tensor[i]);
    }
    break;
  default:
    throw std::runtime_error("unsupported activation");
    break;
  }
  return tensor;
}

template <class Scalar> class Layer;

template <class Scalar> class Synapse {
  AlignedStdVector<Synapse<Scalar>> _inputs;
  std::shared_ptr<Layer<Scalar>> _layer;

public:
  Synapse() {}
  template <class LayerType, class... InputTypes>
  Synapse(int constructor_tag, const LayerType &layer,
          const InputTypes &... inputs)
      : _layer(std::make_shared<LayerType>(layer)), _inputs({inputs...}) {}
  auto &layer() const { return _layer; }
  auto &inputs() const { return _inputs; }
};

struct LayerMode {
  bool training = true;
};

struct NeuralBase {
  virtual ~NeuralBase() {}
  virtual void
  serialize(const std::function<void(NeuralBase *, void *, size_t)> &fnc) {}
};

template <class Scalar> class Layer : public NeuralBase {
public:
  virtual Tensor<Scalar> evaluate(const std::vector<Tensor<Scalar>> &inputs,
                                  const LayerMode &mode) = 0;
};

template <class Scalar, class Impl> class LayerBase : public Layer<Scalar> {
public:
  template <class... Inputs>
  Synapse<Scalar> operator()(const Inputs &... inputs) {
    return Synapse<Scalar>(0, *(Impl *)this, inputs...);
  }
};

template <class Scalar> class InputLayer : public Layer<Scalar> {
  size_t _units = 0;

public:
  InputLayer(size_t units) : _units(units) {}
  virtual Tensor<Scalar> evaluate(const std::vector<Tensor<Scalar>> &inputs,
                                  const LayerMode &mode) override {
    throw std::runtime_error("tried to evaluate an input layer");
  }
};

template <class Scalar> class Input : public Synapse<Scalar> {
public:
  Input(size_t units) : Synapse<Scalar>(0, InputLayer<Scalar>(units)) {}
};

template <class T> void variable(Tensor<T> &tensor) {
  size_t s = tensor.size();
  // std::cout << "tensor var " << s << std::endl;
  for (size_t i = 0; i < s; i++) {
    tractor::variable(tensor[i]);
  }
}

template <class Scalar, class StDev>
void randomize(Tensor<Scalar> &tensor, const StDev &stdev) {
  size_t s = tensor.size();
  static thread_local std::mt19937 gen{std::mt19937::result_type(rand())};
  std::normal_distribution<double> dist(0.0, stdev);
  for (size_t i = 0; i < s; i++) {
    tensor[i] = dist(gen);
  }
}

template <class Scalar>
class DenseLayer : public LayerBase<Scalar, DenseLayer<Scalar>> {
  typedef Var<typename BatchScalar<typename Scalar::Value>::Type> WeightScalar;
  size_t _units = 0;
  Activation _activation;
  bool _initialized = false;
  Tensor<WeightScalar> _weights;
  Tensor<WeightScalar> _bias;
  double _bias_regularization = 0;
  double _weight_regularization = 0;
  double _activity_regularization = 0;
  double _stdev = 0;

public:
  DenseLayer(size_t units, Activation activation = Activation::Linear,
             double bias_regularization = 0, double weight_regularization = 0,
             double activity_regularization = 0, double stdev = 0.001)
      : _units(units), _activation(activation),
        _bias_regularization(bias_regularization),
        _weight_regularization(weight_regularization),
        _activity_regularization(activity_regularization), _stdev(stdev) {}
  virtual Tensor<Scalar> evaluate(const std::vector<Tensor<Scalar>> &inputs,
                                  const LayerMode &mode) override {
    auto &input = inputs.at(0);
    TRACTOR_CHECK_TENSOR_DIMENSIONS(input, 1);

    if (!_initialized) {
      _initialized = true;
      std::cout << "build dense layer " << input.size() << " x " << _units
                << std::endl;
      _weights.resize(input.size(), _units);
      _bias.resize(_units);

      randomize(_weights, _stdev);
      randomize(_bias, _stdev);

      variable(_weights);
      variable(_bias);

      if (_weight_regularization != 0) {
        for (size_t row = 0; row < input.size(); row++) {
          for (size_t col = 0; col < _units; col++) {
            goal(_weights(row, col) * _weight_regularization);
          }
        }
      }

      if (_bias_regularization != 0) {
        for (size_t i = 0; i < _units; i++) {
          goal(_bias(i) * _bias_regularization);
        }
      }
    }

    // Tensor<Scalar> activity = tensor_mul_vec_mat(input, _weights) + _bias;

    Tensor<Scalar> activity = tensor_mul_vec_mat(input, _weights);
    for (size_t i = 0; i < _units; i++) {
      Scalar bias;
      batch(_bias[i], bias);
      activity[i] += bias;
    }

    if (_activity_regularization != 0) {
      for (size_t i = 0; i < _units; i++) {
        goal(activity(i) * typename Scalar::Value(_activity_regularization));
      }
    }

    return applyActivation(activity, _activation);
  }
  auto &weights() const { return _weights; }
  auto &bias() const { return _bias; }
  virtual void serialize(
      const std::function<void(NeuralBase *, void *, size_t)> &fnc) override {
    std::cerr << "serialize dense " << __LINE__ << std::endl;
    fnc(this, _bias.data(), _bias.bytes());
    std::cerr << "serialize dense " << __LINE__ << std::endl;
    fnc(this, _weights.data(), _weights.bytes());
    std::cerr << "serialize dense " << __LINE__ << std::endl;
  }
};

template <class Scalar>
class ActivationLayer : public LayerBase<Scalar, ActivationLayer<Scalar>> {
  Activation _activation = Activation::Linear;

public:
  ActivationLayer(Activation activation) : _activation(activation) {}
  virtual Tensor<Scalar> evaluate(const std::vector<Tensor<Scalar>> &inputs,
                                  const LayerMode &mode) override {
    auto &input = inputs.at(0);
    return applyActivation(input, _activation);
  }
};

template <class Scalar>
class GaussianNoiseLayer
    : public LayerBase<Scalar, GaussianNoiseLayer<Scalar>> {
  double _standard_deviation = 0.0;

public:
  GaussianNoiseLayer(const double &standard_deviation)
      : _standard_deviation(standard_deviation) {}
  virtual Tensor<Scalar> evaluate(const std::vector<Tensor<Scalar>> &inputs,
                                  const LayerMode &mode) override {
    auto activations = inputs.at(0);
    if (mode.training) {
      for (size_t i = 0; i < activations.size(); i++) {
        activations[i] = add_random_normal(activations[i], _standard_deviation);
      }
    }
    return activations;
  }
};

template <class Scalar>
class DropoutLayer : public LayerBase<Scalar, DropoutLayer<Scalar>> {
  double _rate = 0.0;

public:
  DropoutLayer(const double &rate) : _rate(rate) {}
  virtual Tensor<Scalar> evaluate(const std::vector<Tensor<Scalar>> &inputs,
                                  const LayerMode &mode) override {
    auto activations = inputs.at(0);
    if (mode.training) {
      for (size_t i = 0; i < activations.size(); i++) {
        activations[i] = dropout(activations[i], _rate);
      }
    }
    return activations;
  }
};

template <class Scalar>
class ActivityRegularizationLayer
    : public LayerBase<Scalar, ActivityRegularizationLayer<Scalar>> {
  double _l2 = 0.0;

public:
  ActivityRegularizationLayer(const double &l2) : _l2(l2) {}
  virtual Tensor<Scalar> evaluate(const std::vector<Tensor<Scalar>> &inputs,
                                  const LayerMode &mode) override {
    auto activations = inputs.at(0);
    if (_l2 > 0) {
      Scalar l2 = typename Scalar::Value(_l2);
      for (size_t i = 0; i < activations.size(); i++) {
        goal(activations[i] * l2);
      }
    }
    return activations;
  }
};

template <class Scalar> class NeuralNetwork : public NeuralBase {
  std::unordered_set<std::shared_ptr<Layer<Scalar>>> _input_set;
  std::unordered_set<std::shared_ptr<Layer<Scalar>>> _layer_set;
  std::vector<std::shared_ptr<Layer<Scalar>>> _inputs;
  std::vector<std::shared_ptr<Layer<Scalar>>> _outputs;
  std::vector<std::shared_ptr<Layer<Scalar>>> _layers;
  struct Op {
    std::shared_ptr<Layer<Scalar>> layer;
    std::vector<std::shared_ptr<Layer<Scalar>>> inputs;
  };
  std::vector<Op> _ops;
  void _findLayers(const Synapse<Scalar> &synapse) {
    if (_layer_set.insert(synapse.layer()).second) {
      _layers.push_back(synapse.layer());
      if (_input_set.find(synapse.layer()) == _input_set.end()) {
        Op op;
        op.layer = synapse.layer();
        for (auto &input : synapse.inputs()) {
          op.inputs.push_back(input.layer());
        }
        _ops.push_back(op);
      }
    }
    for (auto &input : synapse.inputs()) {
      _findLayers(input);
    }
  }

public:
  NeuralNetwork() {}
  NeuralNetwork(const std::initializer_list<Synapse<Scalar>> &inputs,
                const std::initializer_list<Synapse<Scalar>> &outputs) {
    init(inputs, outputs);
  }
  void init(const std::initializer_list<Synapse<Scalar>> &inputs,
            const std::initializer_list<Synapse<Scalar>> &outputs) {
    clear();
    for (auto &input : inputs) {
      _input_set.insert(input.layer());
      _inputs.push_back(input.layer());
    }
    for (auto &output : outputs) {
      _outputs.push_back(output.layer());
      _findLayers(output);
    }
    std::reverse(_layers.begin(), _layers.end());
    std::reverse(_ops.begin(), _ops.end());
  }
  void clear() {
    _input_set.clear();
    _layer_set.clear();
    _inputs.clear();
    _outputs.clear();
    _layers.clear();
    _ops.clear();
  }
  auto &inputs() const { return _inputs; }
  auto &outputs() const { return _outputs; }
  auto &layers() const { return _layers; }
  std::vector<Tensor<Scalar>>
  predict(const std::vector<Tensor<Scalar>> &inputs,
          const LayerMode &mode = LayerMode()) const {
    if (inputs.size() != _inputs.size()) {
      throw std::runtime_error("wrong input count");
    }
    std::unordered_map<std::shared_ptr<Layer<Scalar>>, Tensor<Scalar>> tensors;
    for (size_t i = 0; i < _inputs.size(); i++) {
      tensors[_inputs.at(i)] = inputs.at(i);
    }
    for (auto &op : _ops) {
      std::vector<Tensor<Scalar>> input_tensors;
      for (auto &input : op.inputs) {
        if (tensors.find(input) == tensors.end()) {
          throw std::runtime_error("input not connected");
        }
        input_tensors.push_back(tensors[input]);
      }
      auto output_tensor = op.layer->evaluate(input_tensors, mode);
      tensors[op.layer] = output_tensor;
    }
    std::vector<Tensor<Scalar>> output_tensors;
    for (auto &output : _outputs) {
      if (tensors.find(output) == tensors.end()) {
        throw std::runtime_error("output not connected");
      }
      output_tensors.push_back(tensors[output]);
    }
    return output_tensors;
  }
  Tensor<Scalar> predict(const Tensor<Scalar> &input,
                         const LayerMode &mode = LayerMode()) const {
    return predict(std::vector<Tensor<Scalar>>({input}), mode).at(0);
  }
  void serialize(
      const std::function<void(NeuralBase *, void *, size_t)> &fnc) override {
    for (auto &layer : _layers) {
      if (layer) {
        layer->serialize(fnc);
      }
    }
  }
  void serializeWeights(std::ostream &stream) {
    std::cerr << "begin serializing weights" << std::endl;
    auto callback = [&](NeuralBase *layer, void *ptr, size_t s) {
      std::cerr << "serializing layer " << typeid(*layer).name() << " " << layer
                << std::endl;
      stream.write((const char *)ptr, s);
    };
    std::cerr << "serializing weights" << std::endl;
    serialize(callback);
  }
  void saveWeights(const std::string &filename) {
    std::cerr << "opening file " << filename << std::endl;
    std::ofstream s(filename);
    std::cerr << "serializing" << std::endl;
    serializeWeights(s);
  }
  void deserializeWeights(std::istream &stream) {
    serialize([&](NeuralBase *layer, void *ptr, size_t s) {
      stream.read((char *)ptr, s);
    });
  }
  void loadWeights(const std::string &filename) {
    std::ifstream s(filename);
    deserializeWeights(s);
  }
};

template <class Scalar>
class SequentialNeuralNetwork : public NeuralNetwork<Scalar> {
  Synapse<Scalar> _input, _output;

public:
  void add(const Synapse<Scalar> &synapse) {
    _input = _output = synapse;
    this->init({_input}, {_output});
  }
  template <class LayerType>
  auto add(LayerType layer) ->
      typename std::enable_if<std::is_base_of<Layer<Scalar>, LayerType>::value,
                              void>::type {
    _output = layer(_output);
    this->init({_input}, {_output});
  }
};

} // namespace tractor
