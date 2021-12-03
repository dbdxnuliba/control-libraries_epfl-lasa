#include "dynamical_systems_bindings.h"

#include <dynamical_systems/DynamicalSystemFactory.hpp>
#include <dynamical_systems/DefaultDynamicalSystem.hpp>
#include <dynamical_systems/PointAttractor.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/parameters/Parameter.hpp>

#include <parameter_container.h>

using namespace state_representation;

//template<class S>
//class PyDynamicalSystem : public IDynamicalSystem<S> {
//public:
//  using IDynamicalSystem<S>::IDynamicalSystem;
//
//  [[nodiscard]] bool is_compatible(const S& state) const override {
//    PYBIND11_OVERRIDE(bool, IDynamicalSystem<S>, is_compatible, state);
//  }
//
//  void set_base_frame(const S& base_frame) override {
//    PYBIND11_OVERRIDE(void, IDynamicalSystem<S>, set_base_frame, base_frame);
//  }
//
//protected:
//  [[nodiscard]] S compute_dynamics(const S& state) const override {
//    PYBIND11_OVERRIDE_PURE(S, IDynamicalSystem<S>, compute_dynamics, state);
//  }
//
//  void validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter) override {
//    PYBIND11_OVERRIDE_PURE(void, IDynamicalSystem<S>, validate_and_set_parameter, parameter);
//  }
//};

class PyDynamicalSystem : public IDynamicalSystem<CartesianState>, public std::enable_shared_from_this<PyDynamicalSystem> {
public:
  using IDynamicalSystem<CartesianState>::IDynamicalSystem;

  [[nodiscard]] bool is_compatible(const CartesianState& state) const override {
    PYBIND11_OVERRIDE(bool, IDynamicalSystem<CartesianState>, is_compatible, state);
  }

  void set_base_frame(const CartesianState& base_frame) override {
    PYBIND11_OVERRIDE(void, IDynamicalSystem<CartesianState>, set_base_frame, base_frame);
  }

protected:
  [[nodiscard]] CartesianState compute_dynamics(const CartesianState& state) const override {
    PYBIND11_OVERRIDE_PURE(CartesianState, IDynamicalSystem<CartesianState>, compute_dynamics, state);
  }

  void validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter) override {
    PYBIND11_OVERRIDE_PURE(void, IDynamicalSystem<CartesianState>, validate_and_set_parameter, parameter);
  }
};

void ds_type(py::module_& m) {
  py::enum_<DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM>(m, "DYNAMICAL_SYSTEM")
      .value("NONE", DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::NONE)
      .value("CIRCULAR", DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::CIRCULAR)
      .value("POINT_ATTRACTOR", DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::POINT_ATTRACTOR)
      .value("RING", DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::RING)
      .export_values();
}

void cartesian(py::module_& m) {
  py::class_<IDynamicalSystem<CartesianState>, PyDynamicalSystem> c(m, "CartesianDS");

  c.def(py::init([](DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM type) {
//    return new PyDynamicalSystem();
    return new PointAttractor<CartesianState>();
//    return DynamicalSystemFactory<CartesianState>::create_dynamical_system(type);
  }));

  c.def("is_compatible", &IDynamicalSystem<CartesianState>::is_compatible);
  c.def("evaluate", &IDynamicalSystem<CartesianState>::evaluate);
  c.def("get_base_frame", &IDynamicalSystem<CartesianState>::get_base_frame);
  c.def("set_base_frame", &IDynamicalSystem<CartesianState>::set_base_frame);
  c.def("get_parameter", [](IDynamicalSystem<CartesianState>& self, const std::string& name) {
    return parameter_interface_ptr_to_container(self.get_parameter(name));
  });
  c.def("get_parameter_value", [](IDynamicalSystem<CartesianState>& self, const std::string& name) {
    return parameter_interface_ptr_to_container(self.get_parameter(name)).get_value();
  });
  c.def("get_parameters", [](IDynamicalSystem<CartesianState>& self) {
    py::dict dict;
    for (const auto& param_it : self.get_parameters()) {
      dict[py::str(param_it.first)] = parameter_interface_ptr_to_container(param_it.second);
    }
    return dict;
  });
  c.def("get_parameter_list", [](IDynamicalSystem<CartesianState>& self) {
    py::list list;
    for (const auto& param_it : self.get_parameters()) {
      list.append(parameter_interface_ptr_to_container(param_it.second));
    }
    return list;
  });
  c.def("set_parameter", [](IDynamicalSystem<CartesianState>& self, const ParameterContainer& parameter) {
    self.set_parameter(container_to_parameter_interface_ptr(parameter));
  });
//  c.def("set_parameters", [](IDynamicalSystem<CartesianState>& self, const py::dict& parameters) {
//    for(const auto& param_it : parameters) {
//      self.set_parameter(container_to_parameter_interface_ptr(param_it.second));
//    }
//  });
}

void bind_cartesian(py::module_& m) {
  ds_type(m);
  cartesian(m);
}