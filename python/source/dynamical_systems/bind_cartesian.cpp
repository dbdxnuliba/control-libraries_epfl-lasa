#include "dynamical_systems_bindings.h"

#include <dynamical_systems/DynamicalSystemFactory.hpp>
#include <dynamical_systems/DefaultDynamicalSystem.hpp>
#include <dynamical_systems/PointAttractor.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>

using namespace state_representation;

template<class S>
class PyDynamicalSystem : public IDynamicalSystem<S> {
public:
  using IDynamicalSystem<S>::IDynamicalSystem;

  [[nodiscard]] bool is_compatible(const S& state) const override {
    PYBIND11_OVERRIDE(bool, IDynamicalSystem<S>, is_compatible, state);
  }

  void set_base_frame(const S& base_frame) override {
    PYBIND11_OVERRIDE(void, IDynamicalSystem<S>, set_base_frame, base_frame);
  }

protected:
  [[nodiscard]] S compute_dynamics(const S& state) const override {
    PYBIND11_OVERRIDE_PURE(S, IDynamicalSystem<S>, compute_dynamics, state);
  }

  void validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter) override {
    PYBIND11_OVERRIDE_PURE(void, IDynamicalSystem<S>, validate_and_set_parameter, parameter);
  }
};

void ds_type(py::module_& m) {
  py::enum_<DynamicalSystemFactory<state_representation::CartesianState>::DYNAMICAL_SYSTEM>(m, "DYNAMICAL_SYSTEM")
      .value("NONE", DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::NONE)
      .value("CIRCULAR", DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::CIRCULAR)
      .value("POINT_ATTRACTOR", DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::POINT_ATTRACTOR)
      .value("RING", DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::RING)
      .export_values();
}

void cartesian(py::module_& m) {
  py::class_<IDynamicalSystem<CartesianState>, PyDynamicalSystem<CartesianState>> c(m, "CartesianDS");

  c.def(py::init([](DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM type) {
    return DynamicalSystemFactory<CartesianState>::create_dynamical_system(type);
  }));

  c.def("is_compatible", &IDynamicalSystem<CartesianState>::is_compatible);
  c.def("evaluate", &IDynamicalSystem<CartesianState>::evaluate);
  c.def("get_base_frame", &IDynamicalSystem<CartesianState>::get_base_frame);
  c.def("set_base_frame", &IDynamicalSystem<CartesianState>::set_base_frame);
  c.def("get_parameter", &IDynamicalSystem<CartesianState>::get_parameter);
  c.def("get_parameters", &IDynamicalSystem<CartesianState>::get_parameters);
  c.def("get_parameter_list", &IDynamicalSystem<CartesianState>::get_parameter_list);
  c.def("set_parameter", &IDynamicalSystem<CartesianState>::set_parameter);
}

void bind_cartesian(py::module_& m) {
  ds_type(m);
  cartesian(m);
}