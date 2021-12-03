#pragma once

//#include <iostream>
//#include <string>
//#include <sstream>
//#include <vector>

#include <pybind11/pybind11.h>
//#include <pybind11/chrono.h>
//#include <pybind11/eigen.h>
//#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <dynamical_systems/IDynamicalSystem.hpp>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace dynamical_systems;

void bind_cartesian(py::module_& m);
