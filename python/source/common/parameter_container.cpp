#include "parameter_container.h"

ParameterContainer::ParameterContainer(const std::string& name, const StateType& type) :
    ParameterInterface(type, name) {}

ParameterContainer::ParameterContainer(const std::string& name, const py::object& value, const StateType& type) :
    ParameterInterface(type, name) {
  set_value(value);
}

ParameterContainer::ParameterContainer(const ParameterContainer& parameter) :
    ParameterInterface(parameter.get_type(), parameter.get_name()), values(parameter.values) {}

void ParameterContainer::set_value(const py::object& value) {
  switch (this->get_type()) {
    case StateType::PARAMETER_INT:
      values.int_value = value.cast<int>();
      break;
    case StateType::PARAMETER_INT_ARRAY:
      values.int_array_value = value.cast < std::vector < int >> ();
      break;
    case StateType::PARAMETER_DOUBLE:
      values.double_value = value.cast<double>();
      break;
    case StateType::PARAMETER_DOUBLE_ARRAY:
      values.double_array_value = value.cast < std::vector < double >> ();
      break;
    case StateType::PARAMETER_BOOL:
      values.bool_value = value.cast<bool>();
      break;
    case StateType::PARAMETER_BOOL_ARRAY:
      values.bool_array_value = value.cast < std::vector < bool >> ();
      break;
    case StateType::PARAMETER_STRING:
      values.string_value = value.cast<std::string>();
      break;
    case StateType::PARAMETER_STRING_ARRAY:
      values.string_array_value = value.cast < std::vector < std::string >> ();
      break;
    case StateType::PARAMETER_CARTESIANSTATE:
      values.cartesian_state = value.cast<CartesianState>();
      break;
    case StateType::PARAMETER_CARTESIANPOSE:
      values.cartesian_pose = value.cast<CartesianPose>();
      break;
    case StateType::PARAMETER_JOINTSTATE:
      values.joint_state = value.cast<JointState>();
      break;
    case StateType::PARAMETER_JOINTPOSITIONS:
      values.joint_positions = value.cast<JointPositions>();
      break;
    case StateType::PARAMETER_MATRIX:
      values.matrix_value = value.cast<Eigen::MatrixXd>();
      break;
    case StateType::PARAMETER_VECTOR:
      values.vector_value = value.cast<Eigen::VectorXd>();
      break;
    default:
      throw std::invalid_argument("This StateType is not a valid Parameter.");
      break;
  }
}

py::object ParameterContainer::get_value() {
  switch (this->get_type()) {
    case StateType::PARAMETER_INT:
      return py::cast(values.int_value);
    case StateType::PARAMETER_INT_ARRAY:
      return py::cast(values.int_array_value);
    case StateType::PARAMETER_DOUBLE:
      return py::cast(values.double_value);
    case StateType::PARAMETER_DOUBLE_ARRAY:
      return py::cast(values.double_array_value);
    case StateType::PARAMETER_BOOL:
      return py::cast(values.bool_value);
    case StateType::PARAMETER_BOOL_ARRAY:
      return py::cast(values.bool_array_value);
    case StateType::PARAMETER_STRING:
      return py::cast(values.string_value);
    case StateType::PARAMETER_STRING_ARRAY:
      return py::cast(values.string_array_value);
    case StateType::PARAMETER_CARTESIANSTATE:
      return py::cast(values.cartesian_state);
    case StateType::PARAMETER_CARTESIANPOSE:
      return py::cast(values.cartesian_pose);
    case StateType::PARAMETER_JOINTSTATE:
      return py::cast(values.joint_state);
    case StateType::PARAMETER_JOINTPOSITIONS:
      return py::cast(values.joint_positions);
    case StateType::PARAMETER_MATRIX:
      return py::cast(values.matrix_value);
    case StateType::PARAMETER_VECTOR:
      return py::cast(values.vector_value);
    default:
      return py::none();
  }
}

ParameterContainer parameter_interface_ptr_to_container(const std::shared_ptr<ParameterInterface>& parameter) {
  switch (parameter->get_type()) {
//      case StateType::PARAMETER_INT: {
//        return *std::static_pointer_cast<Parameter<int>>(parameter->get_name());
//        break;
//      }
//      case StateType::PARAMETER_INT_ARRAY: {
//        return std::static_pointer_cast<Parameter<std::vector<int>>>(parameter->get_name());
//        break;
//      }
//      case StateType::PARAMETER_DOUBLE: {
//        return std::static_pointer_cast<Parameter<double>>(parameter->get_name());
//        break;
//      }
//      case StateType::PARAMETER_DOUBLE_ARRAY: {
//        return std::static_pointer_cast<Parameter<std::vector<double>>>(parameter->get_name());
//        break;
//      }
//      case StateType::PARAMETER_BOOL: {
//        return std::static_pointer_cast<Parameter<bool>>(parameter->get_name());
//        break;
//      }
//      case StateType::PARAMETER_BOOL_ARRAY: {
//        return std::static_pointer_cast<Parameter<std::vector<bool>>>(parameter->get_name());
//        break;
//      }
//      case StateType::PARAMETER_STRING: {
//        return param = std::static_pointer_cast<Parameter<std::string>>(self->get_parameter(name));
//        break;
//      }
//      case StateType::PARAMETER_STRING_ARRAY: {
//        return param = std::static_pointer_cast<Parameter<std::vector<std::string>>>(self->get_parameter(name));
//        break;
//      }
    case StateType::PARAMETER_CARTESIANSTATE: {
      auto param = std::static_pointer_cast < Parameter < CartesianState >> (parameter);
      return ParameterContainer(param->get_name(), py::cast(param->get_value()), param->get_type());
      break;
    }
    case StateType::PARAMETER_CARTESIANPOSE: {
      auto param = std::static_pointer_cast < Parameter < CartesianPose >> (parameter);
      return ParameterContainer(param->get_name(), py::cast(param->get_value()), param->get_type());
      break;
    }
//      case StateType::PARAMETER_JOINTSTATE: {
//        return param = std::static_pointer_cast<Parameter<JointState>>(self->get_parameter(name));
//        break;
//      }
//      case StateType::PARAMETER_JOINTPOSITIONS: {
//        return param = std::static_pointer_cast<Parameter<JointPositions>>(self->get_parameter(name));
//        break;
//      }
    case StateType::PARAMETER_MATRIX: {
      auto param = std::static_pointer_cast < Parameter < Eigen::MatrixXd >> (parameter);
      return ParameterContainer(param->get_name(), py::cast(param->get_value()), param->get_type());
      break;
    }
//      case StateType::PARAMETER_VECTOR: {
//        return param = std::static_pointer_cast<Parameter<Eigen::VectorXd>>(self->get_parameter(name));
//        break;
//      }
    default:
      break;
  }
}

std::shared_ptr<ParameterInterface> container_to_parameter_interface_ptr(const ParameterContainer& parameter) {
  switch (parameter.get_type()) {
//      case StateType::PARAMETER_INT: {
//        return *std::static_pointer_cast<Parameter<int>>(parameter->get_name());
//        break;
//      }
//      case StateType::PARAMETER_INT_ARRAY: {
//        return std::static_pointer_cast<Parameter<std::vector<int>>>(parameter->get_name());
//        break;
//      }
//      case StateType::PARAMETER_DOUBLE: {
//        return std::static_pointer_cast<Parameter<double>>(parameter->get_name());
//        break;
//      }
//      case StateType::PARAMETER_DOUBLE_ARRAY: {
//        return std::static_pointer_cast<Parameter<std::vector<double>>>(parameter->get_name());
//        break;
//      }
//      case StateType::PARAMETER_BOOL: {
//        return std::static_pointer_cast<Parameter<bool>>(parameter->get_name());
//        break;
//      }
//      case StateType::PARAMETER_BOOL_ARRAY: {
//        return std::static_pointer_cast<Parameter<std::vector<bool>>>(parameter->get_name());
//        break;
//      }
//      case StateType::PARAMETER_STRING: {
//        return param = std::static_pointer_cast<Parameter<std::string>>(self->get_parameter(name));
//        break;
//      }
//      case StateType::PARAMETER_STRING_ARRAY: {
//        return param = std::static_pointer_cast<Parameter<std::vector<std::string>>>(self->get_parameter(name));
//        break;
//      }
    case StateType::PARAMETER_CARTESIANSTATE: {
      return std::make_shared<Parameter<CartesianState>>(parameter.get_name(), parameter.values.cartesian_state);
      break;
    }
    case StateType::PARAMETER_CARTESIANPOSE: {
      return std::make_shared<Parameter<CartesianPose>>(parameter.get_name(), parameter.values.cartesian_pose);
      break;
    }
//      case StateType::PARAMETER_JOINTSTATE: {
//        return param = std::static_pointer_cast<Parameter<JointState>>(self->get_parameter(name));
//        break;
//      }
//      case StateType::PARAMETER_JOINTPOSITIONS: {
//        return param = std::static_pointer_cast<Parameter<JointPositions>>(self->get_parameter(name));
//        break;
//      }
//    case StateType::PARAMETER_MATRIX: {
//      return std::make_shared<Parameter<Eigen::MatrixXd>>(parameter.get_name(), py::cast(parameter.values.matrix_value));
//      break;
//    }
//      case StateType::PARAMETER_VECTOR: {
//        return param = std::static_pointer_cast<Parameter<Eigen::VectorXd>>(self->get_parameter(name));
//        break;
//      }
    default:
      return {};
      break;
  }
}