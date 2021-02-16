#pragma once

#include "controllers/Controller.hpp"
#include "state_representation/Parameters/Parameter.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace controllers {
namespace impedance {
/**
 * @class Impedance
 * @brief Definition of an impedance controller in either joint or task space
 * @tparam S the space of the controller either CartesianState or JointState
 */
template <class S>
class Impedance : Controller<S, S> {
private:
  std::shared_ptr<StateRepresentation::Parameter<Eigen::MatrixXd>> stiffness_;///< stiffness matrix of the controller associated to position
  std::shared_ptr<StateRepresentation::Parameter<Eigen::MatrixXd>> damping_;  ///< damping matrix of the controller associated to velocity
  std::shared_ptr<StateRepresentation::Parameter<Eigen::MatrixXd>> inertia_;  ///< inertia matrix of the controller associated to acceleration

public:
  /**
   * @brief Constructor initializing all the matrices
   * @param stiffness the stiffness matrix
   * @param damping the damping matrix
   * @param inertia the inertia matrix
   */
  explicit Impedance(const Eigen::MatrixXd& stiffness, const Eigen::MatrixXd& damping, const Eigen::MatrixXd& inertia);

  /**
   * @brief Compute the force (task space) or torque (joint space) command based on the input state 
   * of the system as the error between the desired state and the real state.
   * @param desired_state the desired state to reach
   * @param feedback_state the real state of the system as read from feedback loop
   * @return the output command at the input state
   */
  virtual S compute_command(const S& desired_state, const S& feedback_state) const;

  /**
   * @brief Getter of the stiffness matrix
   * @return the stiffness matrix
   */
  const Eigen::MatrixXd& get_stiffness() const;

  /**
   * @brief Getter of the dmaping matrix
   * @return the damping matrix
   */
  const Eigen::MatrixXd& get_damping() const;

  /**
   * @brief Getter of the inertia matrix
   * @return the inertia matrix
   */
  const Eigen::MatrixXd& get_inertia() const;

  /**
   * @brief Setter of the stiffness matrix
   * @param the new stiffness matrix
   */
  void set_stiffness(const Eigen::MatrixXd& stiffness);

  /**
   * @brief Setter of the damping matrix
   * @param the new damping matrix
   */
  void set_damping(const Eigen::MatrixXd& damping);

  /**
   * @brief Setter of the inertia matrix
   * @param the new inertia matrix
   */
  void set_inertia(const Eigen::MatrixXd& inertia);

  /**
   * @brief Return a list of all the parameters of the controller
   * @return the list of parameters
   */
  virtual std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> get_parameters() const;
};

template <class S>
inline const Eigen::MatrixXd& Impedance<S>::get_stiffness() const {
  return this->stiffness_->get_value();
}

template <class S>
inline const Eigen::MatrixXd& Impedance<S>::get_damping() const {
  return this->damping_->get_value();
}

template <class S>
inline const Eigen::MatrixXd& Impedance<S>::get_inertia() const {
  return this->inertia_->get_value();
}

template <class S>
inline void Impedance<S>::set_stiffness(const Eigen::MatrixXd& stiffness) {
  this->stiffness_->set_value(stiffness);
}

template <class S>
inline void Impedance<S>::set_damping(const Eigen::MatrixXd& damping) {
  this->damping_->set_value(damping);
}

template <class S>
inline void Impedance<S>::set_inertia(const Eigen::MatrixXd& inertia) {
  this->inertia_->set_value(inertia);
}

template <class S>
std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> Impedance<S>::get_parameters() const {
  std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> param_list;
  param_list.push_back(this->stiffness_);
  param_list.push_back(this->damping_);
  param_list.push_back(this->inertia_);
  return param_list;
}
}// namespace impedance
}// namespace controllers