#include <gtest/gtest.h>

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/NotImplementedException.hpp"

using namespace state_representation;

TEST(CartesianStateTest, IdentityInitialization) {
  CartesianState identity = CartesianState::Identity("test");
  // the joint state should not be considered empty (as it is properly initialized)
  EXPECT_FALSE(identity.is_empty());
  // all data should be zero except for orientation that should be identity
  EXPECT_EQ(identity.get_position().norm(), 0);
  EXPECT_EQ(identity.get_orientation().norm(), 1);
  EXPECT_EQ(identity.get_orientation().w(), 1);
  EXPECT_EQ(identity.get_twist().norm(), 0);
  EXPECT_EQ(identity.get_accelerations().norm(), 0);
  EXPECT_EQ(identity.get_wrench().norm(), 0);
}

TEST(CartesianStateTest, RandomStateInitialization) {
  CartesianState random = CartesianState::Random("test");
  // all data should be random (non 0)
  EXPECT_NE(random.get_position().norm(), 0);
  EXPECT_NE(abs(random.get_orientation().w()), 0);
  EXPECT_NE(abs(random.get_orientation().x()), 0);
  EXPECT_NE(abs(random.get_orientation().y()), 0);
  EXPECT_NE(abs(random.get_orientation().z()), 0);
  EXPECT_NE(random.get_twist().norm(), 0);
  EXPECT_NE(random.get_accelerations().norm(), 0);
  EXPECT_NE(random.get_wrench().norm(), 0);
}

TEST(CartesianStateTest, CopyState) {
  CartesianState state1 = CartesianState::Random("test");
  CartesianState state2(state1);
  EXPECT_EQ(state1.get_name(), state2.get_name());
  EXPECT_EQ(state1.get_reference_frame(), state2.get_reference_frame());
  EXPECT_TRUE(state1.data().isApprox(state2.data()));
  CartesianState state3 = state1;
  EXPECT_EQ(state1.get_name(), state3.get_name());
  EXPECT_EQ(state1.get_reference_frame(), state3.get_reference_frame());
  EXPECT_TRUE(state1.data().isApprox(state3.data()));

  CartesianState state4;
  EXPECT_TRUE(state4.is_empty());
  CartesianState state5 = state4;
  EXPECT_TRUE(state5.is_empty());
}

TEST(CartesianStateTest, GetData) {
  CartesianState cs = CartesianState::Random("test");
  Eigen::VectorXd concatenated_state(25);
  concatenated_state << cs.get_pose(), cs.get_twist(), cs.get_accelerations(), cs.get_wrench();
  EXPECT_TRUE(concatenated_state.isApprox(cs.data()));
}

TEST(CartesianStateTest, SetData) {
  CartesianState cs1 = CartesianState::Identity("test");
  CartesianState cs2 = CartesianState::Random("test");
  cs1.set_data(cs2.data());
  EXPECT_TRUE(cs2.data().isApprox(cs1.data()));

  auto state_vec = cs2.to_std_vector();
  cs1.set_data(state_vec);
  for (std::size_t j = 0; j < state_vec.size(); ++j) {
    EXPECT_FLOAT_EQ(state_vec.at(j), cs1.data()(j));
  }
  EXPECT_THROW(cs1.set_data(Eigen::Vector3d::Zero()), exceptions::IncompatibleSizeException);
}

TEST(CartesianStateTest, CartesianStateToStdVector) {
  CartesianState cs = CartesianState::Random("test");
  std::vector<double> vec_data = cs.to_std_vector();
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(cs.data()(i), vec_data[i]);
  }
}

TEST(CartesianStateTest, TestStateClamping) {
  CartesianState state = CartesianState::Identity("test");
  EXPECT_THROW(state.clamp_state_variable(1, CartesianStateVariable::ORIENTATION), exceptions::NotImplementedException);
  EXPECT_THROW(state.clamp_state_variable(1, CartesianStateVariable::POSE), exceptions::NotImplementedException);
  Eigen::Vector3d position(-2.0, 1, 5);
  state.set_position(position);
  state.clamp_state_variable(10.0, CartesianStateVariable::POSITION);
  EXPECT_EQ(state.get_position(), position);
  state.clamp_state_variable(3.0, CartesianStateVariable::POSITION);
  EXPECT_EQ(state.get_position().norm(), 3.0);
  state.clamp_state_variable(10.0, CartesianStateVariable::POSITION, 0.5);
  EXPECT_EQ(state.get_position().norm(), 0.0);
}

TEST(CartesianStateTest, TestAllNorms) {
  double tolerance = 1e-4;
  CartesianState cs = CartesianState::Random("cs");
  // first test all norms
  std::vector<double> norms = cs.norms();
  ASSERT_TRUE(norms.size() == 8);
  EXPECT_NEAR(norms[0], cs.get_position().norm(), tolerance);
  EXPECT_NEAR(norms[1], cs.get_orientation().norm(), tolerance);
  EXPECT_NEAR(norms[2], cs.get_linear_velocity().norm(), tolerance);
  EXPECT_NEAR(norms[3], cs.get_angular_velocity().norm(), tolerance);
  EXPECT_NEAR(norms[4], cs.get_linear_acceleration().norm(), tolerance);
  EXPECT_NEAR(norms[5], cs.get_angular_acceleration().norm(), tolerance);
  EXPECT_NEAR(norms[6], cs.get_force().norm(), tolerance);
  EXPECT_NEAR(norms[7], cs.get_torque().norm(), tolerance);
}

TEST(CartesianStateTest, TestNormalize) {
  double tolerance = 1e-4;
  CartesianState cs = CartesianState::Random("cs");
  cs.normalize();
  std::vector<double> norms = cs.norms();
  for (double n: norms) {
    EXPECT_NEAR(n, 1.0, tolerance);
  }
}

TEST(CartesianStateTest, TestNormalized) {
  double tolerance = 1e-4;
  CartesianState cs = CartesianState::Random("cs");
  CartesianState csn = cs.normalized();
  std::vector<double> norms = csn.norms();
  for (double n: norms) {
    EXPECT_NEAR(n, 1.0, tolerance);
  }
}

TEST(CartesianStateTest, TestScalarDivison) {
  double scalar = 2;
  CartesianState cs = CartesianState::Random("test");
  CartesianState cscaled = cs / scalar;
  EXPECT_EQ(cscaled.get_position(), cs.get_position() / scalar);
  Eigen::Quaterniond qscaled = math_tools::exp(math_tools::log(cs.get_orientation()), 1.0 / (2. * scalar));
  EXPECT_TRUE(cscaled.get_orientation().coeffs().isApprox(qscaled.coeffs()));
  EXPECT_EQ(cscaled.get_twist(), cs.get_twist() / scalar);
  EXPECT_EQ(cscaled.get_accelerations(), cs.get_accelerations() / scalar);
  EXPECT_EQ(cscaled.get_wrench(), cs.get_wrench() / scalar);
  cs /= scalar;
  EXPECT_EQ(cscaled.data(), cs.data());

  EXPECT_THROW(cs / 0.0, std::runtime_error);

  CartesianState empty;
  EXPECT_THROW(empty / scalar, exceptions::EmptyStateException);
}

TEST(CartesianStateTest, TestOperators) {
  CartesianState state = CartesianState::Random("world");
  CartesianPose pose = CartesianPose::Random("world");
  CartesianTwist twist = CartesianTwist::Random("world");
  CartesianWrench wrench = CartesianWrench::Random("world");

  auto r1 = state * state;
  std::cout << r1 << std::endl;
  auto r2 = state * pose;
  std::cout << r2 << std::endl;
  auto r3 = state * twist;
  std::cout << r3 << std::endl;
  auto r10 = state * wrench;
  std::cout << r10 << std::endl;
  auto r4 = pose * state;
  std::cout << r4 << std::endl;
  auto r5 = pose * pose;
  std::cout << r5 << std::endl;
  auto r6 = pose * twist;
  std::cout << r6 << std::endl;
  auto r11 = pose * wrench;
  std::cout << r11 << std::endl;
//  auto r7 = twist * state;
//  std::cout << r7 << std::endl;
//  auto r8 = twist * pose;
//  std::cout << r8 << std::endl;
//  auto r9 = twist * twist;
//  std::cout << r9 << std::endl;
//  auto r12 = twist * wrench;
//  std::cout << r12 << std::endl;
//  auto r13 = wrench * wrench;
//  std::cout << r13 << std::endl;
//  auto r14 = wrench * state;
//  std::cout << r14 << std::endl;
//  auto r15 = wrench * pose;
//  std::cout << r15 << std::endl;
//  auto r16 = wrench * twist;
//  std::cout << r16 << std::endl;

  state *= state;
//  state *= pose;
//  state *= twist;
//  state *= wrench;
//  pose *= state;
  pose *= pose;
//  pose *= twist;
//  pose *= wrench;
//  twist *= state;
//  twist *= pose;
//  twist *= twist;
//  twist *= wrench;
//  wrench *= state;
//  wrench *= pose;
//  wrench *= twist;
//  wrench *= wrench;
}
