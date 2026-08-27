#include "offset_caster_mujoco_control/forward_kinematics.hpp"
#include "offset_caster_mujoco_control/inverse_kinematics.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

using offset_caster_mujoco_control::CasterGeometry;
using offset_caster_mujoco_control::CasterJointState;
using offset_caster_mujoco_control::ChassisTwist;
using offset_caster_mujoco_control::ForwardKinematics;
using offset_caster_mujoco_control::InverseKinematics;
using offset_caster_mujoco_control::kCasterCount;

constexpr double kTolerance = 1.0e-10;

std::array<CasterGeometry, kCasterCount> model_geometry()
{
  return {{{0.5, 0.35, 0.075, 0.105},
    {0.5, -0.35, 0.075, 0.105},
    {-0.5, -0.35, 0.075, 0.105},
    {-0.5, 0.35, 0.075, 0.105}}};
}

void expect_near(const double actual, const double expected, const std::string & label)
{
  if (std::abs(actual - expected) > kTolerance) {
    throw std::runtime_error(
            label + ": expected " + std::to_string(expected) + ", got " +
            std::to_string(actual));
  }
}

void check_round_trip(
  const ChassisTwist & expected_twist,
  const std::array<double, kCasterCount> & steering_angles)
{
  const auto geometry = model_geometry();
  const InverseKinematics inverse(geometry, 1.0e6, 1.0e6);
  const ForwardKinematics forward(geometry);
  const auto joint_command = inverse.solve(expected_twist, steering_angles);

  std::array<CasterJointState, kCasterCount> joint_states{};
  for (std::size_t i = 0; i < kCasterCount; ++i) {
    joint_states[i] = {
      steering_angles[i], joint_command.steering[i], 0.0, joint_command.wheel[i]};
  }

  const auto result = forward.solve(joint_states);
  expect_near(result.twist.linear_x, expected_twist.linear_x, "round-trip vx");
  expect_near(result.twist.linear_y, expected_twist.linear_y, "round-trip vy");
  expect_near(result.twist.angular_z, expected_twist.angular_z, "round-trip wz");
  expect_near(result.residual_rms, 0.0, "round-trip residual");
}

void test_stationary()
{
  check_round_trip({0.0, 0.0, 0.0}, {0.0, 0.7, -1.3, 2.2});
}

void test_general_motion_at_zero_steering()
{
  check_round_trip({0.24, -0.17, 0.38}, {0.0, 0.0, 0.0, 0.0});
}

void test_general_motion_at_mixed_steering()
{
  check_round_trip({-0.31, 0.22, -0.47}, {0.3, -0.9, 1.7, -2.4});
}

}  // namespace

int main()
{
  try {
    test_stationary();
    test_general_motion_at_zero_steering();
    test_general_motion_at_mixed_steering();
  } catch (const std::exception & error) {
    std::cerr << "Forward kinematics test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }

  std::cout << "All forward kinematics tests passed.\n";
  return EXIT_SUCCESS;
}
