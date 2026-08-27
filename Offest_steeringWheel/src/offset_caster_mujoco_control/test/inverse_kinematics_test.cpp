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
using offset_caster_mujoco_control::ChassisTwist;
using offset_caster_mujoco_control::InverseKinematics;
using offset_caster_mujoco_control::kCasterCount;

constexpr double kTolerance = 1.0e-10;
constexpr double kPi = 3.14159265358979323846;

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

void test_stationary_command()
{
  const InverseKinematics kinematics(model_geometry(), 6.283, 40.0);
  const auto command = kinematics.solve({0.0, 0.0, 0.0}, {0.3, -1.2, 2.1, -2.8});

  for (std::size_t i = 0; i < kCasterCount; ++i) {
    expect_near(command.steering[i], 0.0, "stationary steering");
    expect_near(command.wheel[i], 0.0, "stationary wheel");
  }
  expect_near(command.scale, 1.0, "stationary scale");
}

void test_forward_at_zero_steering()
{
  const InverseKinematics kinematics(model_geometry(), 6.283, 40.0);
  const auto command = kinematics.solve({0.15, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0});

  for (std::size_t i = 0; i < kCasterCount; ++i) {
    expect_near(command.steering[i], 0.0, "forward steering");
    expect_near(command.wheel[i], 2.0, "forward wheel");
  }
}

void test_left_at_quarter_turn()
{
  const InverseKinematics kinematics(model_geometry(), 6.283, 40.0);
  const std::array<double, kCasterCount> angles{kPi / 2.0, kPi / 2.0, kPi / 2.0, kPi / 2.0};
  const auto command = kinematics.solve({0.0, 0.15, 0.0}, angles);

  for (std::size_t i = 0; i < kCasterCount; ++i) {
    expect_near(command.steering[i], 0.0, "left steering");
    expect_near(command.wheel[i], 2.0, "left wheel");
  }
}

void test_paper_formula_conversion()
{
  const auto geometry = model_geometry();
  const InverseKinematics kinematics(geometry, 1000.0, 1000.0);
  const ChassisTwist twist{0.21, -0.13, 0.37};
  const std::array<double, kCasterCount> delta{0.2, -0.7, 1.4, -2.3};
  const auto command = kinematics.solve(twist, delta);

  for (std::size_t i = 0; i < kCasterCount; ++i) {
    const auto & caster = geometry[i];
    const double alpha = delta[i] + kPi;
    const double installation_angle = std::atan2(caster.pivot_y, caster.pivot_x);
    const double installation_radius = std::hypot(caster.pivot_x, caster.pivot_y);

    const double paper_wheel_speed =
      (twist.linear_x * std::cos(alpha) + twist.linear_y * std::sin(alpha) +
      twist.angular_z * installation_radius * std::sin(alpha - installation_angle)) /
      caster.wheel_radius;
    const double paper_steering_speed =
      (twist.linear_x * std::sin(alpha) - twist.linear_y * std::cos(alpha) -
      twist.angular_z * installation_radius * std::cos(installation_angle - alpha)) /
      caster.caster_offset - twist.angular_z;

    // The MJCF wheel-joint positive direction is opposite to the paper's
    // powered-wheel theta direction. The steering directions are identical.
    expect_near(command.wheel[i], -paper_wheel_speed, "paper wheel conversion");
    expect_near(command.steering[i], paper_steering_speed, "paper steering conversion");
  }
}

void test_uniform_speed_limit()
{
  const InverseKinematics kinematics(model_geometry(), 6.283, 40.0);
  const auto command = kinematics.solve({6.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0});

  expect_near(command.scale, 0.5, "limit scale");
  for (std::size_t i = 0; i < kCasterCount; ++i) {
    expect_near(command.steering[i], 0.0, "limited steering");
    expect_near(command.wheel[i], 40.0, "limited wheel");
  }
}

}  // namespace

int main()
{
  try {
    test_stationary_command();
    test_forward_at_zero_steering();
    test_left_at_quarter_turn();
    test_paper_formula_conversion();
    test_uniform_speed_limit();
  } catch (const std::exception & error) {
    std::cerr << "Inverse kinematics test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }

  std::cout << "All inverse kinematics tests passed.\n";
  return EXIT_SUCCESS;
}
