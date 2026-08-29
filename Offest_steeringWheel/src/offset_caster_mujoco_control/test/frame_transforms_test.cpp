#include "offset_caster_mujoco_control/frame_transforms.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1.0e-10;

void expect_near(const double actual, const double expected)
{
  if (std::abs(actual - expected) > kTolerance) {
    throw std::runtime_error("Unexpected transform result");
  }
}

void run_tests()
{
  using offset_caster_mujoco_control::body_to_world;
  using offset_caster_mujoco_control::normalize_angle;
  using offset_caster_mujoco_control::quaternion_to_yaw;
  using offset_caster_mujoco_control::smoothstep;
  using offset_caster_mujoco_control::world_to_body;

  const auto body = world_to_body(1.0, 0.0, kPi / 2.0);
  expect_near(body[0], 0.0);
  expect_near(body[1], -1.0);
  const auto world = body_to_world(body[0], body[1], kPi / 2.0);
  expect_near(world[0], 1.0);
  expect_near(world[1], 0.0);
  expect_near(normalize_angle(kPi + 0.2), -kPi + 0.2);
  expect_near(quaternion_to_yaw(std::cos(kPi / 4.0), 0.0, 0.0, std::sin(kPi / 4.0)), kPi / 2.0);
  expect_near(smoothstep(-1.0), 0.0);
  expect_near(smoothstep(0.5), 0.5);
  expect_near(smoothstep(2.0), 1.0);
}

}  // namespace

int main()
{
  try {
    run_tests();
  } catch (const std::exception & error) {
    std::cerr << "Frame transform test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  std::cout << "All frame transform tests passed.\n";
  return EXIT_SUCCESS;
}
