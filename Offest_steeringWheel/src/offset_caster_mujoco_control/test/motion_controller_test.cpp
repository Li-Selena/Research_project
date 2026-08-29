#include "offset_caster_mujoco_control/motion_controller.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{

using offset_caster_mujoco_control::MotionCommandData;
using offset_caster_mujoco_control::MotionController;
using offset_caster_mujoco_control::MotionControllerParameters;
using offset_caster_mujoco_control::MotionMode;
using offset_caster_mujoco_control::MotionState;
using offset_caster_mujoco_control::RobotPlanarState;

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1.0e-9;

MotionControllerParameters test_parameters()
{
  MotionControllerParameters parameters;
  parameters.max_linear_speed = 10.0;
  parameters.max_angular_speed = 10.0;
  parameters.max_linear_acceleration = 1000.0;
  parameters.max_angular_acceleration = 1000.0;
  parameters.yaw_rate_damping = 0.0;
  return parameters;
}

void expect_near(const double actual, const double expected, const std::string & label)
{
  if (std::abs(actual - expected) > kTolerance) {
    throw std::runtime_error(label + " mismatch");
  }
}

void set(MotionController & controller, const MotionCommandData & command, const RobotPlanarState & state)
{
  std::string error;
  if (!controller.set_command(command, state, error)) {
    throw std::runtime_error("Command rejected: " + error);
  }
}

void test_world_and_body_velocity_frames()
{
  RobotPlanarState state{0.0, 0.0, kPi / 2.0, 0.0};
  MotionController world_controller(test_parameters());
  set(world_controller, {1, MotionMode::WorldVelocityDynamicYaw, 1.0, 0.0, 0.0, 0.3}, state);
  const auto world = world_controller.update(state, 0.02);
  expect_near(world.twist.linear_x, 0.0, "world velocity x");
  expect_near(world.twist.linear_y, -1.0, "world velocity y");
  expect_near(world.twist.angular_z, 0.3, "world velocity yaw rate");

  MotionController body_controller(test_parameters());
  set(body_controller, {2, MotionMode::BodyVelocityDynamicYaw, 1.0, 0.0, 0.0, -0.2}, state);
  const auto body = body_controller.update(state, 0.02);
  expect_near(body.twist.linear_x, 1.0, "body velocity x");
  expect_near(body.twist.linear_y, 0.0, "body velocity y");
  expect_near(body.twist.angular_z, -0.2, "body velocity yaw rate");
}

void test_fixed_yaw_shortest_path()
{
  MotionController controller(test_parameters());
  RobotPlanarState state{0.0, 0.0, kPi - 0.1, 0.0};
  set(controller, {3, MotionMode::BodyVelocityFixedYaw, 0.2, 0.0, -kPi + 0.1, 0.0}, state);
  const auto output = controller.update(state, 0.02);
  expect_near(output.yaw_error, 0.2, "wrapped yaw error");
  expect_near(output.twist.angular_z, 0.4, "fixed yaw correction");
}

void test_position_yaw_modes_and_reached_dwell()
{
  const RobotPlanarState start{};
  MotionController fixed(test_parameters());
  set(fixed, {4, MotionMode::WorldPositionFixedYaw, 1.0, 0.0, kPi / 2.0, 0.0}, start);
  const auto fixed_output = fixed.update({0.5, 0.0, 0.0, 0.0}, 0.02);

  MotionController dynamic(test_parameters());
  const MotionCommandData dynamic_command{
    5, MotionMode::WorldPositionDynamicYaw, 1.0, 0.0, kPi / 2.0, 0.0};
  set(dynamic, dynamic_command, start);
  const RobotPlanarState halfway{0.5, 0.0, 0.0, 0.0};
  const auto dynamic_output = dynamic.update(halfway, 0.02);
  if (!(dynamic_output.twist.angular_z > 0.0 &&
    dynamic_output.twist.angular_z < fixed_output.twist.angular_z))
  {
    throw std::runtime_error("Dynamic position yaw must progress smoothly toward final yaw");
  }
  set(dynamic, dynamic_command, halfway);
  const auto repeated = dynamic.update(halfway, 0.02);
  expect_near(repeated.twist.angular_z, dynamic_output.twist.angular_z, "repeated command progress");

  const RobotPlanarState target{1.0, 0.0, kPi / 2.0, 0.0};
  auto output = dynamic.update(target, 0.1);
  output = dynamic.update(target, 0.1);
  if (output.state != MotionState::Reached) {
    throw std::runtime_error("Position target must reach after dwell time");
  }
  expect_near(output.twist.linear_x, 0.0, "reached linear velocity");
  expect_near(output.twist.angular_z, 0.0, "reached angular velocity");
}

void test_invalid_command()
{
  MotionController controller(test_parameters());
  MotionCommandData command;
  command.mode = MotionMode::WorldVelocityFixedYaw;
  command.x = std::numeric_limits<double>::quiet_NaN();
  std::string error;
  if (controller.set_command(command, {}, error)) {
    throw std::runtime_error("Non-finite command must be rejected");
  }
}

}  // namespace

int main()
{
  try {
    test_world_and_body_velocity_frames();
    test_fixed_yaw_shortest_path();
    test_position_yaw_modes_and_reached_dwell();
    test_invalid_command();
  } catch (const std::exception & error) {
    std::cerr << "Motion controller test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  std::cout << "All motion controller tests passed.\n";
  return EXIT_SUCCESS;
}
