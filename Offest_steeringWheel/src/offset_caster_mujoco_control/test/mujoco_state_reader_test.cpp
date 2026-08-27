#include "offset_caster_mujoco_control/forward_kinematics.hpp"
#include "offset_caster_mujoco_control/inverse_kinematics.hpp"
#include "offset_caster_mujoco_control/mujoco_state_reader.hpp"

#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{

using offset_caster_mujoco_control::CasterGeometry;
using offset_caster_mujoco_control::CasterJointState;
using offset_caster_mujoco_control::ChassisTwist;
using offset_caster_mujoco_control::ForwardKinematics;
using offset_caster_mujoco_control::InverseKinematics;
using offset_caster_mujoco_control::MujocoStateReader;
using offset_caster_mujoco_control::kCasterCount;

constexpr double kTolerance = 1.0e-10;

std::array<CasterGeometry, kCasterCount> model_geometry()
{
  return {{{0.5, 0.35, 0.075, 0.105},
    {0.5, -0.35, 0.075, 0.105},
    {-0.5, -0.35, 0.075, 0.105},
    {-0.5, 0.35, 0.075, 0.105}}};
}

const std::array<std::string, kCasterCount> kSteeringNames{
  "steering_joint_1", "steering_joint_2", "steering_joint_3", "steering_joint_4"};
const std::array<std::string, kCasterCount> kWheelNames{
  "wheel_joint_1", "wheel_joint_2", "wheel_joint_3", "wheel_joint_4"};

void expect_near(const double actual, const double expected, const std::string & label)
{
  if (std::abs(actual - expected) > kTolerance) {
    throw std::runtime_error(
            label + ": expected " + std::to_string(expected) + ", got " +
            std::to_string(actual));
  }
}

int require_joint(const mjModel * model, const std::string & name)
{
  const int id = mj_name2id(model, mjOBJ_JOINT, name.c_str());
  if (id < 0) {
    throw std::runtime_error("Joint missing from test model: " + name);
  }
  return id;
}

void run_test(const std::string & model_path)
{
  char error[1024]{};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
    mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
  if (!model) {
    throw std::runtime_error("mj_loadXML failed: " + std::string(error));
  }
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
  if (!data) {
    throw std::runtime_error("mj_makeData failed");
  }

  const auto geometry = model_geometry();
  const MujocoStateReader reader(model.get(), kSteeringNames, kWheelNames);
  const InverseKinematics inverse(geometry, 1.0e6, 1.0e6);
  const ForwardKinematics forward(geometry);
  const ChassisTwist expected_twist{0.23, -0.16, 0.41};
  const std::array<double, kCasterCount> steering_angles{0.2, -0.8, 1.5, -2.2};
  const auto command = inverse.solve(expected_twist, steering_angles);

  for (std::size_t i = 0; i < kCasterCount; ++i) {
    const int steering_id = require_joint(model.get(), kSteeringNames[i]);
    const int wheel_id = require_joint(model.get(), kWheelNames[i]);
    data->qpos[model->jnt_qposadr[steering_id]] = steering_angles[i];
    data->qvel[model->jnt_dofadr[steering_id]] = command.steering[i];
    data->qpos[model->jnt_qposadr[wheel_id]] = 0.1 * static_cast<double>(i + 1);
    data->qvel[model->jnt_dofadr[wheel_id]] = command.wheel[i];
  }
  mj_forward(model.get(), data.get());

  const auto states = reader.read(data.get());
  for (std::size_t i = 0; i < kCasterCount; ++i) {
    expect_near(states[i].steering_angle, steering_angles[i], "MuJoCo steering angle");
    expect_near(states[i].steering_speed, command.steering[i], "MuJoCo steering speed");
    expect_near(states[i].wheel_angle, 0.1 * static_cast<double>(i + 1), "MuJoCo wheel angle");
    expect_near(states[i].wheel_speed, command.wheel[i], "MuJoCo wheel speed");
  }

  const auto result = forward.solve(states);
  expect_near(result.twist.linear_x, expected_twist.linear_x, "MuJoCo round-trip vx");
  expect_near(result.twist.linear_y, expected_twist.linear_y, "MuJoCo round-trip vy");
  expect_near(result.twist.angular_z, expected_twist.angular_z, "MuJoCo round-trip wz");
  expect_near(result.residual_rms, 0.0, "MuJoCo round-trip residual");
}

}  // namespace

int main(int argc, char * argv[])
{
  if (argc != 2) {
    std::cerr << "Usage: mujoco_state_reader_test MODEL_PATH\n";
    return EXIT_FAILURE;
  }

  try {
    run_test(argv[1]);
  } catch (const std::exception & error) {
    std::cerr << "MuJoCo state-reader test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }

  std::cout << "MuJoCo state-reader and forward-kinematics test passed.\n";
  return EXIT_SUCCESS;
}
