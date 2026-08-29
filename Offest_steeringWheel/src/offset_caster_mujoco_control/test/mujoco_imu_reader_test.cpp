#include "offset_caster_mujoco_control/mujoco_imu_reader.hpp"

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{

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
  mj_forward(model.get(), data.get());

  offset_caster_mujoco_control::MujocoImuReader reader(
    model.get(), "imu_orientation", "base_angular_velocity", "base_linear_acceleration");
  const auto state = reader.read(data.get());
  double norm = 0.0;
  for (const double value : state.orientation_wxyz) {
    if (!std::isfinite(value)) {
      throw std::runtime_error("Non-finite IMU orientation");
    }
    norm += value * value;
  }
  if (std::abs(norm - 1.0) > 1.0e-10) {
    throw std::runtime_error("IMU orientation is not a unit quaternion");
  }
  for (const double value : state.angular_velocity) {
    if (!std::isfinite(value)) {
      throw std::runtime_error("Non-finite IMU gyro value");
    }
  }
  for (const double value : state.linear_acceleration) {
    if (!std::isfinite(value)) {
      throw std::runtime_error("Non-finite IMU accelerometer value");
    }
  }
}

}  // namespace

int main(int argc, char * argv[])
{
  if (argc != 2) {
    std::cerr << "Usage: mujoco_imu_reader_test MODEL_PATH\n";
    return EXIT_FAILURE;
  }
  try {
    run_test(argv[1]);
  } catch (const std::exception & error) {
    std::cerr << "MuJoCo IMU-reader test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  std::cout << "MuJoCo IMU-reader test passed.\n";
  return EXIT_SUCCESS;
}
