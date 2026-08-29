#include "offset_caster_mujoco_control/mujoco_imu_reader.hpp"

#include <stdexcept>
#include <string>

namespace offset_caster_mujoco_control
{

namespace
{

int require_sensor(
  const mjModel * model, const std::string & name, const mjtSensor expected_type,
  const int expected_dimension)
{
  const int sensor_id = mj_name2id(model, mjOBJ_SENSOR, name.c_str());
  if (sensor_id < 0) {
    throw std::runtime_error("MuJoCo sensor not found: " + name);
  }
  if (model->sensor_type[sensor_id] != expected_type ||
    model->sensor_dim[sensor_id] != expected_dimension)
  {
    throw std::runtime_error("MuJoCo sensor has unexpected type or dimension: " + name);
  }
  return model->sensor_adr[sensor_id];
}

}  // namespace

MujocoImuReader::MujocoImuReader(
  const mjModel * model,
  const std::string & orientation_sensor,
  const std::string & gyro_sensor,
  const std::string & accelerometer_sensor)
{
  if (model == nullptr) {
    throw std::invalid_argument("MuJoCo model must not be null");
  }
  orientation_address_ = require_sensor(model, orientation_sensor, mjSENS_FRAMEQUAT, 4);
  gyro_address_ = require_sensor(model, gyro_sensor, mjSENS_GYRO, 3);
  accelerometer_address_ = require_sensor(
    model, accelerometer_sensor, mjSENS_ACCELEROMETER, 3);
}

MujocoImuState MujocoImuReader::read(const mjData * data) const
{
  if (data == nullptr) {
    throw std::invalid_argument("MuJoCo data must not be null");
  }
  MujocoImuState state;
  for (std::size_t i = 0; i < state.orientation_wxyz.size(); ++i) {
    state.orientation_wxyz[i] = data->sensordata[orientation_address_ + i];
  }
  for (std::size_t i = 0; i < state.angular_velocity.size(); ++i) {
    state.angular_velocity[i] = data->sensordata[gyro_address_ + i];
    state.linear_acceleration[i] = data->sensordata[accelerometer_address_ + i];
  }
  return state;
}

}  // namespace offset_caster_mujoco_control
