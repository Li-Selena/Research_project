#ifndef OFFSET_CASTER_MUJOCO_CONTROL__MUJOCO_IMU_READER_HPP_
#define OFFSET_CASTER_MUJOCO_CONTROL__MUJOCO_IMU_READER_HPP_

#include <mujoco/mujoco.h>

#include <array>
#include <string>

namespace offset_caster_mujoco_control
{

struct MujocoImuState
{
  std::array<double, 4> orientation_wxyz{};
  std::array<double, 3> angular_velocity{};
  std::array<double, 3> linear_acceleration{};
};

class MujocoImuReader
{
public:
  MujocoImuReader(
    const mjModel * model,
    const std::string & orientation_sensor,
    const std::string & gyro_sensor,
    const std::string & accelerometer_sensor);

  [[nodiscard]] MujocoImuState read(const mjData * data) const;

private:
  int orientation_address_{-1};
  int gyro_address_{-1};
  int accelerometer_address_{-1};
};

}  // namespace offset_caster_mujoco_control

#endif  // OFFSET_CASTER_MUJOCO_CONTROL__MUJOCO_IMU_READER_HPP_
