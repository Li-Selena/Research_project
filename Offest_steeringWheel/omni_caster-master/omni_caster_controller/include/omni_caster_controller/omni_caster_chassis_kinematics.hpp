#ifndef OMNI_CASTER_KINEMATICS_H
#define OMNI_CASTER_KINEMATICS_H

#include <iostream>
#include <vector>
#include <cmath>

namespace omni_caster_controller
{
  enum JointIndex_3 : size_t
  {
    FORWARD_CASTER = 0,
    LEFT_CASTER = 1,
    RIGHT_CASTER = 2
  };

  class OmniCasterChassisKinematics
  {

  protected:
    struct caster
    {
      double chassis_steering_angle;    // steering angle
      double wheel_angular_velocity;    // wheel angular velocity
      double steering_angular_velocity; // wheel angular velocity
      double phi;
      /* data */
    };

  public:
    double wheel_radius_;
    double offset_distance_;
    double caster_to_center_distance_;
    double caster_number_;

    OmniCasterChassisKinematics(int caster_num, double wheel_radius, double offset_distance, double caster_to_center_distance)
        : wheel_radius_(wheel_radius),
          offset_distance_(offset_distance),
          caster_to_center_distance_(caster_to_center_distance),
          caster_number_(caster_num)
    {
      casters.resize(caster_num);

      if (caster_num == 3)
      {
        casters[FORWARD_CASTER].phi = M_PI / 2.0; // single caster is fixed
        casters[LEFT_CASTER].phi = 7.0 * M_PI / 6.0;
        casters[RIGHT_CASTER].phi = 11.0 * M_PI / 6.0;
      }
      else
      {
        std::cerr << "Unsupported number of casters!" << std::endl;
      }
    }
    ~OmniCasterChassisKinematics();
    void InverseKinematics(const std::vector<double> &Vxyw_cmd, std::vector<caster> &casters);
    void ForwardKinematics(const std::vector<caster> &casters, std::vector<double> &Vxyw_obserce);
    std::vector<caster> casters;

  private:
    /* data */
  };
}
#endif