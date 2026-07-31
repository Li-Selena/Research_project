#include "omni_caster_controller/omni_caster_chassis_kinematics.hpp"
// #include <math.h>
namespace omni_caster_controller
{
    void OmniCasterChassisKinematics::InverseKinematics(const std::vector<double> &Vxyw_cmd, std::vector<caster> &casters)
    {
        double Vx = Vxyw_cmd[0];
        double Vy = Vxyw_cmd[1];
        double Wz = Vxyw_cmd[2];

        for (int i = 0; i < caster_number_; ++i)
        {

            // double wheel_radius_;
            // double offset_distance_;
            // double caster_to_center_distance_;
            // double caster_number_;
            double cos_ai = cos(casters[i].chassis_steering_angle);
            double sin_ai = sin(casters[i].chassis_steering_angle);
            double sin_ai_phi = sin(casters[i].chassis_steering_angle - casters[i].phi);
            double cos_ai_phi = cos(casters[i].chassis_steering_angle - casters[i].phi);

            casters[i].wheel_angular_velocity = (Vy * cos_ai + Vx * sin_ai + Wz * caster_to_center_distance_ * sin_ai_phi) / wheel_radius_;
            casters[i].steering_angular_velocity = -Wz - (Vy * sin_ai - Vx * cos_ai - Wz * caster_to_center_distance_ * cos_ai_phi) / offset_distance_; // Placeholder
        }
    }
    void OmniCasterChassisKinematics::ForwardKinematics(const std::vector<caster> &casters, std::vector<double> &Vxyw_obserce)
    {

        // w = [rdtheat2 - rdtheat1cos(aph1 - aph2) + ddaph1sin(aph1 - aph2)] / [lsin(aph2 - ph2) - l * sin(aph1 - ph1) cos(aph1 - aph2) + lcos(aph1 - ph1) sin(aph1 - aph2) - dsin(aph1 - aph2)];
        // vx = [rdtheat1 - wlsin(aph1 - ph1)] sin(aph1) - [-ddaph1 - dw + wlcos(aph1 - ph1)] cos(aph1);
        // vy = [rdtheat1 - wlsin(aph1 - ph1)] cos(aph1) + [-ddaph1 - dw + wlcos(aph1 - ph1)] sin(aph1);

        if (caster_number_ < 2)
        {
            std::cerr << "Error: At least 2 casters required for forward kinematics!" << std::endl;
            return;
        }

        // 使用前两个舵轮进行计算
        double dtheta1 = casters[0].wheel_angular_velocity;
        double dtheta2 = casters[1].wheel_angular_velocity;
        double daph1 = casters[0].steering_angular_velocity;

        double aph1 = casters[0].chassis_steering_angle;
        double aph2 = casters[1].chassis_steering_angle;
        double ph1 = casters[0].phi;
        double ph2 = casters[1].phi;

        // 预计算三角函数值
        double sin_aph1 = std::sin(aph1);
        double cos_aph1 = std::cos(aph1);
        double sin_aph2 = std::sin(aph2);
        double cos_aph2 = std::cos(aph2);
        double sin_ph1 = std::sin(ph1);
        double cos_ph1 = std::cos(ph1);
        double sin_ph2 = std::sin(ph2);
        double cos_ph2 = std::cos(ph2);

        // 计算 Wz (角速度)
        double w_numerator = wheel_radius_ * dtheta2 - wheel_radius_ * dtheta1 * cos_aph1 * cos_aph2 - wheel_radius_ * dtheta1 * sin_aph1 * sin_aph2 + offset_distance_ * daph1 * sin_aph1 * cos_aph2 - offset_distance_ * daph1 * cos_aph1 * sin_aph2;

        double w_denominator = caster_to_center_distance_ * sin_aph2 * cos_ph2 - caster_to_center_distance_ * cos_aph2 * sin_ph2 + caster_to_center_distance_ * sin_ph1 * cos_aph2 - caster_to_center_distance_ * cos_ph1 * sin_aph2 - offset_distance_ * sin_aph1 * cos_aph2 + offset_distance_ * cos_aph1 * sin_aph2;

        double Wz = 0.0;
        if (std::abs(w_denominator) < 1e-10)
        {
            std::cerr << "Warning: Division by near-zero in Wz calculation!" << std::endl;
        }
        else
        {
            Wz = w_numerator / w_denominator;
        }

        // 计算 Vx (X方向速度)
        double Vx = wheel_radius_ * dtheta1 * sin_aph1 + offset_distance_ * daph1 * cos_aph1 + offset_distance_ * Wz * cos_aph1 - Wz * caster_to_center_distance_ * cos_ph1;

        // 计算 Vy (Y方向速度)
        double Vy = wheel_radius_ * dtheta1 * cos_aph1 - offset_distance_ * daph1 * sin_aph1 - offset_distance_ * Wz * sin_aph1 + Wz * caster_to_center_distance_ * sin_ph1;

        // 输出结果
        Vxyw_obserce.resize(3);
        Vxyw_obserce[0] = Vx;
        Vxyw_obserce[1] = Vy;
        Vxyw_obserce[2] = Wz;
    }

    // 辅助函数：角度归一化到 [-pi, pi]
    static double NormalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    OmniCasterChassisKinematics::~OmniCasterChassisKinematics()
    {
        casters.clear();
    }

}
