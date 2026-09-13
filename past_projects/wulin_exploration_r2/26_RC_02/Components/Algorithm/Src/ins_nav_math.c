#include "ins_nav_math.h"
#include <math.h>

float INS_NavMath_WrapDeg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

float INS_NavMath_WrapRad(float angle_rad)
{
    while (angle_rad > INS_NAV_PI_F)
    {
        angle_rad -= INS_NAV_2PI_F;
    }
    while (angle_rad < -INS_NAV_PI_F)
    {
        angle_rad += INS_NAV_2PI_F;
    }
    return angle_rad;
}

float INS_NavMath_UpdateContinuousYawDeg(float yaw_deg,
                                         float *last_yaw_deg,
                                         int16_t *round_count)
{
    float delta;

    if ((last_yaw_deg == NULL) || (round_count == NULL))
    {
        return yaw_deg;
    }

    delta = yaw_deg - *last_yaw_deg;
    if (delta < -180.0f)
    {
        (*round_count)++;
    }
    else if (delta > 180.0f)
    {
        (*round_count)--;
    }

    *last_yaw_deg = yaw_deg;
    return yaw_deg + (float)(*round_count) * 360.0f;
}

void INS_NavMath_RobotToWorld(float robot_x,
                              float robot_y,
                              float yaw_rad,
                              float *world_x,
                              float *world_y)
{
    float sin_yaw;
    float cos_yaw;
    float result_x;
    float result_y;

    if ((world_x == NULL) || (world_y == NULL))
    {
        return;
    }

    sin_yaw = sinf(yaw_rad);
    cos_yaw = cosf(yaw_rad);

    result_x = robot_x * cos_yaw - robot_y * sin_yaw;
    result_y = robot_x * sin_yaw + robot_y * cos_yaw;
    *world_x = result_x;
    *world_y = result_y;
}

void INS_NavMath_WorldToRobot(float world_x,
                              float world_y,
                              float yaw_rad,
                              float *robot_x,
                              float *robot_y)
{
    float sin_yaw;
    float cos_yaw;
    float result_x;
    float result_y;

    if ((robot_x == NULL) || (robot_y == NULL))
    {
        return;
    }

    sin_yaw = sinf(yaw_rad);
    cos_yaw = cosf(yaw_rad);

    result_x =  world_x * cos_yaw + world_y * sin_yaw;
    result_y = -world_x * sin_yaw + world_y * cos_yaw;
    *robot_x = result_x;
    *robot_y = result_y;
}
