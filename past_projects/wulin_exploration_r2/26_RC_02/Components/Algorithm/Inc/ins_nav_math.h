#ifndef __INS_NAV_MATH_H
#define __INS_NAV_MATH_H

#include <stdint.h>
#include <stddef.h>

#define INS_NAV_PI_F       3.14159265358979323846f
#define INS_NAV_2PI_F      6.28318530717958647692f
#define INS_NAV_DEG2RAD_F  0.01745329251994329577f
#define INS_NAV_RAD2DEG_F  57.2957795130823208768f

float INS_NavMath_WrapDeg(float angle_deg);
float INS_NavMath_WrapRad(float angle_rad);
float INS_NavMath_UpdateContinuousYawDeg(float yaw_deg,
                                         float *last_yaw_deg,
                                         int16_t *round_count);
/* FLU planar transforms. At yaw=0 body and world axes coincide; positive yaw
 * rotates body +X toward world +Y. Input and output pointers may alias. */
void INS_NavMath_RobotToWorld(float robot_x,
                              float robot_y,
                              float yaw_rad,
                              float *world_x,
                              float *world_y);
void INS_NavMath_WorldToRobot(float world_x,
                              float world_y,
                              float yaw_rad,
                              float *robot_x,
                              float *robot_y);

#endif /* __INS_NAV_MATH_H */
