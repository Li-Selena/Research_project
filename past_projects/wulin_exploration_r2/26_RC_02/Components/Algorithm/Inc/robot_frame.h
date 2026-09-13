#ifndef ROBOT_FRAME_H
#define ROBOT_FRAME_H

#include <math.h>

/* Application frame: +X forward, +Y left, +Z up; +yaw is CCW from above.
 * Body origin = chassis center; arm targets use the arm-base origin.
 * World axes coincide with body axes at yaw=0; see COORDINATE_SYSTEM.md.
 * Raw IMU/debug values retain their explicitly named sensor frame.
 */
#define ROBOT_STATUS_PROTOCOL_VERSION 4U
#define ROBOT_FRAME_DEG2RAD 0.01745329251994329577f
#define ROBOT_FRAME_RAD2DEG 57.2957795130823208768f

/* Installed IMU sensor axes: X=right, Y=front, Z=up.
 * Rotate vector components into the application frame (including gyro).
 */
static inline void RobotFrame_ImuVectorToRobot(float sx, float sy, float sz,
                                               float *rx, float *ry, float *rz)
{
    float result_x = sy;
    float result_y = -sx;
    float result_z = sz;
    *rx = result_x;
    *ry = result_y;
    *rz = result_z;
}

/* Convert ZYX Euler attitude by changing BOTH body and reference-world bases:
 * R_robot = T * R_sensor * transpose(T), T = Rz(-90deg).
 * At zero tilt this preserves the existing yaw datum (no added 90deg).
 * Swapping Euler roll/pitch alone is not exact for compound tilts.
 */
static inline void RobotFrame_ImuEulerToRobot(float roll_deg, float pitch_deg,
                                              float yaw_deg, float *robot_roll,
                                              float *robot_pitch, float *robot_yaw)
{
    float r = roll_deg * ROBOT_FRAME_DEG2RAD;
    float p = pitch_deg * ROBOT_FRAME_DEG2RAD;
    float y = yaw_deg * ROBOT_FRAME_DEG2RAD;
    float sr = sinf(r), cr = cosf(r), sp = sinf(p), cp = cosf(p);
    float sy = sinf(y), cy = cosf(y);
    float sin_pitch = -cp * sr;
    if (sin_pitch > 1.0f) sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    *robot_pitch = asinf(sin_pitch) * ROBOT_FRAME_RAD2DEG;
    if (fabsf(sin_pitch) > 0.999999f) {
        /* Euler singularity: choose roll=0 and preserve the rotation matrix. */
        *robot_roll = 0.0f;
        *robot_yaw = atan2f(sy * cp, cy * cp) * ROBOT_FRAME_RAD2DEG;
    } else {
        *robot_roll = atan2f(sp, cp * cr) * ROBOT_FRAME_RAD2DEG;
        *robot_yaw = atan2f(sy * cr - cy * sp * sr,
                            sy * sp * sr + cy * cr) * ROBOT_FRAME_RAD2DEG;
    }
}

#endif /* ROBOT_FRAME_H */
