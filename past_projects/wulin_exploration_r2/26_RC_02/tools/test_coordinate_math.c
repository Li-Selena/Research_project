#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "ins_nav_math.h"
#include "mecanum_classic.h"
#include "robot_frame.h"

static void expect_near(const char *name, float actual, float expected, float tol)
{
    if (fabsf(actual - expected) > tol) {
        fprintf(stderr, "%s: got %.7f, expected %.7f\n", name, actual, expected);
        exit(1);
    }
}

static void check_mecanum(float vx, float vy, float vw)
{
    ChassisVel_t in = {vx, vy, vw};
    ChassisVel_t out;
    WheelSpeed_t wheel;
    MecanumParam_t p = {MEC_R, 0.338f, 0.375f, 100.0f};
    float k_inv = 0.25f / (p.L + p.W);

    Mecanum_Calc(&in, &p, &wheel);
    out.vx = (wheel.fl + wheel.fr + wheel.bl + wheel.br) * 0.25f;
    out.vy = (-wheel.fl + wheel.fr + wheel.bl - wheel.br) * 0.25f;
    out.vw = (-wheel.fl + wheel.fr - wheel.bl + wheel.br) * k_inv;

    expect_near("mecanum vx", out.vx, vx, 1.0e-6f);
    expect_near("mecanum vy", out.vy, vy, 1.0e-6f);
    expect_near("mecanum vw", out.vw, vw, 1.0e-6f);
}

static void euler_zyx_matrix(float roll_deg, float pitch_deg, float yaw_deg,
                             float m[3][3])
{
    float d2r = 3.14159265358979323846f / 180.0f;
    float r = roll_deg * d2r;
    float p = pitch_deg * d2r;
    float y = yaw_deg * d2r;
    float sr = sinf(r), cr = cosf(r);
    float sp = sinf(p), cp = cosf(p);
    float sy = sinf(y), cy = cosf(y);

    m[0][0] = cy * cp;
    m[0][1] = cy * sp * sr - sy * cr;
    m[0][2] = cy * sp * cr + sy * sr;
    m[1][0] = sy * cp;
    m[1][1] = sy * sp * sr + cy * cr;
    m[1][2] = sy * sp * cr - cy * sr;
    m[2][0] = -sp;
    m[2][1] = cp * sr;
    m[2][2] = cp * cr;
}

static void check_imu_euler_basis(float sensor_roll, float sensor_pitch,
                                  float sensor_yaw)
{
    const float t[3][3] = {{0.0f, 1.0f, 0.0f},
                           {-1.0f, 0.0f, 0.0f},
                           {0.0f, 0.0f, 1.0f}};
    float sensor[3][3];
    float temp[3][3] = {{0}};
    float expected[3][3] = {{0}};
    float actual[3][3];
    float roll, pitch, yaw;
    int i, j, k;

    euler_zyx_matrix(sensor_roll, sensor_pitch, sensor_yaw, sensor);
    RobotFrame_ImuEulerToRobot(sensor_roll, sensor_pitch, sensor_yaw,
                               &roll, &pitch, &yaw);
    euler_zyx_matrix(roll, pitch, yaw, actual);

    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            for (k = 0; k < 3; ++k) {
                temp[i][j] += t[i][k] * sensor[k][j];
            }
        }
    }
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            for (k = 0; k < 3; ++k) {
                expected[i][j] += temp[i][k] * t[j][k];
            }
            expect_near("IMU Euler basis matrix", actual[i][j], expected[i][j],
                        2.0e-5f);
        }
    }
}

int main(void)
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;

    check_mecanum(0.6f, 0.0f, 0.0f);
    check_mecanum(0.0f, 0.6f, 0.0f);
    check_mecanum(0.0f, 0.0f, 0.5f);
    check_mecanum(0.4f, -0.3f, 0.2f);

    INS_NavMath_RobotToWorld(1.0f, 0.0f, 0.5f * 3.14159265358979323846f, &x, &y);
    expect_near("body forward -> world left", x, 0.0f, 1.0e-6f);
    expect_near("body forward -> world left", y, 1.0f, 1.0e-6f);

    /* Verify that input and output pointers may alias. */
    INS_NavMath_WorldToRobot(x, y, 0.5f * 3.14159265358979323846f, &x, &y);
    expect_near("world/body inverse x", x, 1.0f, 1.0e-6f);
    expect_near("world/body inverse y", y, 0.0f, 1.0e-6f);

    RobotFrame_ImuVectorToRobot(2.0f, 3.0f, 4.0f, &x, &y, &z);
    expect_near("imu x", x, 3.0f, 1.0e-6f);
    expect_near("imu y", y, -2.0f, 1.0e-6f);
    expect_near("imu z", z, 4.0f, 1.0e-6f);

    RobotFrame_ImuEulerToRobot(10.0f, 0.0f, 20.0f, &roll, &pitch, &yaw);
    expect_near("sensor roll -> robot pitch", roll, 0.0f, 1.0e-4f);
    expect_near("sensor roll -> robot pitch", pitch, -10.0f, 1.0e-4f);
    expect_near("imu yaw datum", yaw, 20.0f, 1.0e-4f);

    RobotFrame_ImuEulerToRobot(0.0f, 10.0f, -20.0f, &roll, &pitch, &yaw);
    expect_near("sensor pitch -> robot roll", roll, 10.0f, 1.0e-4f);
    expect_near("sensor pitch -> robot roll", pitch, 0.0f, 1.0e-4f);
    expect_near("imu yaw datum", yaw, -20.0f, 1.0e-4f);

    check_imu_euler_basis(25.0f, -17.0f, 42.0f);
    check_imu_euler_basis(-37.0f, 22.0f, -118.0f);

    puts("coordinate math verification passed");
    return 0;
}
