#include "include.h"

float motors_rnd[3];
displacement_PID my_displacement_pid;
float displace_buffer[3];
static float previous_wheel_mm[3];
static uint8_t odometry_initialized;

static float clamp_output(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void get_rnd_count_and_diaplacement(void)
{
    float wheel_mm[3];
    float wheel_delta[3];
    float local_delta[3];
    motor_measure_t feedback;

    for (uint8_t i = 0; i < 3; ++i) {
        motor_feedback_snapshot(&motor_can1[i], &feedback);
        motors_rnd[i] = (float)feedback.total_angle;
        wheel_mm[i] = motors_rnd[i] * (2.0f * CONTROL_PI / 8192.0f) *
                      (1.0f / CHASSIS_GEAR_RATIO) * CHASSIS_WHEEL_RADIUS_MM;
        wheel_delta[i] = wheel_mm[i] - previous_wheel_mm[i];
        previous_wheel_mm[i] = wheel_mm[i];
    }
    if (!odometry_initialized) {
        odometry_initialized = 1;
        memset(wheel_delta, 0, sizeof(wheel_delta));
    }
    matrix_multiply(solution_matrix, wheel_delta, local_delta);
    float yaw = my_angle_measure.yaw * CONTROL_PI / 180.0f;
    displace_buffer[0] += cosf(yaw) * local_delta[0] - sinf(yaw) * local_delta[1];
    displace_buffer[1] += sinf(yaw) * local_delta[0] + cosf(yaw) * local_delta[1];
    displace_buffer[2] = my_angle_measure.yaw;
}

void displacement_pid_clear(void)
{
    memset(&my_displacement_pid.x_error, 0, sizeof(my_displacement_pid.x_error));
    memset(&my_displacement_pid.y_error, 0, sizeof(my_displacement_pid.y_error));
    memset(&my_displacement_pid.omega_error, 0, sizeof(my_displacement_pid.omega_error));
    my_displacement_pid.s_x_error = 0;
    my_displacement_pid.s_y_error = 0;
    my_displacement_pid.s_omega_error = 0;
    memset(my_displacement_pid.out, 0, sizeof(my_displacement_pid.out));
}

void rnd_count_and_diaplacement_reset(void)
{
    memset(motors_rnd, 0, sizeof(motors_rnd));
    memset(previous_wheel_mm, 0, sizeof(previous_wheel_mm));
    memset(displace_buffer, 0, sizeof(displace_buffer));
    odometry_initialized = 0;
    displacement_pid_clear();
}

float angle_error(float target, float current)
{
    float diff = fmodf(target - current + 180.0f, 360.0f);
    if (diff < 0) diff += 360.0f;
    return diff - 180.0f;
}

void xy_displacement_pid_config(float kp,float kd,float ki,float max_iout,float max_out)
{
    my_displacement_pid.xy_kp=kp; my_displacement_pid.xy_kd=kd;
    my_displacement_pid.xy_ki=ki; my_displacement_pid.xy_max_iout=max_iout;
    my_displacement_pid.xy_max_out=max_out;
}
void z_displacement_pid_config(float kp,float kd,float ki,float max_iout,float max_out)
{
    my_displacement_pid.z_kp=kp; my_displacement_pid.z_kd=kd;
    my_displacement_pid.z_ki=ki; my_displacement_pid.z_max_iout=max_iout;
    my_displacement_pid.z_max_out=max_out;
}

static float axis_pid(float target, float measured, float error[2], float *sum,
                      float kp, float ki, float kd, float i_limit, float out_limit)
{
    float iout;
    error[1] = error[0];
    error[0] = target - measured;
    *sum += error[0] * CONTROL_DT_S;
    iout = clamp_output(*sum * ki, i_limit);
    return clamp_output(kp * error[0] + iout +
                        kd * (error[0] - error[1]) / CONTROL_DT_S, out_limit);
}

void x_displacement_control(float target)
{
    my_displacement_pid.out[0] = axis_pid(target, displace_buffer[0],
        my_displacement_pid.x_error, &my_displacement_pid.s_x_error,
        my_displacement_pid.xy_kp, my_displacement_pid.xy_ki,
        my_displacement_pid.xy_kd, my_displacement_pid.xy_max_iout,
        my_displacement_pid.xy_max_out);
}
void y_displacement_control(float target)
{
    my_displacement_pid.out[1] = axis_pid(target, displace_buffer[1],
        my_displacement_pid.y_error, &my_displacement_pid.s_y_error,
        my_displacement_pid.xy_kp, my_displacement_pid.xy_ki,
        my_displacement_pid.xy_kd, my_displacement_pid.xy_max_iout,
        my_displacement_pid.xy_max_out);
}
void theta_displacement_control(float target)
{
    my_displacement_pid.omega_error[1] = my_displacement_pid.omega_error[0];
    my_displacement_pid.omega_error[0] = angle_error(target, displace_buffer[2]);
    my_displacement_pid.s_omega_error += my_displacement_pid.omega_error[0] * CONTROL_DT_S;
    float iout = clamp_output(my_displacement_pid.s_omega_error * my_displacement_pid.z_ki,
                              my_displacement_pid.z_max_iout);
    my_displacement_pid.out[2] = clamp_output(
        my_displacement_pid.z_kp * my_displacement_pid.omega_error[0] + iout +
        my_displacement_pid.z_kd * (my_displacement_pid.omega_error[0] -
        my_displacement_pid.omega_error[1]) / CONTROL_DT_S,
        my_displacement_pid.z_max_out);
}
