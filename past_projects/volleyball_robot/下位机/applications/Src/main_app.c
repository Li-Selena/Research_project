#include "include.h"
#include "can.h"
#include "tim.h"
#include "usart.h"

static uint8_t position_target_valid;

static void read_host_target(float target[3])
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    memcpy(target, receive_data, 3U * sizeof(float));
    __set_PRIMASK(irq);
}

void chassis_stop(void)
{
    CAN1_SetMotorCurrent(1, 0);
    CAN1_SetMotorCurrent(2, 0);
    CAN1_SetMotorCurrent(3, 0);
    chassis_pid_clear();
}

void user_init(void)
{
    HAL_Delay(3000);
    PID_devices_Init();
    CAN1_Filter_Init();
    CAN2_Filter_Init();
    CAN_Start(&hcan1);
    CAN_Start(&hcan2);
    uart_receivers_start();
    DWT_Init();
    delay_init();
    inverse_solution_matrix_init();
    solution_matrix_init();
    xy_displacement_pid_config(1, 0, 0, 150, 2000);
    z_displacement_pid_config(2, 0, 0, 184, 3000);
    vel_designer_init(body_acc_design, 1500);
    xy_spatium_designer_init(xyw_spatium_designer, 4800, 1200);
    spatium_designer_init(&xyw_spatium_designer[2], 2000, 500);
    rnd_count_and_diaplacement_reset();
    delta_init();
    robot_control_init();
    HAL_TIM_Base_Start_IT(&htim2);
}

void control_mode_changed(void)
{
    chassis_pid_clear();
    displacement_pid_clear();
    motion_profiles_reset();
    position_target_valid = 0;
    change_mode_flag = 0;
}

void local_velocity_mode_run(void)
{
    float target[3];
    if (!host_input_fresh() || robot_chassis_locked()) {
        chassis_stop();
        return;
    }
    read_host_target(target);
    calc_buffer[0] = target[0] * CHASSIS_RPM_PER_MM_S;
    calc_buffer[1] = target[1] * CHASSIS_RPM_PER_MM_S;
    calc_buffer[2] = target[2] * CONTROL_PI / 180.0f * CHASSIS_RPM_PER_RAD_S;
    feedforword_control(body_acc_design, calc_buffer);
    vel_control(calc_buffer);
}

void world_displacement_mode_run(void)
{
    static float last_target[3];
    static uint32_t settled_since;
    float target[3];
    float world_velocity[3];
    uint32_t now = HAL_GetTick();

    if (!host_input_fresh() || !imu_input_fresh() || robot_chassis_locked()) {
        settled_since = 0;
        chassis_stop();
        return;
    }
    read_host_target(target);
    if (!position_target_valid || fabsf(last_target[0] - target[0]) > 1.0f ||
        fabsf(last_target[1] - target[1]) > 1.0f ||
        fabsf(angle_error(last_target[2], target[2])) > 1.0f) {
        spatium_designer_set_target(&xyw_spatium_designer[0],
            (target[0] - displace_buffer[0]) * CHASSIS_RPM_PER_MM_S);
        spatium_designer_set_target(&xyw_spatium_designer[1],
            (target[1] - displace_buffer[1]) * CHASSIS_RPM_PER_MM_S);
        spatium_designer_set_target(&xyw_spatium_designer[2],
            angle_error(target[2], displace_buffer[2]) * CONTROL_PI / 180.0f * CHASSIS_RPM_PER_RAD_S);
        memcpy(last_target, target, sizeof(last_target));
        position_target_valid = 1;
        settled_since = 0;
    }

    spatium_designer_update(&xyw_spatium_designer[0]);
    spatium_designer_update(&xyw_spatium_designer[1]);
    spatium_designer_update(&xyw_spatium_designer[2]);
    if (xyw_spatium_designer[0].finished) {
        x_displacement_control(target[0]);
        world_velocity[0] = my_displacement_pid.out[0];
    } else world_velocity[0] = xyw_spatium_designer[0].v_out;
    if (xyw_spatium_designer[1].finished) {
        y_displacement_control(target[1]);
        world_velocity[1] = my_displacement_pid.out[1];
    } else world_velocity[1] = xyw_spatium_designer[1].v_out;
    if (xyw_spatium_designer[2].finished) {
        theta_displacement_control(target[2]);
        world_velocity[2] = my_displacement_pid.out[2];
    } else world_velocity[2] = xyw_spatium_designer[2].v_out;

    rotate_matrix_calc(-my_angle_measure.yaw * CONTROL_PI / 180.0f);
    matrix_multiply(rotate_matrix, world_velocity, calc_buffer_2nd);
    feedforword_control(body_acc_design, calc_buffer_2nd);
    vel_control(calc_buffer_2nd);

    if (xyw_spatium_designer[0].finished && xyw_spatium_designer[1].finished &&
        xyw_spatium_designer[2].finished &&
        fabsf(target[0] - displace_buffer[0]) <= CHASSIS_POSITION_TOL_MM &&
        fabsf(target[1] - displace_buffer[1]) <= CHASSIS_POSITION_TOL_MM &&
        fabsf(angle_error(target[2], displace_buffer[2])) <= CHASSIS_HEADING_TOL_DEG) {
        if (settled_since == 0U) settled_since = now;
        if ((uint32_t)(now - settled_since) >= CHASSIS_SETTLE_MS) {
            run_mode = CHASSIS_STOP;
            change_mode_flag = 1;
        }
    } else settled_since = 0;
}

void msg_control(void) { uart_service_tx(); }
