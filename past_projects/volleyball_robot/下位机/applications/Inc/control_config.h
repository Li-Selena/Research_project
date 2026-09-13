#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* All geometry and commissioning constants live here. */
#define CONTROL_DT_S 0.001f
#define CONTROL_PERIOD_MS 1U
#define HOST_TIMEOUT_MS 500U
#define MOTOR_TIMEOUT_MS 100U
#define IMU_TIMEOUT_MS 100U
#define UART_FRAME_TIMEOUT_MS 50U
#define MODBUS_SLAVE_ADDRESS 1U
#define MODBUS_FRAME_TIMEOUT_MS 10U
#define MODBUS_HEARTBEAT_PERIOD_MS 100U
#define CONTROL_MAX_GAP_MS 10U
#define CONTROL_PI 3.14159265358979323846f

#define CHASSIS_WHEEL_RADIUS_MM 76.0f
#define CHASSIS_GEAR_RATIO (3591.0f / 187.0f)
#define CHASSIS_RPM_PER_MM_S (60.0f * CHASSIS_GEAR_RATIO / (2.0f * CONTROL_PI * CHASSIS_WHEEL_RADIUS_MM))
/* Existing tuning: 184 equivalent motor rpm per rad/s of body rotation. */
#define CHASSIS_RPM_PER_RAD_S 184.0f
#define CHASSIS_MAX_RPM 8500.0f
#define CHASSIS_CURRENT_LIMIT 10000.0f
#define CHASSIS_POSITION_TOL_MM 5.0f
#define CHASSIS_HEADING_TOL_DEG 2.0f
#define CHASSIS_SETTLE_MS 100U

/* Delta coordinates: metres, +Z above the base plane; angles: degrees, counterclockwise from horizontal.
 * IDs 4,5,6 correspond to azimuths 0,120,240 degrees, respectively.
 * Confirm these values and motor directions against the mechanism before enabling. */
#define DELTA_BASE_RADIUS_M 0.09857f
#define DELTA_PLATFORM_RADIUS_M 0.110f
#define DELTA_UPPER_ARM_M 0.192f
#define DELTA_LOWER_ARM_M 0.242f
#define DELTA_GEAR_RATIO (3591.0f / 187.0f)
#define DELTA_MOTOR_DIRECTIONS {1.0f, 1.0f, 1.0f}
/* Joint pose physically held when command 0x10 captures encoder zero. */
#define DELTA_CALIBRATION_ANGLES_DEG {0.0f, 0.0f, 0.0f}
#define DELTA_JOINT_MIN_DEG 0.0f
#define DELTA_JOINT_MAX_DEG 90.0f
#define DELTA_JOINT_SLEW_DEG_S 30.0f
#define DELTA_MAX_MOTOR_RPM 1200.0f
#define DELTA_POSITION_KP 20.0f /* motor rpm per degree of joint error */
#define DELTA_SPEED_KP 25.0f
#define DELTA_SPEED_KI 0.05f
#define DELTA_SPEED_KD 0.2f
#define DELTA_PID_OUTPUT_LIMIT 12000.0f
#define DELTA_PID_INTEGRAL_LIMIT 3000.0f
#define DELTA_CURRENT_LIMIT DELTA_PID_OUTPUT_LIMIT
#define DELTA_JOG_STEP_M 0.010f
#define DELTA_STATIONARY_RPM 50.0f
#define DELTA_HOME_X_M 0.0f
#define DELTA_HOME_Y_M 0.0f
#define DELTA_HOME_Z_M 0.16111634f
#define DELTA_DEFAULT_STRIKE_X_M 0.0f
#define DELTA_DEFAULT_STRIKE_Y_M 0.0f
#define DELTA_DEFAULT_STRIKE_Z_M 0.400f
#define DELTA_STRIKE_SLEW_DEG_S 720.0f
#define DELTA_RETURN_SLEW_DEG_S 360.0f
#define DELTA_STRIKE_MAX_MOTOR_RPM 3000.0f
#define DELTA_RETURN_MAX_MOTOR_RPM 2000.0f
#define DELTA_STRIKE_TOLERANCE_DEG 3.0f
#define DELTA_HOME_TOLERANCE_DEG 2.0f
#define DELTA_STRIKE_TIMEOUT_MS 300U

#define ROD_DIRECTION 1.0f
#define ROD_HOME_RAD 0.0f
#define ROD_PREPARE_RAD (ROD_DIRECTION * 2.617993878f)
#define ROD_STRIKE_RAD (ROD_DIRECTION * -0.698131701f)
#define ROD_POSITION_TOLERANCE_RAD 0.087266463f
#define ROD_STATIONARY_RAD_S 0.5f
#define ROD_ZERO_SETTLE_MS 50U
#define ROD_MOVE_TIMEOUT_MS 1500U
#define ROD_STRIKE_TIMEOUT_MS 300U
#define ROD_HOLD_KP 21.0f
#define ROD_HOLD_KD 2.5f
#define ROD_STRIKE_KP 300.0f
#define ROD_STRIKE_KD 4.4f
#define ROD_STRIKE_SPEED_RAD_S (ROD_DIRECTION * -29.0f)
#define ROD_STRIKE_TORQUE_NM (ROD_DIRECTION * -2.0f)

/* Existing fourth timing is reused for the previously missing fifth speed level. */
#define SHOT_HIGH_TRIM_RAD 0.05f
#endif
