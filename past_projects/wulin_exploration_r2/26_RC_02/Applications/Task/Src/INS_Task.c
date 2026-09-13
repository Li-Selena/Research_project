#include "INS_Task.h"
#include "cmsis_os.h"
#include "ins_nav_math.h"
#include "robot_frame.h"
#include <string.h>

#define INS_TASK_PERIOD_MS          1U
#define INS_IMU_OFFLINE_TIMEOUT_MS  100U

/* Installed sensor frame: X right, Y front, Z up.
 * RobotFrame_* converts to X front, Y left, Z up exactly once here.
 * Both world and body XY bases rotate; the level yaw datum is unchanged.
 */
INS_Info_Typedef INS_Info = {0};
INS_NavState_t g_ins_nav_state = {0};
INS_OdometryState_t g_ins_odom_state = {0};
INS_DebugState_t g_ins_debug_state = {0};
static uint8_t ins_yaw_total_initialized = 0U;

static uint8_t INS_TakeImuSnapshot(IMU_Data_t *out);
static uint8_t INS_CheckImuOnline(void);
static void INS_SetImuOnline(uint8_t online);
static void INS_UpdateDebugState(uint8_t has_new_imu);

void INS_Task(void const *argument)
{
    IMU_Data_t imu_snapshot;

    (void)argument;

    IMU_Init();

    for (;;)
    {
        g_ins_debug_state.task_loop_count++;

        if (INS_TakeImuSnapshot(&imu_snapshot) != 0U)
        {
            g_ins_debug_state.imu_snapshot_count++;
            INS_Update_From_IMU(&imu_snapshot);
            INS_Update_Yaw_TotalAngle();
            INS_Update_NavState();
        }
        else
        {
            g_ins_debug_state.imu_miss_count++;
            INS_SetImuOnline(INS_CheckImuOnline());
        }

        osDelay(INS_TASK_PERIOD_MS);
    }
}

void INS_Update_From_IMU(const IMU_Data_t *imu)
{
    if (imu == NULL)
    {
        return;
    }

    RobotFrame_ImuEulerToRobot(imu->roll, imu->pitch, imu->yaw,
                               &INS_Info.Roll_Angle, &INS_Info.Pitch_Angle,
                               &INS_Info.Yaw_Angle);
    RobotFrame_ImuVectorToRobot(imu->gyro_x, imu->gyro_y, imu->gyro_z,
                                &INS_Info.Roll_Gyro, &INS_Info.Pitch_Gyro,
                                &INS_Info.Yaw_Gyro);
    RobotFrame_ImuVectorToRobot(imu->acc_x, imu->acc_y, imu->acc_z,
                                &INS_Info.Accel[0], &INS_Info.Accel[1],
                                &INS_Info.Accel[2]);
    INS_Info.Gyro[0] = INS_Info.Roll_Gyro;
    INS_Info.Gyro[1] = INS_Info.Pitch_Gyro;
    INS_Info.Gyro[2] = INS_Info.Yaw_Gyro;

    /* Preserve the public Angle array order: yaw, roll, pitch (degrees). */
    INS_Info.Angle[0] = INS_Info.Yaw_Angle;
    INS_Info.Angle[1] = INS_Info.Roll_Angle;
    INS_Info.Angle[2] = INS_Info.Pitch_Angle;
}

void INS_Update_Yaw_TotalAngle(void)
{
    if (ins_yaw_total_initialized == 0U)
    {
        INS_Info.Last_Yaw_Angle = INS_Info.Yaw_Angle;
        INS_Info.YawRoundCount = 0;
        INS_Info.Yaw_TolAngle = INS_Info.Yaw_Angle;
        ins_yaw_total_initialized = 1U;
        return;
    }

    INS_Info.Yaw_TolAngle = INS_NavMath_UpdateContinuousYawDeg(
        INS_Info.Yaw_Angle,
        &INS_Info.Last_Yaw_Angle,
        &INS_Info.YawRoundCount);
}

void INS_Update_NavState(void)
{
    INS_NavState_t next_state;
    INS_OdometryState_t odom_snapshot;
    uint32_t primask;
    float chassis_yaw_deg;
    float chassis_yaw_total_deg;

    primask = __get_PRIMASK();
    __disable_irq();
    odom_snapshot = g_ins_odom_state;
    if (primask == 0U)
    {
        __enable_irq();
    }

    memset(&next_state, 0, sizeof(next_state));

    chassis_yaw_deg = INS_NavMath_WrapDeg(INS_Info.Yaw_Angle);
    chassis_yaw_total_deg = INS_Info.Yaw_TolAngle;

    next_state.x_m = odom_snapshot.x_m;
    next_state.y_m = odom_snapshot.y_m;
    next_state.vx_mps = odom_snapshot.vx_mps;
    next_state.vy_mps = odom_snapshot.vy_mps;
    next_state.wz_radps = odom_snapshot.wz_radps;

    next_state.roll_deg = INS_Info.Roll_Angle;
    next_state.pitch_deg = INS_Info.Pitch_Angle;
    next_state.yaw_deg = chassis_yaw_deg;
    next_state.yaw_total_deg = chassis_yaw_total_deg;

    next_state.yaw_rad = chassis_yaw_deg * INS_NAV_DEG2RAD_F;
    next_state.yaw_total_rad = chassis_yaw_total_deg * INS_NAV_DEG2RAD_F;

    next_state.gyro_x_dps = INS_Info.Roll_Gyro;
    next_state.gyro_y_dps = INS_Info.Pitch_Gyro;
    next_state.gyro_z_dps = INS_Info.Yaw_Gyro;
    next_state.acc_x_g = INS_Info.Accel[0];
    next_state.acc_y_g = INS_Info.Accel[1];
    next_state.acc_z_g = INS_Info.Accel[2];

    next_state.imu_online = INS_CheckImuOnline();
    next_state.imu_update_tick = imu_data.last_update_tick;
    next_state.odom_update_tick = odom_snapshot.update_tick;
    next_state.update_tick = HAL_GetTick();

    primask = __get_PRIMASK();
    __disable_irq();
    g_ins_nav_state = next_state;
    imu_data.online = next_state.imu_online;
    if (primask == 0U)
    {
        __enable_irq();
    }

    INS_UpdateDebugState(1U);
}

void INS_GetState(INS_NavState_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out = g_ins_nav_state;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void INS_SetOdometry(float x_m,
                     float y_m,
                     float vx_mps,
                     float vy_mps,
                     float wz_radps)
{
    uint32_t primask;
    uint32_t now_tick;

    now_tick = HAL_GetTick();

    primask = __get_PRIMASK();
    __disable_irq();
    g_ins_odom_state.x_m = x_m;
    g_ins_odom_state.y_m = y_m;
    g_ins_odom_state.vx_mps = vx_mps;
    g_ins_odom_state.vy_mps = vy_mps;
    g_ins_odom_state.wz_radps = wz_radps;
    g_ins_odom_state.update_tick = now_tick;

    g_ins_nav_state.x_m = x_m;
    g_ins_nav_state.y_m = y_m;
    g_ins_nav_state.vx_mps = vx_mps;
    g_ins_nav_state.vy_mps = vy_mps;
    g_ins_nav_state.wz_radps = wz_radps;
    g_ins_nav_state.odom_update_tick = now_tick;
    g_ins_nav_state.update_tick = now_tick;
    if (primask == 0U)
    {
        __enable_irq();
    }

    INS_UpdateDebugState(0U);
}

static uint8_t INS_TakeImuSnapshot(IMU_Data_t *out)
{
    uint8_t has_update;
    uint32_t primask;

    if (out == NULL)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    has_update = imu_data.update_flag;
    if (has_update != 0U)
    {
        *out = imu_data;
        imu_data.update_flag = 0U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    return has_update;
}

static uint8_t INS_CheckImuOnline(void)
{
    uint32_t last_tick;
    uint32_t now_tick;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    last_tick = imu_data.last_update_tick;
    if (primask == 0U)
    {
        __enable_irq();
    }

    now_tick = HAL_GetTick();
    if (last_tick == 0U)
    {
        return 0U;
    }

    return ((now_tick - last_tick) <= INS_IMU_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}

static void INS_SetImuOnline(uint8_t online)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    imu_data.online = online;
    g_ins_nav_state.imu_online = online;
    g_ins_nav_state.update_tick = HAL_GetTick();
    if (primask == 0U)
    {
        __enable_irq();
    }

    INS_UpdateDebugState(0U);
}

static void INS_UpdateDebugState(uint8_t has_new_imu)
{
    INS_DebugState_t next_debug;
    INS_NavState_t nav_snapshot;
    INS_OdometryState_t odom_snapshot;
    IMU_Data_t imu_snapshot;
    uint32_t primask;
    uint32_t now_tick;

    now_tick = HAL_GetTick();

    primask = __get_PRIMASK();
    __disable_irq();
    next_debug = g_ins_debug_state;
    nav_snapshot = g_ins_nav_state;
    odom_snapshot = g_ins_odom_state;
    imu_snapshot = imu_data;
    if (primask == 0U)
    {
        __enable_irq();
    }

    next_debug.tick_ms = now_tick;
    next_debug.imu_online = nav_snapshot.imu_online;
    next_debug.imu_data_online = imu_snapshot.online;
    next_debug.imu_update_flag = imu_snapshot.update_flag;
    next_debug.has_new_imu = has_new_imu;
    next_debug.yaw_total_initialized = ins_yaw_total_initialized;

    next_debug.imu_last_update_tick = imu_snapshot.last_update_tick;
    next_debug.imu_age_ms = (imu_snapshot.last_update_tick != 0U) ?
        (now_tick - imu_snapshot.last_update_tick) : 0xFFFFFFFFU;
    next_debug.nav_update_tick = nav_snapshot.update_tick;
    next_debug.imu_update_tick = nav_snapshot.imu_update_tick;
    next_debug.odom_update_tick = nav_snapshot.odom_update_tick;
    next_debug.odom_age_ms = (odom_snapshot.update_tick != 0U) ?
        (now_tick - odom_snapshot.update_tick) : 0xFFFFFFFFU;

    next_debug.imu_acc_x_g = imu_snapshot.acc_x;
    next_debug.imu_acc_y_g = imu_snapshot.acc_y;
    next_debug.imu_acc_z_g = imu_snapshot.acc_z;
    next_debug.imu_gyro_x_dps = imu_snapshot.gyro_x;
    next_debug.imu_gyro_y_dps = imu_snapshot.gyro_y;
    next_debug.imu_gyro_z_dps = imu_snapshot.gyro_z;
    next_debug.imu_roll_deg = imu_snapshot.roll;
    next_debug.imu_pitch_deg = imu_snapshot.pitch;
    next_debug.imu_yaw_deg = imu_snapshot.yaw;

    next_debug.nav_x_m = nav_snapshot.x_m;
    next_debug.nav_y_m = nav_snapshot.y_m;
    next_debug.nav_yaw_rad = nav_snapshot.yaw_rad;
    next_debug.nav_yaw_total_rad = nav_snapshot.yaw_total_rad;
    next_debug.nav_vx_mps = nav_snapshot.vx_mps;
    next_debug.nav_vy_mps = nav_snapshot.vy_mps;
    next_debug.nav_wz_radps = nav_snapshot.wz_radps;
    next_debug.nav_roll_deg = nav_snapshot.roll_deg;
    next_debug.nav_pitch_deg = nav_snapshot.pitch_deg;
    next_debug.nav_yaw_deg = nav_snapshot.yaw_deg;
    next_debug.nav_yaw_total_deg = nav_snapshot.yaw_total_deg;
    next_debug.nav_gyro_x_dps = nav_snapshot.gyro_x_dps;
    next_debug.nav_gyro_y_dps = nav_snapshot.gyro_y_dps;
    next_debug.nav_gyro_z_dps = nav_snapshot.gyro_z_dps;
    next_debug.nav_acc_x_g = nav_snapshot.acc_x_g;
    next_debug.nav_acc_y_g = nav_snapshot.acc_y_g;
    next_debug.nav_acc_z_g = nav_snapshot.acc_z_g;

    next_debug.odom_x_m = odom_snapshot.x_m;
    next_debug.odom_y_m = odom_snapshot.y_m;
    next_debug.odom_vx_mps = odom_snapshot.vx_mps;
    next_debug.odom_vy_mps = odom_snapshot.vy_mps;
    next_debug.odom_wz_radps = odom_snapshot.wz_radps;

    next_debug.ins_last_yaw_deg = INS_Info.Last_Yaw_Angle;
    next_debug.yaw_round_count = INS_Info.YawRoundCount;

    primask = __get_PRIMASK();
    __disable_irq();
    g_ins_debug_state = next_debug;
    if (primask == 0U)
    {
        __enable_irq();
    }
}
