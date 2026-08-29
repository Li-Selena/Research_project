import math

from offset_caster_interfaces.msg import MotionCommand


MODE_NAMES = {
    MotionCommand.WORLD_VELOCITY_FIXED_YAW: "世界速度 · 固定 Yaw",
    MotionCommand.WORLD_VELOCITY_DYNAMIC_YAW: "世界速度 · 动态 Yaw",
    MotionCommand.BODY_VELOCITY_FIXED_YAW: "机器人速度 · 固定 Yaw",
    MotionCommand.BODY_VELOCITY_DYNAMIC_YAW: "机器人速度 · 动态 Yaw",
    MotionCommand.WORLD_POSITION_FIXED_YAW: "世界位置 · 固定 Yaw",
    MotionCommand.WORLD_POSITION_DYNAMIC_YAW: "世界位置 · 动态 Yaw",
}

POSITION_MODE_NAMES = {
    MotionCommand.WORLD_POSITION_FIXED_YAW: MODE_NAMES[
        MotionCommand.WORLD_POSITION_FIXED_YAW
    ],
    MotionCommand.WORLD_POSITION_DYNAMIC_YAW: MODE_NAMES[
        MotionCommand.WORLD_POSITION_DYNAMIC_YAW
    ],
}


def field_configuration(mode):
    position = mode in (
        MotionCommand.WORLD_POSITION_FIXED_YAW,
        MotionCommand.WORLD_POSITION_DYNAMIC_YAW,
    )
    dynamic_velocity = mode in (
        MotionCommand.WORLD_VELOCITY_DYNAMIC_YAW,
        MotionCommand.BODY_VELOCITY_DYNAMIC_YAW,
    )
    return {
        "x_label": "目标 X (m)" if position else "Vx (m/s)",
        "y_label": "目标 Y (m)" if position else "Vy (m/s)",
        "yaw_enabled": not dynamic_velocity,
        "yaw_rate_enabled": dynamic_velocity,
    }


def quaternion_to_rpy(quaternion):
    sinr_cosp = 2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
    cosr_cosp = 1.0 - 2.0 * (quaternion.x**2 + quaternion.y**2)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )
    return roll, pitch, yaw
