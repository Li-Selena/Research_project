from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("offset_caster_mujoco_control"))
    inverse_parameters = package_share / "config" / "inverse_kinematics.yaml"
    mujoco_parameters = package_share / "config" / "mujoco_state.yaml"
    motion_parameters = package_share / "config" / "motion_controller.yaml"
    enable_viewer = LaunchConfiguration("enable_viewer")
    enable_dashboard = LaunchConfiguration("enable_dashboard")
    enable_monitor = LaunchConfiguration("enable_monitor")
    model_path = LaunchConfiguration("model_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_viewer",
                default_value="true",
                description="Open the native MuJoCo GLFW viewer",
            ),
            DeclareLaunchArgument(
                "enable_dashboard",
                default_value="true",
                description="Open the standalone ROS Qt position dashboard",
            ),
            DeclareLaunchArgument(
                "enable_monitor",
                default_value="true",
                description="Open the independent ROS Qt curve monitor",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value="/workspace/3Dmodel/offestWheel/MJCF/offset_steering_wheel.xml",
                description="Absolute path to the offset-caster MJCF model",
            ),
            Node(
                package="offset_caster_mujoco_control",
                executable="mujoco_state_node",
                name="mujoco_state_node",
                output="screen",
                parameters=[
                    str(mujoco_parameters),
                    {
                        "enable_viewer": ParameterValue(enable_viewer, value_type=bool),
                        "model_path": ParameterValue(model_path, value_type=str),
                    },
                ],
            ),
            Node(
                package="offset_caster_mujoco_control",
                executable="inverse_kinematics_node",
                name="inverse_kinematics_node",
                output="screen",
                parameters=[str(inverse_parameters)],
            ),
            Node(
                package="offset_caster_mujoco_control",
                executable="motion_controller_node",
                name="motion_controller_node",
                output="screen",
                parameters=[str(motion_parameters)],
            ),
            Node(
                package="offset_caster_dashboard",
                executable="offset_caster_dashboard",
                name="offset_caster_dashboard",
                output="screen",
                condition=IfCondition(enable_dashboard),
            ),
            Node(
                package="offset_caster_dashboard",
                executable="offset_caster_monitor",
                name="offset_caster_curve_monitor",
                output="screen",
                condition=IfCondition(enable_monitor),
            ),
        ]
    )
