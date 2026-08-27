from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("offset_caster_mujoco_control"))
    inverse_parameters = package_share / "config" / "inverse_kinematics.yaml"
    mujoco_parameters = package_share / "config" / "mujoco_state.yaml"
    enable_viewer = LaunchConfiguration("enable_viewer")
    model_path = LaunchConfiguration("model_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_viewer",
                default_value="true",
                description="Open the native MuJoCo GLFW viewer",
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
        ]
    )
