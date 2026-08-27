from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from pathlib import Path


def generate_launch_description():
    package_share = Path(get_package_share_directory("offset_caster_mujoco_control"))
    parameters = package_share / "config" / "inverse_kinematics.yaml"

    return LaunchDescription(
        [
            Node(
                package="offset_caster_mujoco_control",
                executable="inverse_kinematics_node",
                name="inverse_kinematics_node",
                output="screen",
                parameters=[str(parameters)],
            )
        ]
    )
