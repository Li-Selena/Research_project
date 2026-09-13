"""Launch the complete vo2 detection, positioning and serial pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the vo2 launch description."""
    camera_id = LaunchConfiguration('camera_id')
    model_path = LaunchConfiguration('model_path')
    calibration_file = LaunchConfiguration('calibration_file')
    pnp_config_file = LaunchConfiguration('pnp_config_file')
    allow_example = LaunchConfiguration('allow_example_calibration')
    show_image = LaunchConfiguration('show_image')
    serial_enabled = LaunchConfiguration('serial_enabled')
    serial_port = LaunchConfiguration('serial_port')
    claim_usb_source = LaunchConfiguration('claim_usb_source')
    enable_arm = LaunchConfiguration('enable_arm_on_start')
    cmd_vel_enabled = LaunchConfiguration('cmd_vel_enabled')
    vision_control_enabled = LaunchConfiguration('vision_control_enabled')
    mission_enabled = LaunchConfiguration('mission_enabled')

    return LaunchDescription([
        DeclareLaunchArgument('camera_id', default_value='0'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('pnp_solve'),
                'config',
                'calibration.example.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'pnp_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('pnp_solve'),
                'config',
                'pnp.example.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'allow_example_calibration', default_value='false'
        ),
        DeclareLaunchArgument('show_image', default_value='true'),
        DeclareLaunchArgument('serial_enabled', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('claim_usb_source', default_value='false'),
        DeclareLaunchArgument('enable_arm_on_start', default_value='false'),
        DeclareLaunchArgument('cmd_vel_enabled', default_value='false'),
        DeclareLaunchArgument('vision_control_enabled', default_value='true'),
        DeclareLaunchArgument('mission_enabled', default_value='false'),
        Node(
            package='target_detection',
            executable='target_detection',
            name='yolo_detection_node',
            output='screen',
            parameters=[{
                'camera_id': ParameterValue(camera_id, value_type=int),
                'model_path': model_path,
                'show_image': ParameterValue(show_image, value_type=bool),
            }],
        ),
        Node(
            package='pnp_solve',
            executable='pnp_solve',
            name='pnp_solve_node',
            output='screen',
            parameters=[
                pnp_config_file,
                {
                    'calibration_file': calibration_file,
                    'allow_example_calibration': ParameterValue(
                        allow_example, value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package='robot_serial_bridge',
            executable='robot_serial_bridge',
            name='robot_serial_bridge_node',
            output='screen',
            parameters=[{
                'send_enabled': ParameterValue(serial_enabled, value_type=bool),
                'port': serial_port,
                'claim_usb_source': ParameterValue(
                    claim_usb_source, value_type=bool
                ),
                'enable_arm_on_start': ParameterValue(
                    enable_arm, value_type=bool
                ),
                'cmd_vel_enabled': ParameterValue(
                    cmd_vel_enabled, value_type=bool
                ),
                'vision_control_enabled': ParameterValue(
                    vision_control_enabled, value_type=bool
                ),
            }],
        ),
        Node(
            package='r2_mission',
            executable='mission_server',
            name='r2_mission_server',
            output='screen',
            condition=IfCondition(mission_enabled),
        ),
    ])
