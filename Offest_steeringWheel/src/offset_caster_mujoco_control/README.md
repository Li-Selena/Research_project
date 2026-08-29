# Offset Caster MuJoCo Control

ROS 2 Jazzy C++ package for keyboard control of the four powered-caster MuJoCo model.

## Data flow

```text
offset_caster_dashboard (world-position /offset_caster/motion_command)
  -> motion_controller_node (/cmd_vel)
  -> inverse_kinematics_node
  -> /offset_caster/joint_velocity_command
  -> mujoco_state_node (data.ctrl, mj_step)
  -> /joint_states
  -> inverse_kinematics_node

teleop_twist_keyboard (/cmd_vel)
  -> inverse_kinematics_node

mujoco_state_node
  -> /offset_caster/forward_velocity
  -> /offset_caster/forward_kinematics_residual
  -> /odom
  -> /clock
  -> /imu/data
  -> world -> base_link -> imu_link TF

motion_controller_node
  -> /offset_caster/motion_status

offset_caster_curve_monitor
  <- /offset_caster/motion_status
  <- /joint_states
```

## Build inside the Dev Container

```bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

## Run

Start the controller and MuJoCo viewer:

```bash
ros2 launch offset_caster_mujoco_control simulation_control.launch.py \
  enable_viewer:=true \
  enable_dashboard:=true \
  enable_monitor:=true
```

The position dashboard and curve monitor are independent Qt programs. They can also be started
separately with `ros2 run offset_caster_dashboard offset_caster_dashboard` and
`ros2 run offset_caster_dashboard offset_caster_monitor`.

In a second terminal, start the standard ROS 2 keyboard node:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel
```

The keyboard node's uppercase holonomic keys command lateral motion. Any unsupported key stops
the vehicle. Both controller layers also stop automatically when their command timeout expires.

If the viewer cannot connect to X11 from the root Dev Container, allow the local root user before
opening the container:

```bash
xhost +si:localuser:root
```

Revoke that permission when finished:

```bash
xhost -si:localuser:root
```
