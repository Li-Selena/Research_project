# vo2 ROS 2 vision positioning

`vo2` detects a target from a USB camera, estimates its 3D position, and
controls the R2 STM32 over a bidirectional USB CDC link. The bridge supports
mechanical-arm and end-tool control, local chassis odometry, velocity and pose
motion, and upstairs/downstairs sequences. An optional mission server connects
these controls to an existing Nav2 localization stack. The supported runtime
is Ubuntu with ROS 2 Humble and Python 3.10.

## Data flow

```text
camera -> target_detection -> /yolo_result -> pnp_solve -> /pnp_result
                                                               |
ROS command service + cmd_vel -> robot_serial_bridge <-> STM32 protocol v4
                                      |             |
                                /robot/odom   status/result topics
```

The published robot coordinates use a right-handed FLU frame:

- `+X`: forward
- `+Y`: left
- `+Z`: up
- positive yaw: counterclockwise when viewed from above

OpenCV optical coordinates are converted with `Xrobot=Zcamera`,
`Yrobot=-Xcamera`, `Zrobot=-Ycamera`. Position and distance fields use metres.
The default output origin is `arm_base_link`; configure the camera translation
relative to the real arm-base origin before enabling serial output.
See [COORDINATE_SYSTEM.md](COORDINATE_SYSTEM.md) for the complete convention.
The serial bridge converts positions to millimetres before transmission.

## Packages

- `msg_interface`: `YoloBox`, `YoloResult` and `PnpResult` messages.
- `target_detection`: camera capture and Ultralytics YOLO inference.
- `pnp_solve`: calibrated PnP, FLU transformation and target-loss prediction.
- `robot_serial_bridge`: opt-in bidirectional STM32 control, status and odometry
  bridge. Its complete command reference is
  [CONTROL.md](src/robot_serial_bridge/CONTROL.md).
- `r2_mission`: optional `TraverseAndPick` action server coordinating Nav2,
  terrain traversal and vision-guided picking. See
  [MISSION.md](src/r2_mission/MISSION.md).

## Install and build

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The embedded model is `yolov8n_test.pt`. Supply another model with the
`model_path` parameter when required.

## Camera calibration

Copy the example before running PnP:

```bash
cp src/pnp_solve/config/calibration.example.yaml \
   src/pnp_solve/config/calibration.yaml
cp src/pnp_solve/config/pnp.example.yaml \
   src/pnp_solve/config/pnp.yaml
```

Replace the example camera matrix and distortion coefficients with values from
the actual camera. The node rejects the bundled approximate calibration unless
`allow_example_calibration:=true` is explicitly selected for a smoke test.
Copy `pnp.example.yaml` as `pnp.yaml` and update the camera position, mounting
angles and target radius. Pass it with `pnp_config_file:=...` when launching.

The target model uses four planar points at the top, bottom, left and right of
a target with radius `target_radius_m` (default `0.12`). Measure the real target
and set this parameter before using the reported distance.

## Run

Run target detection by itself:

```bash
ros2 run target_detection target_detection --ros-args \
  -p camera_id:=0 -p show_image:=true
```

Run detection and PnP with a real calibration:

```bash
ros2 launch pnp_solve vo2.launch.py \
  calibration_file:=$PWD/src/pnp_solve/config/calibration.yaml \
  pnp_config_file:=$PWD/src/pnp_solve/config/pnp.yaml
```

For a headless computer, add `show_image:=false`.

Camera mounting is configured in the robot FLU frame:

```bash
ros2 run pnp_solve pnp_solve --ros-args \
  -p calibration_file:=$PWD/src/pnp_solve/config/calibration.yaml \
  -p camera_to_base_xyz_m:="[0.20, 0.0, 0.50]" \
  -p camera_mount_rpy_deg:="[0.0, 0.0, 0.0]"
```

## STM32 control link

Serial output is disabled by default so starting the launch file cannot move
the robot. Once the target coordinates have been verified, enable it explicitly:

```bash
ros2 launch pnp_solve vo2.launch.py \
  calibration_file:=$PWD/src/pnp_solve/config/calibration.yaml \
  serial_enabled:=true serial_port:=/dev/ttyACM0
```

Add `claim_usb_source:=true` to select USB control automatically. The bridge
reads protocol-v4 feedback, validates CRC and payload lengths, publishes
`/robot/status`, `/robot/command_result` and `/robot/odom`, and reconnects after
link loss. It confirms each service action from firmware feedback instead of
treating a successful serial write as completion.

The bridge can forward measured, valid `arm_base_link` targets. Predicted
points and automatic arm enabling are disabled by default. They can be
controlled with `send_predicted_targets` and `enable_arm_on_start`. For manual
arm testing, launch with `vision_control_enabled:=false`.

The serial frame is `A5 5A LEN CMD DATA CRC_H CRC_L FF`, with CRC16/Modbus over
the header through payload. Arm targets use command `0x23` and four little-endian
float32 values `(x_mm, y_mm, z_mm, 0)`.

For service calls covering arm, gripper, suction, timed velocity, local pose
motion and climbing, see
[the control guide](src/robot_serial_bridge/CONTROL.md). `move_to` uses local
encoder/IMU odometry; the project does not provide map localization or SLAM.

## Existing SLAM/Nav2 integration

When map localization and Nav2 are already running, start the complete
navigation, traversal and picking interface with:

```bash
ros2 launch pnp_solve vo2.launch.py \
  calibration_file:=$PWD/src/pnp_solve/config/calibration.yaml \
  pnp_config_file:=$PWD/src/pnp_solve/config/pnp.yaml \
  serial_enabled:=true claim_usb_source:=true \
  cmd_vel_enabled:=true vision_control_enabled:=false \
  mission_enabled:=true
```

The bridge publishes `/robot/odom` and, by default, the
`odom -> base_link` transform with configurable planar covariance. The
localization stack should provide `map -> odom`; disable `publish_odom_tf` if
another odometry component already owns that transform. Nav2 must publish its
velocity output on `/cmd_vel`.

The mission action accepts optional map-frame terrain and pickup poses, an
`up`/`down` terrain action, and a `gripper`/`suction` choice. It cancels Nav2
and requests a robot stop on failure or cancellation. Full usage is documented
in [MISSION.md](src/r2_mission/MISSION.md).

## Test

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

The upstream snapshot did not declare a source license. Select and document a
license before redistributing the modified source. The embedded Ultralytics
checkpoint also carries its own licensing metadata.
