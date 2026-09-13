# Coordinate system

All target coordinates leaving `pnp_solve` use a right-handed FLU frame:

- `+X`: forward
- `+Y`: left
- `+Z`: up
- positive yaw: counterclockwise when viewed from above

OpenCV optical coordinates use `+X` right, `+Y` down and `+Z` forward. For a
camera facing forward with no additional mounting rotation, conversion is:

```text
X_flu =  Z_camera
Y_flu = -X_camera
Z_flu = -Y_camera
```

`camera_mount_rpy_deg` applies an additional
`Rz(yaw) * Ry(pitch) * Rx(roll)` rotation after this axis conversion.
`camera_to_base_xyz_m` then adds the camera-origin position expressed in the
configured output FLU frame.

`PnpResult.x/y/z/distance` use metres. `robot_serial_bridge` accepts only its
configured `expected_frame_id` (`arm_base_link` by default), converts metres to
millimetres and emits the STM32 `ARM_SET_TARGET` command. The serial bridge is
disabled by default.
