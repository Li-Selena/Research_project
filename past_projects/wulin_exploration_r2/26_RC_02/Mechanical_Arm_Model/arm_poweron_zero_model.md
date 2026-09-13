# Mechanical Arm Power-On Zero Model

This note records the corrected physical zero pose used by
`arm_poweron_zero_topology_axes.svg`.

## World Frame

```text
X0: forward
Y0: left
Z0: up
```

## Zero Pose

At power-on:

```text
theta1 = 0 deg
theta2 = 0 deg
theta3 = 0 deg
```

The assumed joint origins are:

```text
joint1 = (0, 0, 0)
joint2 = (42.64, 0, 98.18)
joint3 = (42.64, 0, 482.109445)
joint4 = joint3
```

`joint2 -> joint3` is vertical in this pose:

```text
joint3 - joint2 = (0, 0, 383.929445)
```

## Axis Directions At Zero

```text
frame 0:
  X0 = forward
  Y0 = left
  Z0 = up

frame 1 / joint1:
  X1 = -Y0
  Y1 = +X0
  Z1 = +Z0

frame 2 / joint2:
  X2 = +Z0
  Y2 = -X0
  Z2 = -Y0

frame 3 / joint3:
  X3 = +Z0
  Y3 = -X0
  Z3 = -Y0

frame 4 / tool:
  X4 = -Y0
  Y4 = +X0
  Z4 = +Z0
```

## Tool Frame Correction

The tool frame `{4}` is not fixed by the `joint2 -> joint3` link direction.
Its origin always coincides with joint3, and its orientation follows the
rotation of joint3.

At the initial zero pose only:

```text
Z4 = +Z0
```

When `theta3` changes, frame `{4}` rotates with joint3 about the joint3 roll
axis. In the zero pose used here, that roll axis is:

```text
Z3 = -Y0
```
