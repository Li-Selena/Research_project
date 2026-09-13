# Corrected Mechanical Arm MDH Parameter Reference

Convention:

```text
T(i-1,i) = Rx(alpha(i-1)) * Tx(a(i-1)) * Rz(theta_i_mdh) * Tz(d_i)
```

All lengths are in millimeters. The physical power-on pose is:

```text
theta1 = theta2 = theta3 = 0 deg
```

## World Frame

```text
X0: forward
Y0: left
Z0: up
```

## MDH Parameter Table

| i | frame | alpha(i-1) | a(i-1) | d_i | theta_i_mdh |
|---:|---|---:|---:|---:|---:|
| 1 | joint1 | 0 | 0 | 98.18 | theta1 + pi/2 |
| 2 | joint2 | pi/2 | 42.64 | 0 | theta2 + pi/2 |
| 3 | joint3 | 0 | 383.929445 | 0 | theta3 + pi/2 |
| 4 | tool | pi/2 | 0 | 0 | pi/2 |

## Zero-Pose Check

With `theta1 = theta2 = theta3 = 0`:

```text
O0 = (0, 0, 0)
O1 = (0, 0, 98.18)
O2 = (42.64, 0, 98.18)
O3 = (42.64, 0, 482.109445)
O4 = O3
```

Axis directions at zero:

```text
frame 1:
  X1 = +X0
  Y1 = +Y0
  Z1 = +Z0

frame 2:
  X2 = +Z0
  Y2 = -X0
  Z2 = -Y0

frame 3:
  X3 = -X0
  Y3 = -Z0
  Z3 = -Y0

frame 4:
  X4 = -Y0
  Y4 = +X0
  Z4 = +Z0
```

## Notes

`O1` is the strict MDH frame origin on the joint1 axis, not the physical joint1
center. The physical joint1 center remains at `O0 = (0, 0, 0)`.

Frame 4 is the tool frame. Its origin coincides with frame 3. At zero,
`X4=-Y0` and `Z4=+Z0`; after motion, frame 4 follows joint3 rotation through
the fixed transform from frame 3 to frame 4.

## Tool Points In Frame 4

The following tool feature points are expressed in the joint4/tool coordinate
frame `{4}`:

| tool point | x4 | y4 | z4 |
|---|---:|---:|---:|
| suction cup 1 | 0 | -46.79 | 45.5 |
| suction cup 2 | 0 | 92.6 | 0 |
| gripper | 103 | -92 | 10.3 |

At the zero pose, frame `{4}` is rotated 90 degrees about Z from frame `{0}` and:

```text
O4 = (42.64, 0, 482.109445)
```

Therefore the same points expressed in frame `{0}` at zero pose are:

| tool point | x0 | y0 | z0 |
|---|---:|---:|---:|
| suction cup 1 | -4.15 | 0 | 527.609445 |
| suction cup 2 | 135.24 | 0 | 482.109445 |
| gripper | -49.36 | -103 | 492.409445 |
