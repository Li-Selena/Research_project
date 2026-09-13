# Mechanical Arm MDH Parameter Table

This model uses the Craig modified Denavit-Hartenberg convention:

```text
T(i-1,i) = Rx(alpha(i-1)) * Tx(a(i-1)) * Rz(theta(i)) * Tz(d(i))
```

All linear dimensions are in millimeters. Angles are in degrees unless noted.

## Robot Frame

The robot frame given by the mechanism description is:

```text
+X_robot: forward
+Y_robot: left
+Z_robot: up
```

The MDH base frame `{0}` is aligned with the canonical robot frame:

```text
+X_mdh = +X_robot
+Y_mdh = +Y_robot
+Z_mdh = +Z_robot
```

Coordinate conversion:

```text
X_robot = X_mdh
Y_robot = Y_mdh
Z_robot =  Z_mdh
```

## Zero-Pose Assumption

The current MDH table assumes the same topology pose used in
`arm_topology_oyz.svg`:

```text
theta1 = 0: joint2 is at robot (42.64, 0, 98.18)
theta2 = 0: the joint2-to-joint3 endpoint chord points along +X_robot
theta3 = 0: joint3 roll axis is parallel to joint2 roll axis
```

If the real power-on or encoder zero is different, keep the same geometric MDH
table and replace the variable columns with:

```text
theta1_mdh = theta1 + theta1_offset
theta2_mdh = theta2 + theta2_offset
theta3_mdh = theta3 + theta3_offset
```

## MDH Table

| i | joint | alpha(i-1) | a(i-1) | theta(i) | d(i) | note |
|---:|---|---:|---:|---:|---:|---|
| 1 | joint1 yaw | 0 | 0 | theta1 | 98.18 | z1 is the yaw axis; frame origin is lifted to joint2 height |
| 2 | joint2 roll | +90 | 42.64 | theta2 | 0 | z2 is the roll axis; at theta1=0 it points along -Y_robot |
| 3 | joint3 roll | 0 | 383.929445 | theta3 | 0 | z3 is parallel to z2; a2 is the joint2-joint3 endpoint chord |

The convex mechanical link between joint2 and joint3 is represented in MDH by
its endpoint chord length. The bend shape is auxiliary geometry, not an extra
joint.

## Joint Positions From This Table

In robot coordinates, with `L23 = 383.929445`:

```text
joint1 = (0, 0, 0)

joint2 =
(
   42.64 * cos(theta1),
   42.64 * sin(theta1),
   98.18
)

joint3 =
(
    (42.64 + L23 * cos(theta2)) * cos(theta1),
    (42.64 + L23 * cos(theta2)) * sin(theta1),
    98.18 + L23 * sin(theta2)
)
```

With `theta1 = theta2 = 0`:

```text
joint2 = (42.64, 0, 98.18)
joint3 = (426.569445, 0, 98.18)
```

## Convex Link Auxiliary Geometry

Given:

```text
joint2-side segment = 270.98076211
joint3-side segment = 124.55
included angle      = 150 deg
endpoint chord      = 383.929445
```

The convex vertex relative to joint2 in the zero-pose link frame is:

```text
x_link = 267.392476
y_link =  43.952671
z_link =   0
```

where:

```text
+X_link: joint2 -> joint3 endpoint chord
+Y_link: convex side of the link in the XZ plane
+Z_link: roll axis direction, -Y_robot at theta1=theta2=0
```

The endpoint distance calculated from the two segment lengths and 150 deg is:

```text
383.928539331
```

This differs from the provided endpoint distance by about `0.000906 mm`, so the
MDH table uses the provided endpoint chord `383.929445 mm`.
