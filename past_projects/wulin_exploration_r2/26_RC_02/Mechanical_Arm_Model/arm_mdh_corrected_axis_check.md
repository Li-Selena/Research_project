# Corrected MDH Axis Check

This note follows the corrected power-on zero description.

## World Frame

```text
X0: forward
Y0: left
Z0: up
```

## Physical Zero Pose

```text
theta1 = theta2 = theta3 = 0 deg

J1 = (0, 0, 0)
J2 = (42.64, 0, 98.18)
J3 = (42.64, 0, 482.109445)
J4 = J3
```

The `J2 -> J3` link is vertical at zero:

```text
J3 - J2 = (0, 0, 383.929445)
```

## Axis Directions At Zero

```text
z1 = +Z0
x1 = +X0

z2 = -Y0
x2 = +Z0

z3 = -Y0
x3 = -X0

z4 = +Z0
x4 = -Y0
```

The missing axes from the right-hand rule are:

```text
y1 = +Y0
y2 = -X0
y3 = -Z0
y4 = +X0
```

## Craig-MDH Check

Using the Craig modified DH rule, `x_i` is the common-normal direction between
`z_i` and `z_(i+1)`.

| pair | check | result |
|---|---|---|
| z1 to z2 | `z1=+Z0`, `z2=-Y0`; common normal is `+/-X0` | `x1=+X0` is valid |
| z2 to z3 | `z2` and `z3` are parallel `-Y0`; J2-to-J3 separation is `+Z0` | `x2=+Z0` is valid |
| z3 to z4 | `z3=-Y0`, `z4=+Z0`; axes intersect at J3/J4 | `x3=-X0` is valid |

For strict Craig-MDH placement, the MDH frame `{1}` origin should lie on `z1`
at the height where the common normal to `z2` begins:

```text
O1_mdh = (0, 0, 98.18)
```

The physical joint1 center can still be shown at:

```text
J1 = (0, 0, 0)
```

Frame `{4}` is the tool frame. Its origin always coincides with J3/J4. At zero,
`z4=+Z0` and `x4=-Y0`; after motion, frame `{4}` follows joint3 rotation.
