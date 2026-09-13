# Mechanical Arm Workspace Limits

Record date: 2026-06-22

This note records the mechanical-arm safety limits after the new power-on
reference pose. J1 CAN output clamp (+/-180 deg) is ignored here; without that
clamp, the 2D rho-z workspace rotates fully around the Z axis.

## Parameters

| Parameter | Old | New |
|---|---:|---:|
| d1 | 0 mm | 0 mm |
| a2 | 320 mm | 320 mm |
| a3 | 320 mm | 320 mm |
| j1_ref | 0 deg | 0 deg |
| j2_ref | 70 deg | 80 deg |
| j3_ref | -145 deg | -165 deg |
| dir | j1=+1, j2=-1, j3=+1 | unchanged |

New power-on model pose:

```text
theta1 = 0 deg
theta2 = 80 deg
theta3 = -165 deg
j3_ctrl = 0 deg
position = (x=3.64 mm, y=0.00 mm, z=83.46 mm)
```

## Model Formula

The current forward kinematics in `Arm3R_FK_Model()`:

```text
r = -a2 * sin(theta2) - a3 * sin(theta2 + theta3)
x = cos(theta1) * r
y = sin(theta1) * r
z = d1 + a2 * cos(theta2) + a3 * cos(theta2 + theta3)
```

## Coordinate Convention

The arm target coordinate frame is:

```text
+X: robot/front direction
+Y: robot/left direction
+Z: vertical upward direction
```

J1 angle follows `atan2(y, x)`:

```text
target on +X axis:      theta1 = 0 deg
rotate +X toward +Y:    theta1 is positive, up to +180 deg
rotate +X toward -Y:    theta1 is negative, down to -180 deg
target on -X axis:      theta1 is the +/-180 deg boundary
```

Firmware keeps this sign convention for both USB and USART XY targets. Only the
exact rear axis is ambiguous, because `+180 deg` and `-180 deg` describe the
same direction there.

With the new reference pose:

```text
j3_ctrl = (theta2 + theta3) - (j2_ref + j3_ref)
j2_ref + j3_ref = -85 deg
theta2 + theta3 = j3_ctrl - 85 deg
```

## Safety Limits

The physical theta2 split rule remains at theta2 = 0 deg. The theta2 upper
limit includes the new reachable power-on pose at 80 deg.

```text
theta2 total range: -62 deg <= theta2 <= 80 deg

theta2 > 0:
  0 deg <= j3_ctrl <= 51.191847 deg

theta2 < 0:
  -63.22950 deg <= j3_ctrl <= 10 deg

theta2 == 0:
  -63.22950 deg <= j3_ctrl <= 51.191847 deg
```

## Workspace By Segment

`rho = |r|` is the horizontal radius after rotating the 2D workspace around Z.

### theta2 < 0

```text
theta2: -62 deg ~ 0 deg
j3_ctrl: -63.22950 deg ~ 10 deg
phi = theta2 + theta3: -148.22950 deg ~ -75 deg

rho: 168.49 mm ~ 602.54 mm
z:   -121.82 mm ~ 402.82 mm
```

Key points:

```text
lowest:       theta2=-62 deg, j3_ctrl=-63.22950 deg -> rho=451.03 mm, z=-121.82 mm
farthest:     theta2=-62 deg, j3_ctrl=-5 deg        -> rho=602.54 mm, z=150.23 mm
upper edge:   theta2=0 deg,   j3_ctrl=10 deg        -> rho=309.10 mm, z=402.82 mm
```

### theta2 == 0

```text
theta2: 0 deg
j3_ctrl: -63.22950 deg ~ 51.191847 deg
phi = theta2 + theta3: -148.22950 deg ~ -33.808153 deg

rho: 168.49 mm ~ 320.00 mm
z:   47.95 mm ~ 585.89 mm
```

Key points:

```text
lowest/nearest: theta2=0 deg, j3_ctrl=-63.22950 deg -> rho=168.49 mm, z=47.95 mm
farthest:       theta2=0 deg, j3_ctrl=-5 deg        -> rho=320.00 mm, z=320.00 mm
highest:        theta2=0 deg, j3_ctrl=51.191847 deg -> rho=178.05 mm, z=585.89 mm
```

### theta2 > 0

```text
theta2: 0 deg ~ 80 deg
j3_ctrl: 0 deg ~ 51.191847 deg
phi = theta2 + theta3: -85 deg ~ -33.808153 deg

rho: 0.00 mm ~ 318.78 mm
z:   83.46 mm ~ 585.89 mm
```

Key points:

```text
power-on/lowest: theta2=80 deg,    j3_ctrl=0 deg         -> rho=3.64 mm,   z=83.46 mm
nearest:         theta2~=33.81 deg,j3_ctrl~=51.19 deg    -> rho~=0.00 mm,  z~=531.78 mm
highest:         theta2=0 deg,     j3_ctrl=51.191847 deg -> rho=178.05 mm, z=585.89 mm
farthest:        theta2=0 deg,     j3_ctrl=0 deg         -> rho=318.78 mm, z=347.89 mm
```

## Overall Workspace

Ignoring the J1 +/-180 deg output clamp:

```text
rho: 0.00 mm ~ 602.54 mm
z:   -121.82 mm ~ 585.89 mm
```

The 2D rho-z region above rotates fully around the Z axis if no J1 limit is
applied.

## J1 Low-Height XZ Plane Rule

Mechanical note: the large arm is offset from the model vertical direction by
20 deg. To keep the mechanism away from unsafe side rotation at low height, J1
rotation is gated by target height:

```text
if target z > 250 mm:
  J1 may rotate normally toward the target XY direction.

if target z <= 250 mm:
  target must stay in the model XZ plane.
  theta1 is locked to 0 deg.
  accepted plane tolerance: |y| <= 5 mm
  workspace radius check uses the XZ projection: rho = |x|
```

Targets that violate this low-height XZ-plane rule return:

```text
status = ARM3R_ERR_UNSAFE
unsafe_reason = ARM3R_UNSAFE_J1_LOCK_PLANE (4)
```

## Control Input Enforcement

Both USB and USART arm target inputs use the same IK-space precheck before a
target is accepted by the control chain.

```text
USB ARM_SET_TARGET:
  valid target   -> Arm_task_USB(), watchdog fed
  invalid target -> IK error status is reported, watchdog disarmed, previous output kept

USART remote frame:
  valid target   -> arm_X/arm_Y/arm_Z updated, arm_input_valid = 1
  invalid target -> arm_X/arm_Y/arm_Z not updated, arm_input_valid = 0, previous output kept
```

The old rectangular input clamp is no longer used for arm targets. Out-of-space
coordinates are rejected instead of being silently moved to a boundary value.

## J1 Output Limit And XY Sign

Joint 1 output is limited to +/-180 deg. The IK model returns theta1 from
`atan2(y, x)`, and the motor target keeps that coordinate sign:

```text
y > 0 side -> positive J1
y < 0 side -> negative J1
exact -X axis -> +/-180 deg boundary
```

For the exact rear axis (`x < 0`, `y ~= 0`), the firmware keeps the boundary on
the previous active side: previous negative J1 uses `-180 deg`, previous
positive J1 uses `+180 deg`. This avoids sign chatter at the one ambiguous
direction while still keeping normal USART/USB XY targets mapped to the correct
`[-180, +180] deg` coordinate angle.

## 50 mm Inward Target Envelope

For target commands, the firmware applies a conservative 50 mm inward Cartesian
envelope by theta2 segment. This is stricter than the raw joint safety limits
above and is intended to keep commanded targets away from workspace boundaries.

The new power-on pose is allowed as the mechanical reference pose, but it is
outside this inward target envelope:

```text
power-on: rho=3.64 mm, z=83.46 mm
positive-segment target envelope requires rho >= 50.00 mm and z >= 133.46 mm
```

### theta2 < 0 target envelope

```text
theta2: -62 deg ~ 0 deg
j3_ctrl: -63.22950 deg ~ 10 deg

rho: 218.49 mm ~ 552.54 mm
z:   -71.82 mm ~ 352.82 mm
```

### theta2 == 0 target envelope

At the split line, the target is accepted only if it lies inside both adjacent
segment envelopes. This is intentionally conservative so the split line keeps
clearance from both sides.

```text
negative-side envelope:
  rho: 218.49 mm ~ 552.54 mm
  z:   -71.82 mm ~ 352.82 mm

positive-side envelope:
  rho: 50.00 mm ~ 268.78 mm
  z:   133.46 mm ~ 535.89 mm

combined intersection:
  rho: 218.49 mm ~ 268.78 mm
  z:   133.46 mm ~ 352.82 mm
  approximate j3_ctrl on theta2=0: -40.65 deg ~ -37.87 deg
```

### theta2 > 0 target envelope

```text
theta2: 0 deg ~ 80 deg
j3_ctrl: 0 deg ~ 51.191847 deg

rho: 50.00 mm ~ 268.78 mm
z:   133.46 mm ~ 535.89 mm
```
