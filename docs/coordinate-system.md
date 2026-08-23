# Coordinate systems

## Contract

The Sender publishes `Twc`: the camera pose in the SLAM world coordinate
system, advertised as `frame: "slam_world"`. Sending `Tcw` is a protocol
violation and must not be corrected by guessing in the Receiver.

The canonical SLAM frame follows the OpenCV camera-axis convention:

```text
+X: right
+Y: down
+Z: forward
handedness: right-handed
unit: metre
```

The Unity frame is:

```text
+X: right
+Y: up
+Z: forward
handedness: left-handed
unit: metre
```

If a chosen SLAM library exposes another convention or `Tcw`, its adapter in
the Sender must convert that output to this canonical wire contract.

## Receiver conversion

Let `S = diag(1, -1, 1)`. The Receiver converts positions and rotations as:

```text
p_unity = S * p_slam
R_unity = S * R_slam * S
```

For protocol quaternions in `[x, y, z, w]` order, the equivalent conversion is:

```text
q_unity = [-x, y, -z, w]
```

Therefore a point `[x, y, z]` becomes `[x, -y, z]`. Both camera positions and
point-cloud positions use the same position conversion.

After conversion the Receiver republishes settings with
`frame: "unity_world"`. Unity rejects telemetry until it has received those
local settings; it must not perform the conversion a second time.

## Quaternion handling

The Receiver must:

1. reject a quaternion whose length is zero or non-finite;
2. normalize the quaternion;
3. convert it to Unity coordinates;
4. enforce sign continuity against the previous accepted pose.

Quaternion `q` and `-q` represent the same rotation. For consecutive converted
quaternions `previous` and `current`, negate `current` when their dot product is
negative. This prevents visual interpolation from taking the long path.

## Verification fixtures

These values are acceptance fixtures for Receiver tests:

| SLAM value | Expected Unity value |
|---|---|
| position `[1, 2, 3]` | `[1, -2, 3]` |
| identity quaternion `[0, 0, 0, 1]` | `[0, 0, 0, 1]` |
| quaternion `[0.1, 0.2, 0.3, 0.92736185]` | normalized `[-0.1, 0.2, -0.3, 0.92736185]` |
| point `[-4, 5, 6]` | `[-4, -5, 6]` |

Before connecting a real SLAM implementation, test a known camera motion in
each positive axis and a known 90-degree rotation around each axis. A mirrored
trajectory or reversed rotation is a contract/adaptor bug, not a Unity camera
setting to compensate for.
