# Zoom controller integration

## Contract

Input topic:

```text
/lens/target_intrinsics
std_msgs/msg/Float64MultiArray
data = [fx_target_px, fy_target_px]
```

The node independently inverts `fx(Z)` and `fy(Z)`.  It rejects the command
when the two inferred positions differ by more than
`max_fx_fy_disagreement_steps`.  A valid pair is converted to one absolute
motor command:

```text
ZPOS round((z_from_fx + z_from_fy) / 2)
```

Output topics:

```text
/lens/zoom_state   std_msgs/msg/Float64MultiArray
/lens/camera_info  sensor_msgs/msg/CameraInfo
```

`zoom_state.data` order:

```text
[valid, target_fx, target_fy, z_from_fx, z_from_fy, commanded_z,
 predicted_fx, predicted_fy, k1, k2, p1, p2, k3]
```

`predicted_*` means the calibrated value at the completed absolute motor
command.  There is no zoom encoder, so it is not a direct physical readback.

## Dry-run first

```bash
colcon build --packages-select velocity_servo_tag
source install/setup.bash
ros2 launch velocity_servo_tag velocity_servo_tag.launch.py \
  start_detector:=false start_mapper:=false \
  start_zoom_controller:=true zoom_dry_run:=true
```

Send the exact Z=2000 calibration pair:

```bash
ros2 topic pub --once /lens/target_intrinsics \
  std_msgs/msg/Float64MultiArray "{data: [4505.996199, 4503.318979]}"
```

Expected command position: `Z=2000`.

## Hardware mode

Use a stable Linux device path from `/dev/serial/by-id/`, then update
`serial_port` in `config/velocity_servo_tag.yaml`.

```bash
ros2 launch velocity_servo_tag velocity_servo_tag.launch.py \
  start_detector:=false start_mapper:=false \
  start_zoom_controller:=true zoom_dry_run:=false
```

Homing is deliberately explicit and must be supervised because the current
firmware homes Z/F against mechanical endpoints without limit switches:

```bash
ros2 service call /zoom_controller_node/home std_srvs/srv/Trigger "{}"
```

Only after the service succeeds should target intrinsics be published.

## Calibration limitation

The current table is the 1920x1080 Windows MSMF calibration with constrained
`k3=0`.  Before final deployment, verify Z=0, 1200, and 2400 using the Ubuntu
V4L2/MJPG capture path.  Until then, keep `zoom_dry_run=true` in shared launch
defaults.
