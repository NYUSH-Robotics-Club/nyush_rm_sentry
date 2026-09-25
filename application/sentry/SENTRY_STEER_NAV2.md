# Sentry Nav2 swerve control

This standalone control path reuses the steering zero positions, motor PID
settings, CAN assignments, wheel geometry, and drive-speed limit from the
keyboard steering test. It replaces eight discrete directions with continuous
body-frame velocity commands.

## Data flow

1. `scripts/sentry_nav2_bridge.py` subscribes to the final ROS 2 `/cmd_vel`.
2. The bridge limits translation to 0.80 m/s and yaw rate to 2.4 rad/s.
3. A CRC-protected USB CDC frame carries `vx`, `vy`, and `wz` to the controller.
4. `SentrySteerNav2Task()` computes one velocity vector per swerve module.
5. Each steering target is optimized by reversing its drive wheel when that
   avoids more than 90 degrees of steering motion.
6. Motor telemetry returns over USB and the bridge publishes `/wheel/odom`.

ROS coordinates are `+x` forward, `+y` left, and `+yaw` counter-clockwise.
Controller command units are mm/s and mrad/s. Motor order is LF, RF, LR, RR.

## Browser aiming and lower-yaw request

The browser sends arrow-key aim rates on `/sentry/gimbal_manual`; the USB bridge
encodes these in a second 15-byte CRC-protected frame (type `0x02`). Left/right
drive the upper GM6020 yaw on CAN1 motor ID 3. The inherited ID 6 belongs to
the lower yaw, not the upper aiming axis. The upper yaw target starts at the
first valid feedback angle, is limited to 10 degrees from that angle, and the
motor stops when the browser command goes stale. The chassis command remains
type `0x01`, so aiming cannot silently change chassis `wz`.

The inherited sentry config says pitch is CAN2 ID 7 and sets both angle limits
to zero. A separate `controls_typec` sentry config says CAN2 ID 5 with limits
-28 to +25 degrees. Neither identifies the live motor in this robot. Pitch is
disabled by `SENTRY_AIM_PITCH_ENABLED=0` until its ID, polarity and physical
limits are verified on the robot. The browser suppresses up/down input. Keep
the mechanism unpowered while identifying the motor and measuring its free
travel; leave margin inside both hard stops when setting software limits.

The operator confirmed the lower yaw is DaMiao on CAN1 ID 6. The speed-mode
backend uses the existing bench-profile master ID 0 and is enabled by
`SENTRY_LOWER_YAW_DM_ENABLED=1`. B toggles its 0.5 rad/s request; toggling it
off commands zero speed, and a stale or disabled browser command stops the motor.
The type `0x02` frame carries B independently of chassis `wz`, so lower-yaw
rotation does not command wheel-driven chassis spin. Verify direction and
actual speed with the robot supported before increasing the configured rate.

## Safety behavior

- Commands expire after 300 ms.
- A missing steering or drive motor stops every chassis motor.
- A fresh, enabled zero-speed command keeps the previous steering direction.
  After motion, the drive motors actively brake until all four remain below
  100 RPM for 30 ms, with a 200 ms hard timeout, and then stop their closed
  loops. Enabling control while already stationary leaves the drive motors
  stopped, preventing zero-speed feedback noise from exciting a lifted wheel.
  A disabled, stale, or offline command still hard-stops every motor.
- PID history is cleared whenever control re-enters from the disabled state, so
  the previous command cannot inject stale integral or filtered output into the
  next movement.
- All active drive modules share one steering-alignment scale. Drive remains
  zero while any active module is more than 16.875 degrees from its target,
  then all four requested RPMs ramp together and reach full output at 5.625
  degrees. Preserving their ratios prevents asymmetric wheel force from
  changing the requested chassis direction during steering transitions.
- A module with negligible requested speed holds its last meaningful steering
  target to avoid jitter around an undefined zero-speed direction. On first
  use it adopts the current angle; a later zero-speed crossing therefore does
  not interrupt and restart an active turn.
- Nav2 steering uses the proven proportional-only outer loop (`Kp=20`) and
  filtered proportional speed loop (`Kp=40`) from the original keyboard
  controller. A 720 deg/s outer limit keeps direction changes fast, while the
  7000-command current cap prevents the measured +/-15000 limit cycle.
- The drive-speed loop is an unfiltered proportional-only loop (`Kp=6`) with a
  120 RPM error deadband. This avoids chasing feedback noise and removes the
  measured phase-lag limit cycle. Integral action should only be reintroduced
  after repeatable loaded tests show a steady RPM bias without oscillation.
- RTT diagnostics report each steering error and target/actual logical drive
  RPM every 500 ms for zero-offset and closed-loop verification.
- Dashboard protocol v8 publishes the four steering target/actual angles,
  steering speed-loop references and feedback, controller output/current,
  drive RPM, and drive output/current at 100 Hz. The browser `Swerve` tab keeps
  continuous plots and the monitor export stores the same samples in TSV form.
- Closing the bridge sends three disabled zero-speed frames.
- USB CDC output is copied into a persistent transmit buffer before the
  asynchronous transfer starts. A transfer that remains busy for 250 ms is
  aborted and retried, so one lost IN completion cannot permanently stop wheel
  odometry.
- The ROS bridge closes and rediscovers the NYUSH USB controller when telemetry
  remains stale for 1.5 seconds. Reconnection resets only the MCU timestamp
  baseline; the accumulated odometry pose is retained.

## Build and run

Build the firmware without flashing:

```sh
just build sentry_swerve
```

After explicitly flashing the firmware, run the bridge on the NUC:

```sh
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
ros2 run mid360_nav_bringup sentry_nav2_bridge.py
```

Automatic selection matches USB VID/PID `0483:00ca`, so it remains valid if
Linux changes `ttyACM0` to another number after re-enumeration. A fixed `port`
parameter should only be used when more than one matching controller is
connected. Stale warnings include received-byte, valid-frame, CRC-error, and
discarded-byte counters for distinguishing a silent controller from malformed
frames.

Keep the chassis lifted for the first sign and steering-direction checks.

## RTT swerve plots

Stop any existing RTT reader before starting the browser bridge because one
J-Link probe can only be owned by one reader:

```sh
just logger
```

Open `http://JETSON_IP:8765/` from a device on the same LAN and select
`Swerve`. A stable target
with oscillating angle error indicates closed-loop tuning or mechanical
backlash; a jumping target points to direction selection or command input; a
stable steering trace with oscillating drive RPM isolates the drive loop.
