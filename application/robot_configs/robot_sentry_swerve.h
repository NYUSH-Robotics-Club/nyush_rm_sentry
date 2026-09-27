#pragma once
#ifndef ROBOT_SENTRY_SWERVE_CONFIG_H
#define ROBOT_SENTRY_SWERVE_CONFIG_H

/* Reuse the existing sentry configuration and override only the standalone swerve-control settings. */
#include "robot_sentry.h"

#define SENTRY_STEER_KEYBOARD_TEST 0
#define SENTRY_STEER_NAV2_CONTROL 1

// The inherited sentry ID 6 is the lower yaw on this robot, not aiming yaw.
#define SENTRY_AIM_YAW_CAN_BUS hfdcan2
#define SENTRY_AIM_YAW_MOTOR_ID 4u
// Lower yaw: DaMiao on CAN1 ID 6 (operator-confirmed).
#define SENTRY_LOWER_YAW_MOTOR_ID 6u
#define SENTRY_LOWER_YAW_CAN_BUS hfdcan1
#define SENTRY_LOWER_YAW_MASTER_ID 0u
#define SENTRY_LOWER_YAW_DM_ENABLED 1
#define SENTRY_LOWER_YAW_SPIN_RAD_S 0.5f
#define SENTRY_LOWER_YAW_MOTOR_REVERSE MOTOR_DIRECTION_NORMAL
// Pitch wiring and travel have not been verified on this robot.
#define SENTRY_AIM_PITCH_ENABLED 0
#define SENTRY_AIM_PITCH_CAN_BUS hfdcan2
#define SENTRY_AIM_PITCH_MOTOR_ID 7
#define SENTRY_AIM_YAW_TRAVEL_DEG 10.0f
#define SENTRY_AIM_PITCH_TRAVEL_DEG 5.0f
#define SENTRY_AIM_COMMAND_TIMEOUT_MS 300u
#define SENTRY_STEER_KEYBOARD_LOG_PERIOD_MS 500u

// Both motor types use the same split: IDs 1/4 on CAN1 and IDs 2/3 on CAN2.
#define CHASSIS_MOTOR_CAN_HANDLE(motor_id) \
    (((motor_id) == 2 || (motor_id) == 3) ? &hfdcan2 : &hfdcan1)

// Four-module swerve order: LF, RF, RB, LB.
#define CHASSIS_DRIVE_MOTOR_LF_ID 1
#define CHASSIS_DRIVE_MOTOR_RF_ID 2
#define CHASSIS_DRIVE_MOTOR_RB_ID 3
#define CHASSIS_DRIVE_MOTOR_LB_ID 4

#define CHASSIS_STEER_MOTOR_LF_ID 1
#define CHASSIS_STEER_MOTOR_RF_ID 2
#define CHASSIS_STEER_MOTOR_RB_ID 3
#define CHASSIS_STEER_MOTOR_LB_ID 4

// Measured mechanical zero positions (GM6020 single-turn ECD, 0..8191).
// LF/RF confirmed correct live on the robot (within 2 ticks). RB/LB were each
// off by ~118 degrees in opposite directions (old RB=3407 read live at 3414;
// old LB=6106 read live at 6091) -- recaptured directly from the live raw
// steering encoder while the browser held the chassis pointed forward,
// not estimated from a verbal report.
#define CHASSIS_STEER_MOTOR_LF_ZERO_ECD 5484u
#define CHASSIS_STEER_MOTOR_RF_ZERO_ECD 702u
#define CHASSIS_STEER_MOTOR_RB_ZERO_ECD 3414u
#define CHASSIS_STEER_MOTOR_LB_ZERO_ECD 6091u

// USB keyboard steering test settings. ID order is LF, RF, left-rear, right-rear.
#define CHASSIS_STEER_KEYBOARD_ONLINE_STABLE_MS 200u
#define CHASSIS_STEER_KEYBOARD_COMMAND_TIMEOUT_MS 500u
#define CHASSIS_STEER_KEYBOARD_TOLERANCE_ECD 64
#define CHASSIS_STEER_KEYBOARD_ANGLE_PID_KP 24.0f
#define CHASSIS_STEER_KEYBOARD_ANGLE_PID_KI 0.18f
#define CHASSIS_STEER_KEYBOARD_ANGLE_PID_KD 0.0f
#define CHASSIS_STEER_KEYBOARD_ANGLE_PID_DEADBAND 0.50f
#define CHASSIS_STEER_KEYBOARD_ANGLE_PID_INT_LIMIT 400.0f
#define CHASSIS_STEER_KEYBOARD_ANGLE_PID_OUTPUT_LPF_RC 0.010f
#define CHASSIS_STEER_KEYBOARD_MAX_SPEED_APS 1080.0f
#define CHASSIS_STEER_KEYBOARD_SPEED_PID_KP 100.0f
#define CHASSIS_STEER_KEYBOARD_SPEED_PID_KI 35.0f
#define CHASSIS_STEER_KEYBOARD_SPEED_PID_KD 0.0f
#define CHASSIS_STEER_KEYBOARD_SPEED_PID_DEADBAND 5.0f
#define CHASSIS_STEER_KEYBOARD_SPEED_PID_INT_LIMIT 2000.0f
#define CHASSIS_STEER_KEYBOARD_SPEED_PID_OUTPUT_LPF_RC 0.008f
#define CHASSIS_STEER_KEYBOARD_MAX_CURRENT 15000.0f
#define CHASSIS_STEER_KEYBOARD_LF_DIRECTION_SIGN 1
#define CHASSIS_STEER_KEYBOARD_RF_DIRECTION_SIGN 1
#define CHASSIS_STEER_KEYBOARD_LR_DIRECTION_SIGN 1
#define CHASSIS_STEER_KEYBOARD_RR_DIRECTION_SIGN 1
#define CHASSIS_STEER_KEYBOARD_DRIVE_TARGET_RPM 1000.0f

// Nav2 command and telemetry settings. ROS autonomous navigation remains
// limited separately; these limits also allow faster high-priority teleop.
#define CHASSIS_STEER_NAV2_COMMAND_TIMEOUT_MS 300u
#define CHASSIS_STEER_NAV2_TELEMETRY_PERIOD_MS 50u
#define CHASSIS_STEER_NAV2_TELEMETRY_TX_TIMEOUT_MS 250u
#define CHASSIS_STEER_NAV2_TELEMETRY_RECOVERY_LOG_PERIOD_MS 2000u
#define CHASSIS_STEER_NAV2_LOG_PERIOD_MS 500u
#define CHASSIS_STEER_NAV2_MAX_TRANSLATION_MM_S 800.0f
#define CHASSIS_STEER_NAV2_MAX_ANGULAR_MRAD_S 2400.0f
#define CHASSIS_STEER_NAV2_MAX_DRIVE_RPM 2500.0f
#define CHASSIS_STEER_NAV2_MAX_TRANSLATION_ACCEL_MM_S2 6000.0f
#define CHASSIS_STEER_NAV2_MAX_ANGULAR_ACCEL_MRAD_S2 18000.0f
#define CHASSIS_STEER_NAV2_STOP_TRANSLATION_MM_S 5.0f
#define CHASSIS_STEER_NAV2_STOP_ANGULAR_MRAD_S 10.0f
#define CHASSIS_STEER_NAV2_DIRECTION_HOLD_MM_S 15.0f
// Restore the proven steering-loop structure from the original keyboard
// controller. The 2026-08-12 dashboard capture showed that Kp=70 still drove
// the GM6020 output into a sustained +/-15000 limit cycle.
#define CHASSIS_STEER_NAV2_ANGLE_PID_KP 20.0f
#define CHASSIS_STEER_NAV2_ANGLE_PID_KI 0.0f
#define CHASSIS_STEER_NAV2_ANGLE_PID_KD 0.0f
#define CHASSIS_STEER_NAV2_ANGLE_PID_DEADBAND 1.0f
#define CHASSIS_STEER_NAV2_MAX_STEER_SPEED_APS 720.0f
#define CHASSIS_STEER_NAV2_SPEED_PID_KP 40.0f
#define CHASSIS_STEER_NAV2_SPEED_PID_KI 0.0f
#define CHASSIS_STEER_NAV2_SPEED_PID_KD 0.0f
#define CHASSIS_STEER_NAV2_SPEED_PID_DEADBAND 15.0f
#define CHASSIS_STEER_NAV2_SPEED_PID_OUTPUT_LPF_RC 0.010f
// Preserve the stable 7000-current limit near the target, but allow extra
// torque while a module is still far from its requested direction. The
// linear transition starts at 11.25 degrees and reaches full boost at 22.5
// degrees; full drive output is reached only inside the low-current region.
#define CHASSIS_STEER_NAV2_MAX_STEER_CURRENT 7000.0f
#define CHASSIS_STEER_NAV2_BOOST_MAX_STEER_CURRENT 11000.0f
#define CHASSIS_STEER_NAV2_BOOST_START_ECD 256
#define CHASSIS_STEER_NAV2_BOOST_FULL_ECD 512

// The 10 ms output filter introduced a 20-30 Hz drive limit cycle in the
// 2026-08-12 lifted-wheel test. Use a lower-gain, unfiltered P loop and ignore
// feedback noise within 120 RPM of the requested speed.
#define CHASSIS_STEER_NAV2_DRIVE_SPEED_PID_KP 6.0f
#define CHASSIS_STEER_NAV2_DRIVE_SPEED_PID_KI 0.0f
#define CHASSIS_STEER_NAV2_DRIVE_SPEED_PID_KD 0.0f
#define CHASSIS_STEER_NAV2_DRIVE_SPEED_PID_DEADBAND 120.0f
#define CHASSIS_STEER_NAV2_DRIVE_SPEED_PID_MAX_OUT 12000.0f
// Actively brake only after motion, then stop the drive motors so zero-speed
// feedback noise cannot excite a lifted wheel indefinitely.
#define CHASSIS_STEER_NAV2_DRIVE_BRAKE_MAX_MS 200u
#define CHASSIS_STEER_NAV2_DRIVE_BRAKE_SETTLE_MS 30u
#define CHASSIS_STEER_NAV2_DRIVE_BRAKE_SETTLE_RPM 100

// Preserve the four requested wheel-speed ratios while steering catches up.
// Drive remains zero above 16.875 degrees of worst-module error and ramps to
// full output at 5.625 degrees instead of enabling each wheel independently.
#define CHASSIS_STEER_NAV2_DRIVE_START_ALIGNMENT_ECD 384
#define CHASSIS_STEER_NAV2_DRIVE_FULL_ALIGNMENT_ECD 128
// Keep the selected 180-degree solution stable in a +/-2.8125 degree band.
#define CHASSIS_STEER_NAV2_FLIP_ENTER_ECD 2112
#define CHASSIS_STEER_NAV2_FLIP_EXIT_ECD 1984

#undef CHASSIS_MOTOR_LF_ID
#undef CHASSIS_MOTOR_RF_ID
#undef CHASSIS_MOTOR_RB_ID
#undef CHASSIS_MOTOR_LB_ID
#define CHASSIS_MOTOR_LF_ID CHASSIS_DRIVE_MOTOR_LF_ID
#define CHASSIS_MOTOR_RF_ID CHASSIS_DRIVE_MOTOR_RF_ID
#define CHASSIS_MOTOR_RB_ID CHASSIS_DRIVE_MOTOR_RB_ID
#define CHASSIS_MOTOR_LB_ID CHASSIS_DRIVE_MOTOR_LB_ID

// IDs 1 and 4 are mounted opposite to IDs 2 and 3 on the swerve modules.
#undef CHASSIS_MOTOR_LF_REVERSE
#undef CHASSIS_MOTOR_LB_REVERSE
#define CHASSIS_MOTOR_LF_REVERSE MOTOR_DIRECTION_REVERSE
#define CHASSIS_MOTOR_LB_REVERSE MOTOR_DIRECTION_REVERSE

#undef CHASSIS_SPEED_PID_KP
#undef CHASSIS_SPEED_PID_KI
#undef CHASSIS_SPEED_PID_KD
#undef CHASSIS_SPEED_PID_INT_LIMIT
#undef CHASSIS_SPEED_PID_MAX_OUT
#define CHASSIS_SPEED_PID_KP 12.0f
#define CHASSIS_SPEED_PID_KI 0.0f
#define CHASSIS_SPEED_PID_KD 0.0f
#define CHASSIS_SPEED_PID_INT_LIMIT 3000.0f
#define CHASSIS_SPEED_PID_MAX_OUT 12000.0f

#undef CHASSIS_CURRENT_PID_KP
#undef CHASSIS_CURRENT_PID_KI
#undef CHASSIS_CURRENT_PID_KD
#undef CHASSIS_CURRENT_PID_INT_LIMIT
#undef CHASSIS_CURRENT_PID_MAX_OUT
#define CHASSIS_CURRENT_PID_KP 0.65f
#define CHASSIS_CURRENT_PID_KI 0.0f
#define CHASSIS_CURRENT_PID_KD 0.0f
#define CHASSIS_CURRENT_PID_INT_LIMIT 3000.0f
#define CHASSIS_CURRENT_PID_MAX_OUT 15000.0f

#endif // ROBOT_SENTRY_SWERVE_CONFIG_H
