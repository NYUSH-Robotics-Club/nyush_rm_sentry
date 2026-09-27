#include "sentry_steer_nav2.h"

#include "FreeRTOS.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usb.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "general_def.h"
#include "led.h"
#include "message_center.h"
#include "robot_def.h"
#include "task.h"

#include <math.h>
#include <stdlib.h>

#if defined(ROBOT_TYPE_sentry_swerve) && SENTRY_STEER_NAV2_CONTROL

typedef enum
{
    STEER_NAV2_LF = 0,
    STEER_NAV2_RF,
    STEER_NAV2_LR,
    STEER_NAV2_RR,
} SteerNav2Index_e;

typedef enum
{
    STEER_NAV2_CONTROL_DISABLED = 0,
    STEER_NAV2_CONTROL_HOLD,
    STEER_NAV2_CONTROL_MOTION,
} SteerNav2ControlMode_e;

#define DJI_ECD_FULL_RANGE 8192
#define DJI_ECD_HALF_RANGE 4096
#define DJI_ECD_QUARTER_RANGE 2048
#define STEER_ALL_ONLINE_BITMAP 0x0fu
#define NAV2_PROTOCOL_MAGIC_0 0xa5u
#define NAV2_PROTOCOL_MAGIC_1 0x5au
#define NAV2_PROTOCOL_VERSION 1u
#define NAV2_COMMAND_TYPE 0x01u
#define AIM_COMMAND_TYPE 0x02u
#define NAV2_TELEMETRY_TYPE 0x81u
#define NAV2_COMMAND_FRAME_SIZE 15u
#define NAV2_TELEMETRY_FRAME_SIZE 29u
#define NAV2_COMMAND_ENABLE_FLAG 0x01u
#define NAV2_STATUS_CONTROL_ENABLED 0x01u
#define NAV2_STATUS_COMMAND_FRESH 0x02u

#if CHASSIS_STEER_NAV2_FLIP_EXIT_ECD >= DJI_ECD_QUARTER_RANGE || \
    CHASSIS_STEER_NAV2_FLIP_ENTER_ECD <= DJI_ECD_QUARTER_RANGE
#error "Swerve flip hysteresis must straddle the 90-degree encoder position"
#endif

#if CHASSIS_STEER_NAV2_DRIVE_FULL_ALIGNMENT_ECD >= \
        CHASSIS_STEER_NAV2_DRIVE_START_ALIGNMENT_ECD || \
    CHASSIS_STEER_NAV2_DRIVE_START_ALIGNMENT_ECD >= DJI_ECD_QUARTER_RANGE
#error "Swerve drive alignment thresholds must satisfy 0 < full < start < 90 degrees"
#endif

#if CHASSIS_STEER_NAV2_BOOST_START_ECD <= 0 || \
    CHASSIS_STEER_NAV2_BOOST_START_ECD >= CHASSIS_STEER_NAV2_BOOST_FULL_ECD || \
    CHASSIS_STEER_NAV2_BOOST_FULL_ECD >= DJI_ECD_QUARTER_RANGE
#error "Swerve steering boost thresholds must satisfy 0 < start < full < 90 degrees"
#endif

#if CHASSIS_STEER_NAV2_DRIVE_BRAKE_MAX_MS == 0u || \
    CHASSIS_STEER_NAV2_DRIVE_BRAKE_SETTLE_MS == 0u || \
    CHASSIS_STEER_NAV2_DRIVE_BRAKE_SETTLE_MS >= \
        CHASSIS_STEER_NAV2_DRIVE_BRAKE_MAX_MS || \
    CHASSIS_STEER_NAV2_DRIVE_BRAKE_SETTLE_RPM <= 0
#error "Swerve drive braking requires positive settle thresholds below the timeout"
#endif

#if CHASSIS_STEER_NAV2_TELEMETRY_TX_TIMEOUT_MS <= \
    CHASSIS_STEER_NAV2_TELEMETRY_PERIOD_MS
#error "Swerve telemetry TX timeout must exceed its normal publish period"
#endif

static DJIMotorInstance *steer_motors[SENTRY_STEER_NAV2_MOTOR_COUNT];
static DJIMotorInstance *drive_motors[SENTRY_STEER_NAV2_MOTOR_COUNT];
static DJIMotorInstance *aim_yaw_motor;
static DMMotorInstance *lower_yaw_motor;
static uint8_t lower_yaw_spinning;
static uint8_t lower_yaw_last_online = 2u;
static uint8_t lower_yaw_last_request = 2u;
#if SENTRY_AIM_PITCH_ENABLED
static DJIMotorInstance *aim_pitch_motor;
#endif
static volatile int16_t rx_aim_yaw_mrad_s;
static volatile int16_t rx_aim_pitch_mrad_s;
static volatile uint8_t rx_aim_enable;
static volatile uint8_t rx_lower_spin_request;
static volatile uint16_t rx_aim_sequence;
static uint16_t handled_aim_sequence;
static uint32_t last_aim_command_ms;
static uint32_t last_aim_update_ms;
static float aim_yaw_origin_deg;
static float aim_yaw_target_deg;
static uint8_t aim_yaw_origin_valid;
static uint8_t aim_yaw_active;
static uint8_t aim_yaw_last_online = 2u;
#if SENTRY_AIM_PITCH_ENABLED
static float aim_pitch_origin_deg;
static float aim_pitch_target_deg;
static uint8_t aim_pitch_origin_valid;
static uint8_t aim_pitch_active;
#endif
static float target_total_angle_deg[SENTRY_STEER_NAV2_MOTOR_COUNT];
static float drive_target_rpm[SENTRY_STEER_NAV2_MOTOR_COUNT];
static float applied_drive_target_rpm[SENTRY_STEER_NAV2_MOTOR_COUNT];
static uint8_t *usb_rx_buffer;
static uint8_t rx_frame[NAV2_COMMAND_FRAME_SIZE];
static uint8_t rx_frame_index;
static volatile int16_t rx_vx_mm_s;
static volatile int16_t rx_vy_mm_s;
static volatile int16_t rx_wz_mrad_s;
static volatile uint8_t rx_enable;
static volatile uint16_t rx_sequence;
static uint16_t handled_rx_sequence;
static uint16_t telemetry_sequence;
static uint32_t last_command_ms;
static uint32_t online_stable_start_ms;
static uint32_t last_telemetry_ms;
static uint32_t telemetry_tx_started_ms;
static uint32_t telemetry_busy_started_ms;
static uint32_t telemetry_recovery_count;
static uint32_t last_telemetry_recovery_log_ms;
static uint32_t last_log_ms;
static uint32_t last_slew_ms;
static uint32_t drive_brake_start_ms;
static uint32_t drive_brake_settle_start_ms;
static uint8_t online_timer_started;
static uint8_t command_seen;
static volatile uint8_t telemetry_tx_pending;
static uint8_t drive_flip_bitmap;
static uint8_t steer_target_valid_bitmap;
static uint8_t drive_brake_active;
static SteerNav2ControlMode_e control_mode;
static uint16_t drive_common_scale_permille;
static float slewed_vx_mm_s;
static float slewed_vy_mm_s;
static float slewed_wz_mrad_s;
static Publisher_t *dashboard_pub;
static SentrySteerNav2Telemetry_s dashboard_telemetry;
static float ClampFloat(float value, float min_value, float max_value);

volatile SentrySteerNav2State_s sentry_steer_nav2_state;

/* Protocol and kinematics order: LF, RF, LR (ID 4), RR (ID 3). */
static const uint8_t steer_motor_ids[SENTRY_STEER_NAV2_MOTOR_COUNT] = {
    [STEER_NAV2_LF] = CHASSIS_STEER_MOTOR_LF_ID,
    [STEER_NAV2_RF] = CHASSIS_STEER_MOTOR_RF_ID,
    [STEER_NAV2_LR] = CHASSIS_STEER_MOTOR_LB_ID,
    [STEER_NAV2_RR] = CHASSIS_STEER_MOTOR_RB_ID,
};

static const uint8_t drive_motor_ids[SENTRY_STEER_NAV2_MOTOR_COUNT] = {
    [STEER_NAV2_LF] = CHASSIS_DRIVE_MOTOR_LF_ID,
    [STEER_NAV2_RF] = CHASSIS_DRIVE_MOTOR_RF_ID,
    [STEER_NAV2_LR] = CHASSIS_DRIVE_MOTOR_LB_ID,
    [STEER_NAV2_RR] = CHASSIS_DRIVE_MOTOR_RB_ID,
};

static const Motor_Reverse_Flag_e drive_motor_directions[SENTRY_STEER_NAV2_MOTOR_COUNT] = {
    [STEER_NAV2_LF] = CHASSIS_MOTOR_LF_REVERSE,
    [STEER_NAV2_RF] = CHASSIS_MOTOR_RF_REVERSE,
    [STEER_NAV2_LR] = CHASSIS_MOTOR_LB_REVERSE,
    [STEER_NAV2_RR] = CHASSIS_MOTOR_RB_REVERSE,
};

static const uint16_t steer_zero_ecd[SENTRY_STEER_NAV2_MOTOR_COUNT] = {
    [STEER_NAV2_LF] = CHASSIS_STEER_MOTOR_LF_ZERO_ECD,
    [STEER_NAV2_RF] = CHASSIS_STEER_MOTOR_RF_ZERO_ECD,
    [STEER_NAV2_LR] = CHASSIS_STEER_MOTOR_LB_ZERO_ECD,
    [STEER_NAV2_RR] = CHASSIS_STEER_MOTOR_RB_ZERO_ECD,
};

static const int8_t steer_direction_sign[SENTRY_STEER_NAV2_MOTOR_COUNT] = {
    [STEER_NAV2_LF] = CHASSIS_STEER_KEYBOARD_LF_DIRECTION_SIGN,
    [STEER_NAV2_RF] = CHASSIS_STEER_KEYBOARD_RF_DIRECTION_SIGN,
    [STEER_NAV2_LR] = CHASSIS_STEER_KEYBOARD_LR_DIRECTION_SIGN,
    [STEER_NAV2_RR] = CHASSIS_STEER_KEYBOARD_RR_DIRECTION_SIGN,
};

static const float module_x_mm[SENTRY_STEER_NAV2_MOTOR_COUNT] = {
    [STEER_NAV2_LF] = 0.5f * WHEEL_BASE,
    [STEER_NAV2_RF] = 0.5f * WHEEL_BASE,
    [STEER_NAV2_LR] = -0.5f * WHEEL_BASE,
    [STEER_NAV2_RR] = -0.5f * WHEEL_BASE,
};

/* ROS uses +y to the left. */
static const float module_y_mm[SENTRY_STEER_NAV2_MOTOR_COUNT] = {
    [STEER_NAV2_LF] = 0.5f * TRACK_WIDTH,
    [STEER_NAV2_RF] = -0.5f * TRACK_WIDTH,
    [STEER_NAV2_LR] = 0.5f * TRACK_WIDTH,
    [STEER_NAV2_RR] = -0.5f * TRACK_WIDTH,
};

static int16_t ReadI16Le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void WriteU16Le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xffu);
    data[1] = (uint8_t)(value >> 8);
}

static void WriteU32Le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xffu);
    data[1] = (uint8_t)((value >> 8) & 0xffu);
    data[2] = (uint8_t)((value >> 16) & 0xffu);
    data[3] = (uint8_t)(value >> 24);
}

static uint16_t ProtocolCrc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xffffu;

    for (uint16_t i = 0u; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0u; bit < 8u; ++bit)
            crc = (crc & 0x8000u) != 0u ? (uint16_t)((crc << 1) ^ 0x1021u)
                                        : (uint16_t)(crc << 1);
    }

    return crc;
}

static int16_t WrapEcdDelta(int32_t delta_ecd)
{
    if (delta_ecd > DJI_ECD_HALF_RANGE)
        delta_ecd -= DJI_ECD_FULL_RANGE;
    else if (delta_ecd < -DJI_ECD_HALF_RANGE)
        delta_ecd += DJI_ECD_FULL_RANGE;

    return (int16_t)delta_ecd;
}

static uint16_t WrapEcdTarget(int32_t target_ecd)
{
    target_ecd %= DJI_ECD_FULL_RANGE;
    if (target_ecd < 0)
        target_ecd += DJI_ECD_FULL_RANGE;
    return (uint16_t)target_ecd;
}

static uint8_t MotorOnlineBitmap(DJIMotorInstance *const motors[SENTRY_STEER_NAV2_MOTOR_COUNT])
{
    uint8_t bitmap = 0u;

    for (uint8_t i = 0u; i < SENTRY_STEER_NAV2_MOTOR_COUNT; ++i)
    {
        if (DaemonIsOnline(motors[i]->daemon))
            bitmap |= (uint8_t)(1u << i);
    }

    return bitmap;
}

static int16_t MotorRpm(const DJIMotorInstance *motor)
{
    float rpm = motor->measure.speed_aps / RPM_2_ANGLE_PER_SEC;
    if (rpm > 32767.0f)
        rpm = 32767.0f;
    else if (rpm < -32768.0f)
        rpm = -32768.0f;
    return (int16_t)lroundf(rpm);
}

static int LogicalDriveMotorRpm(uint8_t index)
{
    int rpm = MotorRpm(drive_motors[index]);

    if (drive_motor_directions[index] == MOTOR_DIRECTION_REVERSE)
        rpm = -rpm;
    return rpm;
}

static int16_t ClampI16(float value)
{
    if (value > 32767.0f)
        value = 32767.0f;
    else if (value < -32768.0f)
        value = -32768.0f;
    return (int16_t)lroundf(value);
}

static int16_t LogicalDriveValue(uint8_t index, float value)
{
    if (drive_motor_directions[index] == MOTOR_DIRECTION_REVERSE)
        value = -value;
    return ClampI16(value);
}

static void PublishDashboardTelemetry(uint8_t command_fresh)
{
    dashboard_telemetry.command_vx_mm_s = sentry_steer_nav2_state.command_vx_mm_s;
    dashboard_telemetry.command_vy_mm_s = sentry_steer_nav2_state.command_vy_mm_s;
    dashboard_telemetry.command_wz_mrad_s = sentry_steer_nav2_state.command_wz_mrad_s;
    dashboard_telemetry.drive_scale_permille = drive_common_scale_permille;
    dashboard_telemetry.steer_online_bitmap = sentry_steer_nav2_state.steer_online_bitmap;
    dashboard_telemetry.drive_online_bitmap = sentry_steer_nav2_state.drive_online_bitmap;
    dashboard_telemetry.aligned_bitmap = sentry_steer_nav2_state.aligned_bitmap;
    dashboard_telemetry.flip_bitmap = drive_flip_bitmap;
    dashboard_telemetry.control_enabled = sentry_steer_nav2_state.control_enabled;
    dashboard_telemetry.command_fresh = command_fresh;

    for (uint8_t i = 0u; i < SENTRY_STEER_NAV2_MOTOR_COUNT; ++i)
    {
        const int16_t error_ecd = WrapEcdDelta(
            (int32_t)sentry_steer_nav2_state.target_ecd[i] -
            (int32_t)steer_motors[i]->measure.ecd);
        const uint8_t steer_enabled = steer_motors[i]->stop_flag == MOTOR_ENALBED;
        const uint8_t drive_enabled = drive_motors[i]->stop_flag == MOTOR_ENALBED;

        dashboard_telemetry.drive_target_rpm[i] =
            (float)sentry_steer_nav2_state.drive_target_rpm[i];
        dashboard_telemetry.drive_actual_rpm[i] = (float)LogicalDriveMotorRpm(i);
        dashboard_telemetry.steer_target_ecd[i] = sentry_steer_nav2_state.target_ecd[i];
        dashboard_telemetry.steer_actual_ecd[i] = steer_motors[i]->measure.ecd;
        dashboard_telemetry.steer_error_ecd[i] = error_ecd;
        dashboard_telemetry.steer_target_angle_deg[i] = target_total_angle_deg[i];
        dashboard_telemetry.steer_actual_angle_deg[i] = steer_motors[i]->measure.total_angle;
        dashboard_telemetry.steer_target_speed_deg_s[i] =
            steer_enabled ? steer_motors[i]->motor_controller.angle_PID.Output : 0.0f;
        dashboard_telemetry.steer_actual_speed_deg_s[i] = steer_motors[i]->measure.speed_aps;
        dashboard_telemetry.steer_output[i] =
            steer_enabled ? ClampI16(steer_motors[i]->motor_controller.pre_current_ref) : 0;
        dashboard_telemetry.steer_current[i] = steer_motors[i]->measure.real_current;
        dashboard_telemetry.drive_output[i] =
            drive_enabled
                ? LogicalDriveValue(i, drive_motors[i]->motor_controller.current_PID.Output)
                : 0;
        dashboard_telemetry.drive_current[i] =
            LogicalDriveValue(i, (float)drive_motors[i]->measure.real_current);
    }

    PubPushMessage(dashboard_pub, &dashboard_telemetry);
}

static void ResetPidState(PIDInstance *pid)
{
    if (pid == NULL)
        return;

    PID_Init_Config_s config = {
        .Kp = pid->Kp,
        .Ki = pid->Ki,
        .Kd = pid->Kd,
        .MaxOut = pid->MaxOut,
        .DeadBand = pid->DeadBand,
        .Improve = pid->Improve,
        .IntegralLimit = pid->IntegralLimit,
        .CoefA = pid->CoefA,
        .CoefB = pid->CoefB,
        .Output_LPF_RC = pid->Output_LPF_RC,
        .Derivative_LPF_RC = pid->Derivative_LPF_RC,
    };

    PIDInit(pid, &config);
}

static void ResetMotorController(DJIMotorInstance *motor)
{
    if (motor == NULL)
        return;

    ResetPidState(&motor->motor_controller.angle_PID);
    ResetPidState(&motor->motor_controller.speed_PID);
    ResetPidState(&motor->motor_controller.current_PID);
    motor->motor_controller.pid_ref = 0.0f;
    motor->motor_controller.pre_current_ref = 0.0f;
}

static void ResetAllMotorControllers(void)
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0u; i < SENTRY_STEER_NAV2_MOTOR_COUNT; ++i)
    {
        ResetMotorController(steer_motors[i]);
        ResetMotorController(drive_motors[i]);
    }
    taskEXIT_CRITICAL();
}

static void PrepareClosedLoop(void)
{
    if (control_mode != STEER_NAV2_CONTROL_DISABLED)
        return;

    ResetAllMotorControllers();
}

static void CancelDriveBrake(void)
{
    drive_brake_active = 0u;
    drive_brake_start_ms = 0u;
    drive_brake_settle_start_ms = 0u;
}

static void DisableAllMotors(void)
{
    for (uint8_t i = 0u; i < SENTRY_STEER_NAV2_MOTOR_COUNT; ++i)
    {
        steer_motors[i]->motor_controller.speed_PID.MaxOut =
            CHASSIS_STEER_NAV2_MAX_STEER_CURRENT;
        DJIMotorSetRef(steer_motors[i], steer_motors[i]->measure.total_angle);
        DJIMotorStop(steer_motors[i]);
        DJIMotorSetRef(drive_motors[i], 0.0f);
        DJIMotorStop(drive_motors[i]);
        drive_target_rpm[i] = 0.0f;
        applied_drive_target_rpm[i] = 0.0f;
        target_total_angle_deg[i] = steer_motors[i]->measure.total_angle;
        sentry_steer_nav2_state.target_ecd[i] = steer_motors[i]->measure.ecd;
        sentry_steer_nav2_state.drive_target_rpm[i] = 0;
    }
    if (control_mode != STEER_NAV2_CONTROL_DISABLED)
        ResetAllMotorControllers();

    slewed_vx_mm_s = 0.0f;
    slewed_vy_mm_s = 0.0f;
    slewed_wz_mrad_s = 0.0f;
    last_slew_ms = 0u;
    drive_flip_bitmap = 0u;
    steer_target_valid_bitmap = 0u;
    CancelDriveBrake();
    drive_common_scale_permille = 0u;
    sentry_steer_nav2_state.command_vx_mm_s = 0;
    sentry_steer_nav2_state.command_vy_mm_s = 0;
    sentry_steer_nav2_state.command_wz_mrad_s = 0;
    sentry_steer_nav2_state.control_enabled = 0u;
    control_mode = STEER_NAV2_CONTROL_DISABLED;
}

static uint8_t DriveMotorsSettled(void)
{
    for (uint8_t i = 0u; i < SENTRY_STEER_NAV2_MOTOR_COUNT; ++i)
    {
        if (abs(MotorRpm(drive_motors[i])) >
            CHASSIS_STEER_NAV2_DRIVE_BRAKE_SETTLE_RPM)
            return 0u;
    }

    return 1u;
}

static void UpdateDriveBrake(uint32_t now_ms)
{
    if (!drive_brake_active)
        return;

    if ((now_ms - drive_brake_start_ms) >=
        CHASSIS_STEER_NAV2_DRIVE_BRAKE_MAX_MS)
    {
        CancelDriveBrake();
        return;
    }

    if (!DriveMotorsSettled())
    {
        drive_brake_settle_start_ms = 0u;
        return;
    }

    if (drive_brake_settle_start_ms == 0u)
    {
        drive_brake_settle_start_ms = now_ms;
        return;
    }

    if ((now_ms - drive_brake_settle_start_ms) >=
        CHASSIS_STEER_NAV2_DRIVE_BRAKE_SETTLE_MS)
        CancelDriveBrake();
}

static void HoldZeroMotion(uint32_t now_ms)
{
    uint8_t aligned_bitmap = 0u;

    if (control_mode == STEER_NAV2_CONTROL_MOTION)
    {
        drive_brake_active = 1u;
        drive_brake_start_ms = now_ms;
        drive_brake_settle_start_ms = 0u;
    }
    else if (control_mode == STEER_NAV2_CONTROL_DISABLED)
    {
        CancelDriveBrake();
    }

    PrepareClosedLoop();
    UpdateDriveBrake(now_ms);

    for (uint8_t i = 0u; i < SENTRY_STEER_NAV2_MOTOR_COUNT; ++i)
    {
        const uint8_t wheel_mask = (uint8_t)(1u << i);

        if ((steer_target_valid_bitmap & wheel_mask) == 0u)
        {
            target_total_angle_deg[i] = steer_motors[i]->measure.total_angle;
            sentry_steer_nav2_state.target_ecd[i] = steer_motors[i]->measure.ecd;
            steer_target_valid_bitmap |= wheel_mask;
        }

        steer_motors[i]->motor_controller.speed_PID.MaxOut =
            CHASSIS_STEER_NAV2_MAX_STEER_CURRENT;
        DJIMotorSetRef(steer_motors[i], target_total_angle_deg[i]);
        DJIMotorEnable(steer_motors[i]);
        DJIMotorSetRef(drive_motors[i], 0.0f);
        if (drive_brake_active)
            DJIMotorEnable(drive_motors[i]);
        else
            DJIMotorStop(drive_motors[i]);
        drive_target_rpm[i] = 0.0f;
        applied_drive_target_rpm[i] = 0.0f;
        sentry_steer_nav2_state.drive_target_rpm[i] = 0;

        const int16_t error_ecd = WrapEcdDelta(
            (int32_t)sentry_steer_nav2_state.target_ecd[i] -
            (int32_t)steer_motors[i]->measure.ecd);
        sentry_steer_nav2_state.current_ecd[i] = steer_motors[i]->measure.ecd;
        sentry_steer_nav2_state.error_ecd[i] = error_ecd;
        if (abs(error_ecd) <= CHASSIS_STEER_NAV2_DRIVE_FULL_ALIGNMENT_ECD)
            aligned_bitmap |= wheel_mask;
    }

    slewed_vx_mm_s = 0.0f;
    slewed_vy_mm_s = 0.0f;
    slewed_wz_mrad_s = 0.0f;
    last_slew_ms = 0u;
    drive_common_scale_permille = 0u;
    sentry_steer_nav2_state.command_vx_mm_s = 0;
    sentry_steer_nav2_state.command_vy_mm_s = 0;
    sentry_steer_nav2_state.command_wz_mrad_s = 0;
    sentry_steer_nav2_state.aligned_bitmap = aligned_bitmap;
    sentry_steer_nav2_state.control_enabled = 1u;
    control_mode = STEER_NAV2_CONTROL_HOLD;
}

static void DecodeCommandFrame(const uint8_t *frame)
{
    const uint16_t received_crc = (uint16_t)frame[NAV2_COMMAND_FRAME_SIZE - 2u] |
                                  ((uint16_t)frame[NAV2_COMMAND_FRAME_SIZE - 1u] << 8);

    if (frame[2] != NAV2_PROTOCOL_VERSION ||
        ProtocolCrc16(frame, NAV2_COMMAND_FRAME_SIZE - 2u) != received_crc)
        return;

    if (frame[3] == AIM_COMMAND_TYPE)
    {
        rx_aim_yaw_mrad_s = ReadI16Le(&frame[6]);
        rx_aim_pitch_mrad_s = ReadI16Le(&frame[8]);
        rx_aim_enable = frame[12] & NAV2_COMMAND_ENABLE_FLAG;
        rx_lower_spin_request = frame[12] & 0x02u;
        rx_aim_sequence = (uint16_t)(rx_aim_sequence + 1u);
        return;
    }
    if (frame[3] != NAV2_COMMAND_TYPE)
        return;

    rx_vx_mm_s = ReadI16Le(&frame[6]);
    rx_vy_mm_s = ReadI16Le(&frame[8]);
    rx_wz_mrad_s = ReadI16Le(&frame[10]);
    rx_enable = frame[12] & NAV2_COMMAND_ENABLE_FLAG;
    rx_sequence = (uint16_t)(rx_sequence + 1u);
}

static void UpdateAimMotor(DJIMotorInstance *motor, float *origin_deg,
                           float *target_deg, uint8_t *origin_valid, uint8_t *active,
                           int16_t rate_mrad_s, float travel_deg, float dt_s,
                           uint8_t enabled)
{
    if (motor == NULL || motor->feed_cnt == 0u ||
        !DaemonIsOnline(motor->daemon))
    {
        if (motor != NULL)
            DJIMotorStop(motor);
        *active = 0u;
        return;
    }
    if (!*origin_valid)
    {
        *origin_deg = motor->measure.total_angle;
        *target_deg = *origin_deg;
        *origin_valid = 1u;
    }
    if (!enabled)
    {
        DJIMotorStop(motor);
        *active = 0u;
        return;
    }
    if (!*active)
    {
        const float angle = motor->measure.total_angle;
        if (angle < *origin_deg - travel_deg || angle > *origin_deg + travel_deg)
        {
            DJIMotorStop(motor);
            return;
        }
        taskENTER_CRITICAL();
        ResetMotorController(motor);
        taskEXIT_CRITICAL();
        *target_deg = angle;
        *active = 1u;
    }
    const float rate = ClampFloat((float)rate_mrad_s, -300.0f, 300.0f);
    *target_deg = ClampFloat(*target_deg + rate * 0.001f * RAD_2_DEGREE * dt_s,
                             *origin_deg - travel_deg, *origin_deg + travel_deg);
    DJIMotorSetRef(motor, *target_deg);
    DJIMotorEnable(motor);
}

static void UpdateManualAim(uint32_t now_ms)
{
    const uint8_t yaw_online = aim_yaw_motor != NULL &&
        aim_yaw_motor->feed_cnt != 0u && DaemonIsOnline(aim_yaw_motor->daemon);
    if (yaw_online != aim_yaw_last_online)
    {
        aim_yaw_last_online = yaw_online;
        if (yaw_online)
            LOGINFO("[steer-nav2] aiming yaw online");
        else
            LOGWARNING("[steer-nav2] aiming yaw offline");
    }
    if (handled_aim_sequence != rx_aim_sequence)
    {
        handled_aim_sequence = rx_aim_sequence;
        last_aim_command_ms = now_ms;
    }
    const uint8_t enabled = rx_aim_enable &&
        (now_ms - last_aim_command_ms) <= SENTRY_AIM_COMMAND_TIMEOUT_MS;
    float dt_s = 0.005f;
    if (last_aim_update_ms != 0u)
        dt_s = ClampFloat((float)(now_ms - last_aim_update_ms) * 0.001f,
                          0.0f, 0.05f);
    last_aim_update_ms = now_ms;
    UpdateAimMotor(aim_yaw_motor, &aim_yaw_origin_deg, &aim_yaw_target_deg,
                   &aim_yaw_origin_valid, &aim_yaw_active, rx_aim_yaw_mrad_s,
                   SENTRY_AIM_YAW_TRAVEL_DEG, dt_s, enabled);
#if SENTRY_AIM_PITCH_ENABLED
    UpdateAimMotor(aim_pitch_motor, &aim_pitch_origin_deg, &aim_pitch_target_deg,
                   &aim_pitch_origin_valid, &aim_pitch_active, rx_aim_pitch_mrad_s,
                   SENTRY_AIM_PITCH_TRAVEL_DEG, dt_s, enabled);
#endif
    if (lower_yaw_motor == NULL)
        return;

    const uint8_t lower_online = DMMotorIsOnline(lower_yaw_motor);
    const uint8_t lower_request = enabled && rx_lower_spin_request;
    if (lower_online != lower_yaw_last_online)
    {
        lower_yaw_last_online = lower_online;
        if (lower_online)
            LOGINFO("[steer-nav2] lower yaw online");
        else
            LOGWARNING("[steer-nav2] lower yaw offline");
    }
    if (lower_request != lower_yaw_last_request)
    {
        lower_yaw_last_request = lower_request;
        LOGINFO("[steer-nav2] lower yaw spin request=%u",
                (unsigned int)lower_request);
    }

    if (!enabled)
    {
        DMMotorStop(lower_yaw_motor);
        lower_yaw_spinning = 0u;
        return;
    }

    if (rx_lower_spin_request)
    {
        DMMotorEnable(lower_yaw_motor);
        DMMotorSetRef(lower_yaw_motor,
                      lower_online && DMMotorIsEnabled(lower_yaw_motor) ?
                          SENTRY_LOWER_YAW_SPIN_RAD_S : 0.0f);
        lower_yaw_spinning = 1u;
    }
    else if (lower_yaw_spinning)
    {
        DMMotorSetRef(lower_yaw_motor, 0.0f);
        if (!lower_online || fabsf(lower_yaw_motor->measure.velocity) < 0.15f)
        {
            DMMotorStop(lower_yaw_motor);
            lower_yaw_spinning = 0u;
        }
    }
}

static void Nav2UsbRxCallback(uint16_t length)
{
    for (uint16_t i = 0u; i < length; ++i)
    {
        const uint8_t byte = usb_rx_buffer[i];

        if (rx_frame_index == 0u)
        {
            if (byte == NAV2_PROTOCOL_MAGIC_0)
                rx_frame[rx_frame_index++] = byte;
            continue;
        }

        if (rx_frame_index == 1u && byte != NAV2_PROTOCOL_MAGIC_1)
        {
            rx_frame_index = byte == NAV2_PROTOCOL_MAGIC_0 ? 1u : 0u;
            rx_frame[0] = byte;
            continue;
        }

        rx_frame[rx_frame_index++] = byte;
        if (rx_frame_index == NAV2_COMMAND_FRAME_SIZE)
        {
            DecodeCommandFrame(rx_frame);
            rx_frame_index = 0u;
        }
    }
}

static void Nav2UsbTxCallback(uint16_t length)
{
    (void)length;
    telemetry_tx_pending = 0u;
}

static float ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

static uint8_t IsMotionCommand(float vx_mm_s, float vy_mm_s, float wz_mrad_s)
{
    return fabsf(vx_mm_s) >= CHASSIS_STEER_NAV2_STOP_TRANSLATION_MM_S ||
           fabsf(vy_mm_s) >= CHASSIS_STEER_NAV2_STOP_TRANSLATION_MM_S ||
           fabsf(wz_mrad_s) >= CHASSIS_STEER_NAV2_STOP_ANGULAR_MRAD_S;
}

static float SlewScalar(float current, float target, float max_delta)
{
    const float delta = target - current;

    if (delta > max_delta)
        return current + max_delta;
    if (delta < -max_delta)
        return current - max_delta;
    return target;
}

static void SlewMotionCommand(uint32_t now_ms,
                              float target_vx_mm_s,
                              float target_vy_mm_s,
                              float target_wz_mrad_s)
{
    float dt_s = 0.005f;

    if (last_slew_ms != 0u)
    {
        const uint32_t elapsed_ms = now_ms - last_slew_ms;
        if (elapsed_ms > 0u)
            dt_s = ClampFloat((float)elapsed_ms * 0.001f, 0.001f, 0.05f);
    }
    last_slew_ms = now_ms;

    const float delta_vx = target_vx_mm_s - slewed_vx_mm_s;
    const float delta_vy = target_vy_mm_s - slewed_vy_mm_s;
    const float delta_speed = sqrtf(delta_vx * delta_vx + delta_vy * delta_vy);
    const float max_translation_delta =
        CHASSIS_STEER_NAV2_MAX_TRANSLATION_ACCEL_MM_S2 * dt_s;

    if (delta_speed > max_translation_delta && delta_speed > 0.0f)
    {
        const float scale = max_translation_delta / delta_speed;
        slewed_vx_mm_s += delta_vx * scale;
        slewed_vy_mm_s += delta_vy * scale;
    }
    else
    {
        slewed_vx_mm_s = target_vx_mm_s;
        slewed_vy_mm_s = target_vy_mm_s;
    }

    slewed_wz_mrad_s = SlewScalar(
        slewed_wz_mrad_s,
        target_wz_mrad_s,
        CHASSIS_STEER_NAV2_MAX_ANGULAR_ACCEL_MRAD_S2 * dt_s);
}

static void CalculateSwerveTargets(float vx_mm_s, float vy_mm_s, float wz_mrad_s)
{
    float wheel_speed_mm_s[SENTRY_STEER_NAV2_MOTOR_COUNT];
    float wheel_angle_rad[SENTRY_STEER_NAV2_MOTOR_COUNT];
    float max_wheel_rpm = 0.0f;
    const float translation_speed = sqrtf(vx_mm_s * vx_mm_s + vy_mm_s * vy_mm_s);
    const float wz_rad_s = ClampFloat(
        wz_mrad_s,
        -CHASSIS_STEER_NAV2_MAX_ANGULAR_MRAD_S,
        CHASSIS_STEER_NAV2_MAX_ANGULAR_MRAD_S) * 0.001f;

    if (translation_speed > CHASSIS_STEER_NAV2_MAX_TRANSLATION_MM_S)
    {
        const float scale = CHASSIS_STEER_NAV2_MAX_TRANSLATION_MM_S / translation_speed;
        vx_mm_s *= scale;
        vy_mm_s *= scale;
    }

    for (uint8_t i = 0u; i < SENTRY_STEER_NAV2_MOTOR_COUNT; ++i)
    {
        const float module_vx = vx_mm_s - wz_rad_s * module_y_mm[i];
        const float module_vy_left = vy_mm_s + wz_rad_s * module_x_mm[i];
        const float module_vy_right = -module_vy_left;

        wheel_speed_mm_s[i] = sqrtf(module_vx * module_vx +
                                    module_vy_right * module_vy_right);
        wheel_angle_rad[i] = atan2f(module_vy_right, module_vx);
        drive_target_rpm[i] = wheel_speed_mm_s[i] * 60.0f * REDUCTION_RATIO_WHEEL /
                              (2.0f * PI * RADIUS_WHEEL);
        if (drive_target_rpm[i] > max_wheel_rpm)
            max_wheel_rpm = drive_target_rpm[i];
    }

    if (max_wheel_rpm > CHASSIS_STEER_NAV2_MAX_DRIVE_RPM)
    {
        const float scale = CHASSIS_STEER_NAV2_MAX_DRIVE_RPM / max_wheel_rpm;
        for (uint8_t i = 0u; i < SENTRY_STEER_NAV2_MOTOR_COUNT; ++i)
            drive_target_rpm[i] *= scale;
    }

    for (uint8_t i = 0u; i < SENTRY_STEER_NAV2_MOTOR_COUNT; ++i)
    {
        const uint8_t wheel_mask = (uint8_t)(1u << i);

        if (wheel_speed_mm_s[i] < CHASSIS_STEER_NAV2_DIRECTION_HOLD_MM_S)
        {
            drive_target_rpm[i] = 0.0f;
            if ((steer_target_valid_bitmap & wheel_mask) == 0u)
            {
                target_total_angle_deg[i] = steer_motors[i]->measure.total_angle;
                sentry_steer_nav2_state.target_ecd[i] = steer_motors[i]->measure.ecd;
            }
            sentry_steer_nav2_state.drive_target_rpm[i] = 0;
            continue;
        }

        const int32_t direction_offset_ecd =
            (int32_t)lroundf(wheel_angle_rad[i] * (float)DJI_ECD_FULL_RANGE /
                             (2.0f * PI));
        uint16_t target_ecd = WrapEcdTarget(
            (int32_t)steer_zero_ecd[i] +
            (int32_t)steer_direction_sign[i] * direction_offset_ecd);
        int16_t delta_ecd = WrapEcdDelta(
            (int32_t)target_ecd - (int32_t)steer_motors[i]->measure.ecd);
        const int16_t main_delta_ecd = delta_ecd;
        const int16_t main_abs_delta_ecd = (int16_t)abs(main_delta_ecd);
        float drive_sign = 1.0f;

        if ((drive_flip_bitmap & wheel_mask) != 0u)
        {
            if (main_abs_delta_ecd <= CHASSIS_STEER_NAV2_FLIP_EXIT_ECD)
                drive_flip_bitmap &= (uint8_t)~wheel_mask;
        }
        else if (main_abs_delta_ecd >= CHASSIS_STEER_NAV2_FLIP_ENTER_ECD)
        {
            drive_flip_bitmap |= wheel_mask;
        }

        if ((drive_flip_bitmap & wheel_mask) != 0u)
        {
            delta_ecd = main_delta_ecd >= 0
                            ? (int16_t)(main_delta_ecd - DJI_ECD_HALF_RANGE)
                            : (int16_t)(main_delta_ecd + DJI_ECD_HALF_RANGE);
            drive_sign = -1.0f;
        }

        target_ecd = WrapEcdTarget(
            (int32_t)steer_motors[i]->measure.ecd + (int32_t)delta_ecd);
        target_total_angle_deg[i] = steer_motors[i]->measure.total_angle +
                                    (float)delta_ecd * ECD_ANGLE_COEF_DJI;
        drive_target_rpm[i] *= drive_sign;
        sentry_steer_nav2_state.target_ecd[i] = target_ecd;
        steer_target_valid_bitmap |= wheel_mask;
    }
}

static float AlignmentDriveScale(int16_t abs_error_ecd)
{
    if (abs_error_ecd >= CHASSIS_STEER_NAV2_DRIVE_START_ALIGNMENT_ECD)
        return 0.0f;
    if (abs_error_ecd <= CHASSIS_STEER_NAV2_DRIVE_FULL_ALIGNMENT_ECD)
        return 1.0f;

    return (float)(CHASSIS_STEER_NAV2_DRIVE_START_ALIGNMENT_ECD - abs_error_ecd) /
           (float)(CHASSIS_STEER_NAV2_DRIVE_START_ALIGNMENT_ECD -
                   CHASSIS_STEER_NAV2_DRIVE_FULL_ALIGNMENT_ECD);
}

static float SteerCurrentLimit(int16_t abs_error_ecd)
{
    if (abs_error_ecd <= CHASSIS_STEER_NAV2_BOOST_START_ECD)
        return CHASSIS_STEER_NAV2_MAX_STEER_CURRENT;
    if (abs_error_ecd >= CHASSIS_STEER_NAV2_BOOST_FULL_ECD)
        return CHASSIS_STEER_NAV2_BOOST_MAX_STEER_CURRENT;

    const float boost_scale =
        (float)(abs_error_ecd - CHASSIS_STEER_NAV2_BOOST_START_ECD) /
        (float)(CHASSIS_STEER_NAV2_BOOST_FULL_ECD -
                CHASSIS_STEER_NAV2_BOOST_START_ECD);
    return CHASSIS_STEER_NAV2_MAX_STEER_CURRENT +
           boost_scale * (CHASSIS_STEER_NAV2_BOOST_MAX_STEER_CURRENT -
                          CHASSIS_STEER_NAV2_MAX_STEER_CURRENT);
}

static uint8_t ApplySwerveTargets(void)
{
    uint8_t aligned_bitmap = 0u;
    uint8_t active_drive_bitmap = 0u;
    float common_drive_scale = 1.0f;

    for (uint8_t i = 0u; i < SENTRY_STEER_NAV2_MOTOR_COUNT; ++i)
    {
        const int16_t error_ecd = WrapEcdDelta(
            (int32_t)sentry_steer_nav2_state.target_ecd[i] -
            (int32_t)steer_motors[i]->measure.ecd);
        const int16_t abs_error_ecd = (int16_t)abs(error_ecd);

        sentry_steer_nav2_state.current_ecd[i] = steer_motors[i]->measure.ecd;
        sentry_steer_nav2_state.error_ecd[i] = error_ecd;
        steer_motors[i]->motor_controller.speed_PID.MaxOut =
            SteerCurrentLimit(abs_error_ecd);
        DJIMotorSetRef(steer_motors[i], target_total_angle_deg[i]);
        DJIMotorEnable(steer_motors[i]);

        const uint8_t wheel_mask = (uint8_t)(1u << i);

        if (fabsf(drive_target_rpm[i]) < 0.5f)
        {
            aligned_bitmap |= wheel_mask;
            continue;
        }

        active_drive_bitmap |= wheel_mask;
        if (abs_error_ecd <= CHASSIS_STEER_NAV2_DRIVE_FULL_ALIGNMENT_ECD)
            aligned_bitmap |= (uint8_t)(1u << i);

        const float wheel_alignment_scale = AlignmentDriveScale(abs_error_ecd);
        if (wheel_alignment_scale < common_drive_scale)
            common_drive_scale = wheel_alignment_scale;
    }

    if (active_drive_bitmap == 0u)
        common_drive_scale = 0.0f;
    drive_common_scale_permille =
        (uint16_t)lroundf(common_drive_scale * 1000.0f);

    for (uint8_t i = 0u; i < SENTRY_STEER_NAV2_MOTOR_COUNT; ++i)
    {
        applied_drive_target_rpm[i] = drive_target_rpm[i] * common_drive_scale;
        sentry_steer_nav2_state.drive_target_rpm[i] =
            (int16_t)lroundf(applied_drive_target_rpm[i]);
        const float drive_target = applied_drive_target_rpm[i] * RPM_2_ANGLE_PER_SEC;
        DJIMotorSetRef(drive_motors[i], drive_target);
        if (common_drive_scale > 0.0f)
            DJIMotorEnable(drive_motors[i]);
        else
            DJIMotorStop(drive_motors[i]);
    }

    return aligned_bitmap;
}

static void RecordTelemetryRecovery(uint32_t now_ms, const char *reason)
{
    ++telemetry_recovery_count;
    if (telemetry_recovery_count != 1u &&
        (now_ms - last_telemetry_recovery_log_ms) <
            CHASSIS_STEER_NAV2_TELEMETRY_RECOVERY_LOG_PERIOD_MS)
        return;

    last_telemetry_recovery_log_ms = now_ms;
    LOGWARNING("[steer-nav2] recovered %s USB telemetry transfer count=%lu",
               reason,
               (unsigned long)telemetry_recovery_count);
}

static void SendTelemetry(uint32_t now_ms, uint8_t command_fresh)
{
    uint8_t frame[NAV2_TELEMETRY_FRAME_SIZE] = {0};
    uint8_t status = 0u;
    uint8_t transmit_result;

    if ((now_ms - last_telemetry_ms) < CHASSIS_STEER_NAV2_TELEMETRY_PERIOD_MS)
        return;
    last_telemetry_ms = now_ms;

    if (telemetry_tx_pending)
    {
        if ((now_ms - telemetry_tx_started_ms) <
            CHASSIS_STEER_NAV2_TELEMETRY_TX_TIMEOUT_MS)
            return;

        const uint8_t abort_result = USBAbortTransmit();
        telemetry_tx_pending = 0u;
        telemetry_busy_started_ms = 0u;
        if (abort_result == USBD_OK)
            RecordTelemetryRecovery(now_ms, "stalled");
    }

    if (sentry_steer_nav2_state.control_enabled)
        status |= NAV2_STATUS_CONTROL_ENABLED;
    if (command_fresh)
        status |= NAV2_STATUS_COMMAND_FRESH;

    frame[0] = NAV2_PROTOCOL_MAGIC_0;
    frame[1] = NAV2_PROTOCOL_MAGIC_1;
    frame[2] = NAV2_PROTOCOL_VERSION;
    frame[3] = NAV2_TELEMETRY_TYPE;
    WriteU16Le(&frame[4], telemetry_sequence);
    WriteU32Le(&frame[6], now_ms);
    for (uint8_t i = 0u; i < SENTRY_STEER_NAV2_MOTOR_COUNT; ++i)
        WriteU16Le(&frame[10u + 2u * i], steer_motors[i]->measure.ecd);
    for (uint8_t i = 0u; i < SENTRY_STEER_NAV2_MOTOR_COUNT; ++i)
        WriteU16Le(&frame[18u + 2u * i], (uint16_t)MotorRpm(drive_motors[i]));
    frame[26] = status;
    WriteU16Le(&frame[27], ProtocolCrc16(frame, NAV2_TELEMETRY_FRAME_SIZE - 2u));

    /* Set the flag before submission so an immediate completion interrupt
       cannot race with the task and leave a false pending state behind. */
    telemetry_tx_pending = 1u;
    transmit_result = USBTransmit(frame, NAV2_TELEMETRY_FRAME_SIZE);
    if (transmit_result == USBD_OK)
    {
        telemetry_tx_started_ms = now_ms;
        telemetry_busy_started_ms = 0u;
        ++telemetry_sequence;
        return;
    }

    telemetry_tx_pending = 0u;
    if (transmit_result != USBD_BUSY)
    {
        telemetry_busy_started_ms = 0u;
        return;
    }

    if (telemetry_busy_started_ms == 0u)
    {
        telemetry_busy_started_ms = now_ms;
        return;
    }

    if ((now_ms - telemetry_busy_started_ms) >=
        CHASSIS_STEER_NAV2_TELEMETRY_TX_TIMEOUT_MS)
    {
        const uint8_t abort_result = USBAbortTransmit();
        telemetry_busy_started_ms = 0u;
        if (abort_result == USBD_OK)
            RecordTelemetryRecovery(now_ms, "busy");
    }
}

static void LogNav2State(uint32_t now_ms, uint8_t command_fresh)
{
    if ((now_ms - last_log_ms) < CHASSIS_STEER_NAV2_LOG_PERIOD_MS)
        return;
    last_log_ms = now_ms;

    LOGINFO("[steer-nav2] cmd=%d/%d/%d fresh=%u enabled=%u aligned=0x%x "
            "drive_scale=%u/1000 flip=0x%x",
            sentry_steer_nav2_state.command_vx_mm_s,
            sentry_steer_nav2_state.command_vy_mm_s,
            sentry_steer_nav2_state.command_wz_mrad_s,
            command_fresh,
            sentry_steer_nav2_state.control_enabled,
            sentry_steer_nav2_state.aligned_bitmap,
            drive_common_scale_permille,
            drive_flip_bitmap);
    LOGINFO("[steer-nav2] steer_error LF/RF/LR/RR=%d/%d/%d/%d",
            sentry_steer_nav2_state.error_ecd[STEER_NAV2_LF],
            sentry_steer_nav2_state.error_ecd[STEER_NAV2_RF],
            sentry_steer_nav2_state.error_ecd[STEER_NAV2_LR],
            sentry_steer_nav2_state.error_ecd[STEER_NAV2_RR]);
    LOGINFO("[steer-nav2] drive_rpm target=%d/%d/%d/%d actual=%d/%d/%d/%d",
            sentry_steer_nav2_state.drive_target_rpm[STEER_NAV2_LF],
            sentry_steer_nav2_state.drive_target_rpm[STEER_NAV2_RF],
            sentry_steer_nav2_state.drive_target_rpm[STEER_NAV2_LR],
            sentry_steer_nav2_state.drive_target_rpm[STEER_NAV2_RR],
            LogicalDriveMotorRpm(STEER_NAV2_LF),
            LogicalDriveMotorRpm(STEER_NAV2_RF),
            LogicalDriveMotorRpm(STEER_NAV2_LR),
            LogicalDriveMotorRpm(STEER_NAV2_RR));
}

void SentrySteerNav2Init(void)
{
    Motor_Init_Config_s aim_config = {
        .can_init_config.can_handle = &SENTRY_AIM_YAW_CAN_BUS,
        .can_init_config.tx_id = SENTRY_AIM_YAW_MOTOR_ID,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = GIMBAL_YAW_ANGLE_PID_KP,
                .MaxOut = 120.0f,
            },
            .speed_PID = {
                .Kp = GIMBAL_YAW_SPEED_PID_KP,
                .Ki = GIMBAL_YAW_SPEED_PID_KI,
                .IntegralLimit = GIMBAL_YAW_SPEED_PID_INT_LIMIT,
                .MaxOut = 6000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = GIMBAL_YAW_MOTOR_REVERSE,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    Motor_Init_Config_s steer_config = {
        .can_init_config.can_handle = NULL,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = CHASSIS_STEER_NAV2_ANGLE_PID_KP,
                .Ki = CHASSIS_STEER_NAV2_ANGLE_PID_KI,
                .Kd = CHASSIS_STEER_NAV2_ANGLE_PID_KD,
                .DeadBand = CHASSIS_STEER_NAV2_ANGLE_PID_DEADBAND,
                .Improve = PID_IMPROVE_NONE,
                .MaxOut = CHASSIS_STEER_NAV2_MAX_STEER_SPEED_APS,
            },
            .speed_PID = {
                .Kp = CHASSIS_STEER_NAV2_SPEED_PID_KP,
                .Ki = CHASSIS_STEER_NAV2_SPEED_PID_KI,
                .Kd = CHASSIS_STEER_NAV2_SPEED_PID_KD,
                .DeadBand = CHASSIS_STEER_NAV2_SPEED_PID_DEADBAND,
                .Improve = PID_OutputFilter,
                .Output_LPF_RC = CHASSIS_STEER_NAV2_SPEED_PID_OUTPUT_LPF_RC,
                .MaxOut = CHASSIS_STEER_NAV2_MAX_STEER_CURRENT,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    Motor_Init_Config_s drive_config = {
        .can_init_config.can_handle = NULL,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = CHASSIS_STEER_NAV2_DRIVE_SPEED_PID_KP,
                .Ki = CHASSIS_STEER_NAV2_DRIVE_SPEED_PID_KI,
                .Kd = CHASSIS_STEER_NAV2_DRIVE_SPEED_PID_KD,
                .DeadBand = CHASSIS_STEER_NAV2_DRIVE_SPEED_PID_DEADBAND,
                .Improve = PID_IMPROVE_NONE,
                .MaxOut = CHASSIS_STEER_NAV2_DRIVE_SPEED_PID_MAX_OUT,
            },
            .current_PID = {
                .Kp = CHASSIS_CURRENT_PID_KP,
                .Ki = CHASSIS_CURRENT_PID_KI,
                .Kd = CHASSIS_CURRENT_PID_KD,
                .IntegralLimit = CHASSIS_CURRENT_PID_INT_LIMIT,
                .MaxOut = CHASSIS_CURRENT_PID_MAX_OUT,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    USB_Init_Config_s usb_config = {
        .tx_cbk = Nav2UsbTxCallback,
        .rx_cbk = Nav2UsbRxCallback,
    };

    _Static_assert(sizeof(SentrySteerNav2Telemetry_s) <= 255u,
                   "Swerve dashboard topic exceeds message-center limit");
    dashboard_pub = PubRegister("sentry_steer_feed", sizeof(SentrySteerNav2Telemetry_s));

    for (uint8_t i = 0u; i < SENTRY_STEER_NAV2_MOTOR_COUNT; ++i)
    {
        steer_config.can_init_config.can_handle = CHASSIS_MOTOR_CAN_HANDLE(steer_motor_ids[i]);
        steer_config.can_init_config.tx_id = steer_motor_ids[i];
        steer_motors[i] = DJIMotorInit(&steer_config);
        DJIMotorStop(steer_motors[i]);
        sentry_steer_nav2_state.target_ecd[i] = steer_zero_ecd[i];

        drive_config.can_init_config.can_handle = CHASSIS_MOTOR_CAN_HANDLE(drive_motor_ids[i]);
        drive_config.can_init_config.tx_id = drive_motor_ids[i];
        drive_config.controller_setting_init_config.motor_reverse_flag = drive_motor_directions[i];
        drive_motors[i] = DJIMotorInit(&drive_config);
        DJIMotorSetRef(drive_motors[i], 0.0f);
        DJIMotorStop(drive_motors[i]);
    }

    aim_yaw_motor = DJIMotorInit(&aim_config);
    if (aim_yaw_motor == NULL)
        LOGERROR("[steer-nav2] aiming yaw registration failed");
    else
    {
        DJIMotorStop(aim_yaw_motor);
        LOGINFO("[steer-nav2] aiming yaw registered id=%u",
                (unsigned int)SENTRY_AIM_YAW_MOTOR_ID);
    }

    if (SENTRY_LOWER_YAW_DM_ENABLED)
    {
        Motor_Init_Config_s lower_config = {
            .can_init_config = {
                .can_handle = &SENTRY_LOWER_YAW_CAN_BUS,
                .tx_id = SENTRY_LOWER_YAW_MOTOR_ID,
                .rx_id = SENTRY_LOWER_YAW_MASTER_ID,
            },
            .controller_setting_init_config = {
                .angle_feedback_source = MOTOR_FEED,
                .speed_feedback_source = MOTOR_FEED,
                .outer_loop_type = OPEN_LOOP,
                .close_loop_type = OPEN_LOOP,
                .motor_reverse_flag = SENTRY_LOWER_YAW_MOTOR_REVERSE,
                .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            },
            .motor_type = MOTOR_TYPE_NONE,
            .dm_auto_zero_on_boot = 0u,
        };
        lower_yaw_motor = DMMotorInit(&lower_config);
        if (lower_yaw_motor == NULL)
            LOGERROR("[steer-nav2] lower yaw ID 6 registration failed");
        else
        {
            DMMotorSetControlMode(lower_yaw_motor, DM_MOTOR_CONTROL_SPEED);
            DMMotorSetRef(lower_yaw_motor, 0.0f);
            DMMotorStop(lower_yaw_motor);
            LOGINFO("[steer-nav2] lower DaMiao yaw registered id=%u",
                    (unsigned int)SENTRY_LOWER_YAW_MOTOR_ID);
        }
    }
#if SENTRY_AIM_PITCH_ENABLED
    aim_config.can_init_config.can_handle = &SENTRY_AIM_PITCH_CAN_BUS;
    aim_config.can_init_config.tx_id = SENTRY_AIM_PITCH_MOTOR_ID;
    aim_config.controller_param_init_config.angle_PID.Kp = GIMBAL_PITCH_ANGLE_PID_KP;
    aim_config.controller_param_init_config.speed_PID.Kp = GIMBAL_PITCH_SPEED_PID_KP;
    aim_config.controller_param_init_config.speed_PID.Ki = GIMBAL_PITCH_SPEED_PID_KI;
    aim_config.controller_setting_init_config.motor_reverse_flag = GIMBAL_PITCH_MOTOR_REVERSE;
    aim_pitch_motor = DJIMotorInit(&aim_config);
    if (aim_pitch_motor == NULL)
        LOGERROR("[steer-nav2] aiming pitch registration failed");
    else
        DJIMotorStop(aim_pitch_motor);
#endif

    usb_rx_buffer = USBInit(usb_config);
    LEDSetStatus(LED_STATUS_RED_ON);
    LOGINFO("[steer-nav2] waiting for motor feedback and USB velocity commands");
}

void SentrySteerNav2Task(void)
{
    const uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();
    uint8_t steer_online;
    uint8_t drive_online;
    uint8_t command_fresh;
    float vx_mm_s;
    float vy_mm_s;
    float wz_mrad_s;

    /* A bus-off on CAN1/CAN2 otherwise never recovers on its own (H7 FDCAN
     * has no bxCAN AutoBusOff bit); poll before reading online status so a
     * cleared bus-off is reflected in this same cycle. */
    CANPollRecoverAll();
    UpdateManualAim(now_ms);
    steer_online = MotorOnlineBitmap(steer_motors);
    drive_online = MotorOnlineBitmap(drive_motors);

    if (handled_rx_sequence != rx_sequence)
    {
        handled_rx_sequence = rx_sequence;
        last_command_ms = now_ms;
        command_seen = 1u;
    }

    command_fresh = command_seen &&
                    (now_ms - last_command_ms) <= CHASSIS_STEER_NAV2_COMMAND_TIMEOUT_MS;
    vx_mm_s = command_fresh && rx_enable ? (float)rx_vx_mm_s : 0.0f;
    vy_mm_s = command_fresh && rx_enable ? (float)rx_vy_mm_s : 0.0f;
    wz_mrad_s = command_fresh && rx_enable ? (float)rx_wz_mrad_s : 0.0f;

    sentry_steer_nav2_state.steer_online_bitmap = steer_online;
    sentry_steer_nav2_state.drive_online_bitmap = drive_online;

    if (steer_online != STEER_ALL_ONLINE_BITMAP ||
        drive_online != STEER_ALL_ONLINE_BITMAP)
    {
        DisableAllMotors();
        online_timer_started = 0u;
        sentry_steer_nav2_state.aligned_bitmap = 0u;
        LEDSetStatus(LED_STATUS_RED_ON);
        SendTelemetry(now_ms, command_fresh);
        LogNav2State(now_ms, command_fresh);
        PublishDashboardTelemetry(command_fresh);
        return;
    }

    if (!online_timer_started)
    {
        online_stable_start_ms = now_ms;
        online_timer_started = 1u;
    }

    if (!command_fresh || !rx_enable ||
        (now_ms - online_stable_start_ms) < CHASSIS_STEER_KEYBOARD_ONLINE_STABLE_MS)
    {
        DisableAllMotors();
        sentry_steer_nav2_state.aligned_bitmap = 0u;
        LEDSetStatus(LED_STATUS_BLUE_ON);
        SendTelemetry(now_ms, command_fresh);
        LogNav2State(now_ms, command_fresh);
        PublishDashboardTelemetry(command_fresh);
        return;
    }

    if (!IsMotionCommand(vx_mm_s, vy_mm_s, wz_mrad_s))
    {
        HoldZeroMotion(now_ms);
        LEDSetStatus(LED_STATUS_BLUE_ON);
        SendTelemetry(now_ms, command_fresh);
        LogNav2State(now_ms, command_fresh);
        PublishDashboardTelemetry(command_fresh);
        return;
    }

    PrepareClosedLoop();
    CancelDriveBrake();
    SlewMotionCommand(now_ms, vx_mm_s, vy_mm_s, wz_mrad_s);
    sentry_steer_nav2_state.command_vx_mm_s = (int16_t)lroundf(slewed_vx_mm_s);
    sentry_steer_nav2_state.command_vy_mm_s = (int16_t)lroundf(slewed_vy_mm_s);
    sentry_steer_nav2_state.command_wz_mrad_s = (int16_t)lroundf(slewed_wz_mrad_s);
    CalculateSwerveTargets(slewed_vx_mm_s, slewed_vy_mm_s, slewed_wz_mrad_s);
    sentry_steer_nav2_state.aligned_bitmap = ApplySwerveTargets();
    sentry_steer_nav2_state.control_enabled = 1u;
    control_mode = STEER_NAV2_CONTROL_MOTION;
    LEDSetStatus(sentry_steer_nav2_state.aligned_bitmap == STEER_ALL_ONLINE_BITMAP
                     ? LED_STATUS_GREEN_ON
                     : LED_STATUS_YELLOW_ON);
    SendTelemetry(now_ms, command_fresh);
    LogNav2State(now_ms, command_fresh);
    PublishDashboardTelemetry(command_fresh);
}

#else

volatile SentrySteerNav2State_s sentry_steer_nav2_state;

void SentrySteerNav2Init(void) {}
void SentrySteerNav2Task(void) {}

#endif
