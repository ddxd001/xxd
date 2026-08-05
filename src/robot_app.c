#include "robot_app.h"

#include <limits.h>
#include <string.h>

#include "hal_data.h"
#include "robot_config.h"
#include "robot_kinematics.h"
#include "robot_protocol.h"
#include "sts3215.h"

#define LORA_RX_RING_SIZE       (256U)
#define CONTROL_PAYLOAD_LENGTH  (10U)
#define CONFIG_PAYLOAD_LENGTH   (4U)
#define TELEMETRY_PAYLOAD_SIZE  (85U)

typedef struct st_robot_command
{
    int16_t vx;
    int16_t vy;
    int16_t omega;
    int16_t lift;
} robot_command_t;

typedef struct st_robot_runtime
{
    robot_state_t state;
    uint32_t      fault_bits;
    robot_command_t command;
    robot_wheel_ramp_t wheel_ramp;
    int16_t       wheel_speed[3];
    int16_t       servo_speed[STS3215_SERVO_COUNT];
    bool          homed;
    bool          upper_limit_valid;
    int32_t       upper_limit_counts;
    int32_t       lift_position;
    int32_t       lift_target;
    int16_t       previous_lift_raw;
    bool          lift_feedback_valid;
    uint32_t      lift_feedback_stamp;
    bool          lift_was_online;
    bool          home_switch;
    bool          home_raw;
    uint8_t       home_stable_ms;
    uint32_t      homing_start_ms;
    uint32_t      homing_progress_ms;
    int32_t       homing_progress_position;
    uint32_t      last_control_ms;
    uint32_t      last_control_tick;
    uint32_t      last_servo_tick;
    uint32_t      last_telemetry_tick;
    uint16_t      last_control_sequence;
    bool          control_sequence_valid;
    bool          control_received;
    uint16_t      telemetry_sequence;
    uint32_t      recovery_started_ms;
    uint32_t      recovery_clear_ms;
    uint32_t      recovery_window_start_ms;
    uint8_t       recovery_attempts;
    bool          recovery_clear_pending;
    uint16_t      servo_temp_status_ms[STS3215_SERVO_COUNT];
    bool          fault_snapshot_valid;
    uint8_t       fault_snapshot_servo_id;
    uint8_t       fault_snapshot_protocol_error;
    uint8_t       fault_snapshot_status_flags;
    uint8_t       fault_snapshot_temperature_c;
    uint8_t       fault_snapshot_temperature_limit_c;
} robot_runtime_t;

static robot_runtime_t g_robot;
static volatile uint32_t g_milliseconds;
static volatile bool g_home_irq_seen;

static uint8_t g_lora_rx_byte;
static volatile uint16_t g_lora_rx_head;
static volatile uint16_t g_lora_rx_tail;
static uint8_t g_lora_rx_ring[LORA_RX_RING_SIZE];
static volatile bool g_lora_tx_busy;
static bool g_lora_open;
static uint8_t g_lora_tx_buffer[ROBOT_PROTOCOL_MAX_FRAME];
static robot_parser_t g_lora_parser;

static int16_t clamp_command (int16_t value)
{
    if (value > 1000)
    {
        return 1000;
    }
    if (value < -1000)
    {
        return -1000;
    }
    return value;
}

static int32_t clamp_i32 (int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static void set_zero_motion (void)
{
    memset(&g_robot.command, 0, sizeof(g_robot.command));
    robot_kinematics_stop(&g_robot.wheel_ramp);
    memset(g_robot.wheel_speed, 0, sizeof(g_robot.wheel_speed));
    memset(g_robot.servo_speed, 0, sizeof(g_robot.servo_speed));
}

static void enter_fault (uint32_t fault)
{
    g_robot.fault_bits |= fault;
    g_robot.state = ROBOT_STATE_FAULT;
    set_zero_motion();
}

static void enter_disarmed (void)
{
    g_robot.state = (0U != g_robot.fault_bits) ? ROBOT_STATE_FAULT :
                    (g_robot.homed ? ROBOT_STATE_DISARMED_HOMED : ROBOT_STATE_DISARMED_UNHOMED);
    set_zero_motion();
}

static void stop_outputs_preserve_command (void)
{
    robot_kinematics_stop(&g_robot.wheel_ramp);
    memset(g_robot.wheel_speed, 0, sizeof(g_robot.wheel_speed));
    memset(g_robot.servo_speed, 0, sizeof(g_robot.servo_speed));
}

static void enter_servo_recovery (uint32_t now_ms)
{
    if (ROBOT_STATE_RECOVERING == g_robot.state)
    {
        return;
    }
    if (ROBOT_STATE_ARMED != g_robot.state)
    {
        enter_fault(ROBOT_FAULT_SERVO_STATUS);
        return;
    }
    if ((0U == g_robot.recovery_attempts) ||
        ((uint32_t) (now_ms - g_robot.recovery_window_start_ms) > ROBOT_RECOVERY_WINDOW_MS))
    {
        g_robot.recovery_attempts = 0U;
        g_robot.recovery_window_start_ms = now_ms;
    }
    if (g_robot.recovery_attempts >= ROBOT_RECOVERY_MAX_ATTEMPTS)
    {
        enter_fault(ROBOT_FAULT_SERVO_STATUS);
        return;
    }

    g_robot.recovery_attempts++;
    g_robot.recovery_started_ms = now_ms;
    g_robot.recovery_clear_pending = false;
    g_robot.fault_bits |= ROBOT_FAULT_SERVO_STATUS;
    g_robot.state = ROBOT_STATE_RECOVERING;
    stop_outputs_preserve_command();
}

static void resume_after_servo_recovery (void)
{
    g_robot.fault_bits &= ~(uint32_t) ROBOT_FAULT_SERVO_STATUS;
    g_robot.recovery_clear_pending = false;
    stop_outputs_preserve_command();
    (void) sts3215_set_torque(true);
    g_robot.state = ROBOT_STATE_ARMED;
}

static bool sequence_is_newer (uint16_t sequence, uint16_t previous)
{
    return (int16_t) (sequence - previous) > 0;
}

static void clear_fault_if_safe (void)
{
    if (sts3215_all_online() && !sts3215_any_serious_fault())
    {
        g_robot.fault_bits = 0U;
        g_robot.fault_snapshot_valid = false;
        g_robot.fault_snapshot_servo_id = 0U;
        g_robot.fault_snapshot_protocol_error = 0U;
        g_robot.fault_snapshot_status_flags = 0U;
        g_robot.fault_snapshot_temperature_c = 0U;
        g_robot.fault_snapshot_temperature_limit_c = 0U;
        (void) sts3215_set_torque(true);
        enter_disarmed();
    }
}

static void start_homing (uint32_t now_ms)
{
    if ((ROBOT_STATE_ARMED != g_robot.state) && (ROBOT_STATE_FAULT != g_robot.state) && sts3215_all_online())
    {
        g_robot.homed = false;
        g_robot.state = ROBOT_STATE_HOMING;
        g_robot.homing_start_ms = now_ms;
        g_robot.homing_progress_ms = now_ms;
        g_robot.homing_progress_position = g_robot.lift_position;
        set_zero_motion();
        g_robot.servo_speed[3] = ROBOT_LIFT_HOME_SPEED;
    }
}

static void start_calibration (void)
{
    if ((ROBOT_STATE_DISARMED_HOMED == g_robot.state) && g_robot.homed && !g_robot.upper_limit_valid &&
        g_robot.lift_feedback_valid && (0U == g_robot.fault_bits) && sts3215_all_online())
    {
        set_zero_motion();
        g_robot.lift_target = g_robot.lift_position;
        g_robot.state = ROBOT_STATE_CALIBRATING;
    }
}

static void handle_control_frame (robot_frame_t const * frame, uint32_t now_ms)
{
    if (frame->length != CONTROL_PAYLOAD_LENGTH)
    {
        return;
    }
    if (g_robot.control_sequence_valid && !sequence_is_newer(frame->sequence, g_robot.last_control_sequence))
    {
        return;
    }

    g_robot.control_sequence_valid = true;
    g_robot.last_control_sequence = frame->sequence;
    g_robot.last_control_ms = now_ms;
    g_robot.control_received = true;

    int16_t const vx = clamp_command(robot_protocol_get_i16(&frame->payload[0]));
    int16_t const vy = clamp_command(robot_protocol_get_i16(&frame->payload[2]));
    int16_t const omega = clamp_command(robot_protocol_get_i16(&frame->payload[4]));
    int16_t const lift = clamp_command(robot_protocol_get_i16(&frame->payload[6]));
    uint16_t const flags = robot_protocol_get_u16(&frame->payload[8]);

    if (0U != (flags & ROBOT_FLAG_DISARM))
    {
        enter_disarmed();
        return;
    }
    if (0U != (flags & ROBOT_FLAG_CLEAR_FAULT))
    {
        clear_fault_if_safe();
    }
    if (0U != (flags & ROBOT_FLAG_HOME))
    {
        start_homing(now_ms);
        return;
    }
    if (0U != (flags & ROBOT_FLAG_CALIBRATE))
    {
        start_calibration();
        return;
    }
    if ((0U != (flags & ROBOT_FLAG_ARM)) &&
        ((ROBOT_STATE_DISARMED_UNHOMED == g_robot.state) || (ROBOT_STATE_DISARMED_HOMED == g_robot.state)) &&
        (0U == g_robot.fault_bits) && sts3215_all_online())
    {
        g_robot.state = ROBOT_STATE_ARMED;
    }

    if ((ROBOT_STATE_ARMED == g_robot.state) || (ROBOT_STATE_RECOVERING == g_robot.state))
    {
        g_robot.command.vx = vx;
        g_robot.command.vy = vy;
        g_robot.command.omega = omega;
        g_robot.command.lift = lift;
    }
    else if (ROBOT_STATE_CALIBRATING == g_robot.state)
    {
        g_robot.command.vx = 0;
        g_robot.command.vy = 0;
        g_robot.command.omega = 0;
        g_robot.command.lift = lift;
    }
}

static void handle_config_frame (robot_frame_t const * frame)
{
    if ((frame->length != CONFIG_PAYLOAD_LENGTH) || (ROBOT_STATE_ARMED == g_robot.state) ||
        (ROBOT_STATE_HOMING == g_robot.state))
    {
        return;
    }
    int32_t const upper_limit = robot_protocol_get_i32(frame->payload);
    if ((upper_limit > 0) && (upper_limit < 100000000))
    {
        bool const was_calibrating = (ROBOT_STATE_CALIBRATING == g_robot.state);
        g_robot.upper_limit_counts = upper_limit;
        g_robot.upper_limit_valid = true;
        if (was_calibrating)
        {
            g_robot.lift_target = clamp_i32(g_robot.lift_position, 0, upper_limit);
            set_zero_motion();
            g_robot.state = ROBOT_STATE_DISARMED_HOMED;
        }
        else if (g_robot.homed)
        {
            g_robot.lift_target = clamp_i32(g_robot.lift_target, 0, upper_limit);
        }
    }
    else
    {
        g_robot.upper_limit_valid = false;
    }
}

static void consume_lora (uint32_t now_ms)
{
    robot_frame_t frame;
    while (g_lora_rx_tail != g_lora_rx_head)
    {
        uint8_t const byte = g_lora_rx_ring[g_lora_rx_tail];
        g_lora_rx_tail = (uint16_t) ((g_lora_rx_tail + 1U) % LORA_RX_RING_SIZE);
        if (robot_protocol_parse_byte(&g_lora_parser, byte, &frame))
        {
            if (ROBOT_MSG_CONTROL == frame.type)
            {
                handle_control_frame(&frame, now_ms);
            }
            else if (ROBOT_MSG_CONFIG == frame.type)
            {
                handle_config_frame(&frame);
            }
            else
            {
                /* Unsupported messages are intentionally ignored. */
            }
        }
    }
}

static void update_home_switch (void)
{
    bsp_io_level_t level = BSP_IO_LEVEL_LOW;
    (void) g_ioport.p_api->pinRead(g_ioport.p_ctrl, BSP_IO_PORT_05_PIN_02, &level);
    bool const raw = (BSP_IO_LEVEL_HIGH == level);
    if (raw == g_robot.home_raw)
    {
        if (g_robot.home_stable_ms < ROBOT_HOME_DEBOUNCE_MS)
        {
            uint16_t const elapsed = (uint16_t) g_robot.home_stable_ms + ROBOT_CONTROL_PERIOD_MS;
            g_robot.home_stable_ms = (uint8_t) ((elapsed > ROBOT_HOME_DEBOUNCE_MS) ? ROBOT_HOME_DEBOUNCE_MS : elapsed);
        }
        if (g_robot.home_stable_ms >= ROBOT_HOME_DEBOUNCE_MS)
        {
            g_robot.home_switch = raw;
        }
    }
    else
    {
        g_robot.home_raw = raw;
        g_robot.home_stable_ms = 0U;
    }
    g_home_irq_seen = false;
}

static void update_lift_feedback (void)
{
    sts3215_status_t const * status = sts3215_status(3U);
    if ((NULL == status) || !status->online)
    {
        if (g_robot.lift_was_online)
        {
            g_robot.homed = false;
            g_robot.upper_limit_valid = false;
        }
        g_robot.lift_was_online = false;
        g_robot.lift_feedback_valid = false;
        return;
    }

    if (!g_robot.lift_was_online)
    {
        g_robot.homed = false;
        g_robot.lift_feedback_valid = false;
        g_robot.lift_was_online = true;
    }
    if (status->last_seen_ms == g_robot.lift_feedback_stamp)
    {
        return;
    }
    g_robot.lift_feedback_stamp = status->last_seen_ms;
    int16_t raw = (int16_t) (status->position % 4096);
    if (raw < 0)
    {
        raw = (int16_t) (raw + 4096);
    }
    if (!g_robot.lift_feedback_valid)
    {
        g_robot.previous_lift_raw = raw;
        g_robot.lift_feedback_valid = true;
        return;
    }
    int32_t delta = (int32_t) raw - g_robot.previous_lift_raw;
    if (delta > 2048)
    {
        delta -= 4096;
    }
    else if (delta < -2048)
    {
        delta += 4096;
    }
    g_robot.lift_position += delta;
    g_robot.previous_lift_raw = raw;
}

static int16_t lift_hold_speed (void)
{
    if (!g_robot.homed || !g_robot.upper_limit_valid || !g_robot.lift_feedback_valid)
    {
        return 0;
    }
    int32_t error = g_robot.lift_target - g_robot.lift_position;
    if ((error < ROBOT_LIFT_HOLD_DEADBAND_COUNTS) && (error > -ROBOT_LIFT_HOLD_DEADBAND_COUNTS))
    {
        return 0;
    }
    int32_t speed = (error * ROBOT_LIFT_POSITION_KP_NUM) / ROBOT_LIFT_POSITION_KP_DEN;
    speed = clamp_i32(speed, -ROBOT_LIFT_MAX_SPEED, ROBOT_LIFT_MAX_SPEED);
    if ((g_robot.lift_position <= 0) && (speed < 0))
    {
        speed = 0;
    }
    if ((g_robot.lift_position >= g_robot.upper_limit_counts) && (speed > 0))
    {
        speed = 0;
    }
    return (int16_t) speed;
}

static void capture_servo_fault_snapshot (uint8_t index, sts3215_status_t const * status)
{
    if (g_robot.fault_snapshot_valid || (NULL == status))
    {
        return;
    }

    g_robot.fault_snapshot_valid = true;
    g_robot.fault_snapshot_servo_id = (uint8_t) (index + 1U);
    g_robot.fault_snapshot_protocol_error = status->protocol_error;
    g_robot.fault_snapshot_status_flags = status->status_flags;
    g_robot.fault_snapshot_temperature_c = status->temperature_c;
    g_robot.fault_snapshot_temperature_limit_c = status->temperature_limit_valid ?
                                                   status->temperature_limit_c : 0U;
}

static void evaluate_servo_faults (bool * hard_fault, bool * recoverable_fault)
{
    uint8_t const recoverable_mask = STS3215_STATUS_VOLTAGE | STS3215_STATUS_CURRENT |
                                     STS3215_STATUS_OVERLOAD;
    uint8_t const hard_status_mask = (uint8_t) ~(recoverable_mask | STS3215_STATUS_TEMPERATURE);

    *hard_fault = false;
    *recoverable_fault = false;
    for (uint8_t i = 0U; i < STS3215_SERVO_COUNT; i++)
    {
        sts3215_status_t const * status = sts3215_status(i);
        if ((NULL == status) || !status->online)
        {
            g_robot.servo_temp_status_ms[i] = 0U;
            continue;
        }

        uint8_t const flags = (uint8_t) (status->protocol_error | status->status_flags);
        bool const numeric_over_temperature = status->temperature_c >= ROBOT_SERVO_FAULT_TEMP_C;
        if ((0U != flags) || numeric_over_temperature)
        {
            capture_servo_fault_snapshot(i, status);
        }

        bool confirmed_temperature_status = false;
        if (0U != (flags & STS3215_STATUS_TEMPERATURE))
        {
            uint32_t const elapsed = (uint32_t) g_robot.servo_temp_status_ms[i] + ROBOT_CONTROL_PERIOD_MS;
            g_robot.servo_temp_status_ms[i] = (uint16_t) ((elapsed > UINT16_MAX) ? UINT16_MAX : elapsed);
            confirmed_temperature_status = g_robot.servo_temp_status_ms[i] >= ROBOT_TEMP_STATUS_CONFIRM_MS;
        }
        else
        {
            g_robot.servo_temp_status_ms[i] = 0U;
        }

        bool const hard_status = 0U != (flags & hard_status_mask);
        bool const recoverable_status = 0U != (flags & recoverable_mask);
        if (numeric_over_temperature || confirmed_temperature_status || hard_status)
        {
            *hard_fault = true;
            capture_servo_fault_snapshot(i, status);
        }
        if (recoverable_status)
        {
            *recoverable_fault = true;
            capture_servo_fault_snapshot(i, status);
        }
    }
}

static void control_tick (uint32_t now_ms)
{
    update_home_switch();
    update_lift_feedback();

    bool const servo_offline = (now_ms >= ROBOT_SERVO_STARTUP_GRACE_MS) && !sts3215_all_online();
    bool hard_servo_fault;
    bool recoverable_servo_fault;
    evaluate_servo_faults(&hard_servo_fault, &recoverable_servo_fault);

    if (servo_offline)
    {
        enter_fault(ROBOT_FAULT_SERVO_OFFLINE);
    }
    else if (hard_servo_fault)
    {
        enter_fault(ROBOT_FAULT_SERVO_STATUS);
    }
    else if (recoverable_servo_fault)
    {
        g_robot.recovery_clear_pending = false;
        enter_servo_recovery(now_ms);
    }
    else if (ROBOT_STATE_RECOVERING == g_robot.state)
    {
        if (!g_robot.recovery_clear_pending)
        {
            g_robot.recovery_clear_pending = true;
            g_robot.recovery_clear_ms = now_ms;
        }
        else if ((uint32_t) (now_ms - g_robot.recovery_clear_ms) >= ROBOT_RECOVERY_CLEAR_MS)
        {
            resume_after_servo_recovery();
        }
    }

    if ((ROBOT_STATE_RECOVERING == g_robot.state) &&
        ((uint32_t) (now_ms - g_robot.recovery_started_ms) >= ROBOT_RECOVERY_TIMEOUT_MS))
    {
        enter_fault(ROBOT_FAULT_SERVO_STATUS);
    }
    if ((ROBOT_STATE_ARMED == g_robot.state) && (g_robot.recovery_attempts > 0U) &&
        ((uint32_t) (now_ms - g_robot.recovery_window_start_ms) > ROBOT_RECOVERY_WINDOW_MS))
    {
        g_robot.recovery_attempts = 0U;
    }
    if (g_robot.control_received && ((uint32_t) (now_ms - g_robot.last_control_ms) > ROBOT_LINK_TIMEOUT_MS) &&
        ((ROBOT_STATE_ARMED == g_robot.state) || (ROBOT_STATE_HOMING == g_robot.state) ||
         (ROBOT_STATE_CALIBRATING == g_robot.state) || (ROBOT_STATE_RECOVERING == g_robot.state)))
    {
        enter_fault(ROBOT_FAULT_LINK_TIMEOUT);
    }

    if (ROBOT_STATE_RECOVERING == g_robot.state)
    {
        stop_outputs_preserve_command();
        return;
    }

    if (ROBOT_STATE_HOMING == g_robot.state)
    {
        memset(g_robot.servo_speed, 0, sizeof(g_robot.servo_speed));
        g_robot.servo_speed[3] = ROBOT_LIFT_HOME_SPEED;
        if (g_robot.home_switch)
        {
            g_robot.servo_speed[3] = 0;
            g_robot.lift_position = 0;
            g_robot.lift_target = 0;
            g_robot.homed = true;
            g_robot.state = ROBOT_STATE_DISARMED_HOMED;
        }
        else if ((uint32_t) (now_ms - g_robot.homing_start_ms) >= ROBOT_HOME_TIMEOUT_MS)
        {
            enter_fault(ROBOT_FAULT_HOME_TIMEOUT);
        }
        else if (g_robot.lift_feedback_valid)
        {
            int32_t const progress = g_robot.lift_position - g_robot.homing_progress_position;
            if ((progress >= ROBOT_HOME_PROGRESS_COUNTS) || (progress <= -ROBOT_HOME_PROGRESS_COUNTS))
            {
                g_robot.homing_progress_position = g_robot.lift_position;
                g_robot.homing_progress_ms = now_ms;
            }
            else if ((uint32_t) (now_ms - g_robot.homing_progress_ms) >= ROBOT_HOME_STALL_TIMEOUT_MS)
            {
                enter_fault(ROBOT_FAULT_HOME_STALL);
            }
        }
        return;
    }

    if (ROBOT_STATE_CALIBRATING == g_robot.state)
    {
        robot_kinematics_stop(&g_robot.wheel_ramp);
        memset(g_robot.wheel_speed, 0, sizeof(g_robot.wheel_speed));
        memset(g_robot.servo_speed, 0, sizeof(g_robot.servo_speed));
        if (g_robot.command.lift > ROBOT_WHEEL_DEADZONE)
        {
            g_robot.servo_speed[3] = ROBOT_LIFT_CALIBRATION_SPEED;
        }
        return;
    }

    if (ROBOT_STATE_ARMED == g_robot.state)
    {
        int16_t target[3];
        robot_kinematics_calculate(g_robot.command.vx, g_robot.command.vy, g_robot.command.omega, target);
        robot_kinematics_ramp(&g_robot.wheel_ramp, target, g_robot.wheel_speed);
        for (uint8_t i = 0U; i < 3U; i++)
        {
            g_robot.servo_speed[i] = g_robot.wheel_speed[i];
        }
        if (g_robot.homed && g_robot.upper_limit_valid)
        {
            int32_t const step = ((int32_t) g_robot.command.lift * ROBOT_LIFT_TARGET_STEP_PER_TICK) / 1000;
            g_robot.lift_target = clamp_i32(g_robot.lift_target + step, 0, g_robot.upper_limit_counts);
            g_robot.servo_speed[3] = lift_hold_speed();
        }
        else
        {
            g_robot.servo_speed[3] = 0;
        }
    }
    else if ((ROBOT_STATE_FAULT == g_robot.state) &&
             (ROBOT_FAULT_LINK_TIMEOUT == (g_robot.fault_bits & ROBOT_FAULT_LINK_TIMEOUT)))
    {
        memset(g_robot.servo_speed, 0, sizeof(g_robot.servo_speed));
        g_robot.servo_speed[3] = lift_hold_speed();
    }
    else
    {
        set_zero_motion();
    }
}

static void send_telemetry (uint32_t now_ms)
{
    if (g_lora_tx_busy || !g_lora_open)
    {
        return;
    }
    uint8_t payload[TELEMETRY_PAYLOAD_SIZE];
    memset(payload, 0, sizeof(payload));
    payload[0] = (uint8_t) g_robot.state;
    robot_protocol_put_i32(&payload[1], (int32_t) g_robot.fault_bits);
    robot_protocol_put_u16(&payload[5], g_robot.last_control_sequence);
    uint32_t const age = g_robot.control_received ? (now_ms - g_robot.last_control_ms) : UINT16_MAX;
    robot_protocol_put_u16(&payload[7], (uint16_t) ((age > UINT16_MAX) ? UINT16_MAX : age));
    for (uint8_t i = 0U; i < STS3215_SERVO_COUNT; i++)
    {
        sts3215_status_t const * status = sts3215_status(i);
        if ((NULL != status) && status->online)
        {
            payload[9] |= (uint8_t) (1U << i);
        }
        if ((NULL != status) && (status->temperature_c >= ROBOT_SERVO_WARN_TEMP_C))
        {
            payload[10] |= (uint8_t) (1U << i);
        }
    }
    payload[11] = g_robot.home_switch ? 1U : 0U;
    payload[12] = g_robot.homed ? 1U : 0U;
    robot_protocol_put_i32(&payload[13], g_robot.lift_position);
    robot_protocol_put_i32(&payload[17], g_robot.lift_target);
    robot_protocol_put_i32(&payload[21], g_robot.upper_limit_valid ? g_robot.upper_limit_counts : 0);
    for (uint8_t i = 0U; i < 3U; i++)
    {
        robot_protocol_put_i16(&payload[25U + (i * 2U)], g_robot.wheel_speed[i]);
    }
    for (uint8_t i = 0U; i < STS3215_SERVO_COUNT; i++)
    {
        sts3215_status_t const * status = sts3215_status(i);
        uint8_t const offset = (uint8_t) (31U + (i * 12U));
        if (NULL != status)
        {
            payload[offset] = status->online ? 1U : 0U;
            payload[offset + 1U] = status->protocol_error;
            payload[offset + 2U] = status->status_flags;
            payload[offset + 3U] = status->temperature_c;
            payload[offset + 4U] = status->temperature_limit_valid ? status->temperature_limit_c : 0U;
            payload[offset + 5U] = status->voltage_tenths;
            robot_protocol_put_i16(&payload[offset + 6U], status->position);
            robot_protocol_put_i16(&payload[offset + 8U], status->speed);
            robot_protocol_put_i16(&payload[offset + 10U], status->current);
        }
    }
    payload[79] = g_robot.fault_snapshot_valid ? 1U : 0U;
    payload[80] = g_robot.fault_snapshot_servo_id;
    payload[81] = g_robot.fault_snapshot_protocol_error;
    payload[82] = g_robot.fault_snapshot_status_flags;
    payload[83] = g_robot.fault_snapshot_temperature_c;
    payload[84] = g_robot.fault_snapshot_temperature_limit_c;

    size_t const length = robot_protocol_encode(ROBOT_MSG_TELEMETRY, g_robot.telemetry_sequence++, payload,
                                                 sizeof(payload), g_lora_tx_buffer, sizeof(g_lora_tx_buffer));
    if (length > 0U)
    {
        g_lora_tx_busy = true;
        if (FSP_SUCCESS != g_uart0.p_api->write(g_uart0.p_ctrl, g_lora_tx_buffer, (uint32_t) length))
        {
            g_lora_tx_busy = false;
        }
    }
}

bool robot_app_init (void)
{
    memset(&g_robot, 0, sizeof(g_robot));
    g_robot.state = ROBOT_STATE_DISARMED_UNHOMED;
    robot_protocol_parser_init(&g_lora_parser);

    bool ok = sts3215_init();
    if (FSP_SUCCESS == g_uart0.p_api->open(g_uart0.p_ctrl, g_uart0.p_cfg))
    {
        g_lora_open = true;
        if (FSP_SUCCESS != g_uart0.p_api->read(g_uart0.p_ctrl, &g_lora_rx_byte, 1U))
        {
            ok = false;
        }
    }
    else
    {
        ok = false;
    }

    if ((FSP_SUCCESS != g_system_tick.p_api->open(g_system_tick.p_ctrl, g_system_tick.p_cfg)) ||
        (FSP_SUCCESS != g_system_tick.p_api->start(g_system_tick.p_ctrl)))
    {
        ok = false;
    }
    if ((FSP_SUCCESS != g_home_irq.p_api->open(g_home_irq.p_ctrl, g_home_irq.p_cfg)) ||
        (FSP_SUCCESS != g_home_irq.p_api->enable(g_home_irq.p_ctrl)))
    {
        ok = false;
    }
    if (!ok)
    {
        enter_fault(ROBOT_FAULT_DRIVER_INIT);
    }
    return ok;
}

void robot_app_process (void)
{
    uint32_t const now_ms = robot_app_millis();
    consume_lora(now_ms);

    if ((uint32_t) (now_ms - g_robot.last_control_tick) >= ROBOT_CONTROL_PERIOD_MS)
    {
        g_robot.last_control_tick = now_ms;
        control_tick(now_ms);
    }
    if ((uint32_t) (now_ms - g_robot.last_servo_tick) >= ROBOT_SERVO_COMMAND_PERIOD_MS)
    {
        g_robot.last_servo_tick = now_ms;
        (void) sts3215_send_speeds(g_robot.servo_speed, ROBOT_SERVO_ACCELERATION);
    }
    sts3215_process(now_ms);

    if ((uint32_t) (now_ms - g_robot.last_telemetry_tick) >= ROBOT_TELEMETRY_PERIOD_MS)
    {
        g_robot.last_telemetry_tick = now_ms;
        send_telemetry(now_ms);
    }
}

uint32_t robot_app_millis (void)
{
    return g_milliseconds;
}

void system_tick_callback (timer_callback_args_t * p_args)
{
    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        g_milliseconds++;
    }
}

void home_irq_callback (external_irq_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    g_home_irq_seen = true;
}

void servo_uart_callback (uart_callback_args_t * p_args)
{
    sts3215_uart_callback(p_args);
}

void lora_uart_callback (uart_callback_args_t * p_args)
{
    if (UART_EVENT_RX_COMPLETE == p_args->event)
    {
        uint16_t const next = (uint16_t) ((g_lora_rx_head + 1U) % LORA_RX_RING_SIZE);
        if (next != g_lora_rx_tail)
        {
            g_lora_rx_ring[g_lora_rx_head] = g_lora_rx_byte;
            g_lora_rx_head = next;
        }
        if (g_lora_open)
        {
            (void) g_uart0.p_api->read(g_uart0.p_ctrl, &g_lora_rx_byte, 1U);
        }
    }
    else if (UART_EVENT_TX_COMPLETE == p_args->event)
    {
        g_lora_tx_busy = false;
    }
    else
    {
        g_lora_tx_busy = false;
    }
}
