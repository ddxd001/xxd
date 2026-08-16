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
    bool          lift_zero_valid;
    int32_t       lift_position;
    int32_t       lift_target;
    int16_t       previous_lift_raw;
    bool          lift_feedback_valid;
    bool          lift_manual_active;
    uint32_t      lift_feedback_stamp;
    bool          lift_was_online;
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
    uint32_t      fault_auto_clear_ms;
    bool          fault_auto_clear_pending;
    bool          temperature_fault_latched;
    uint16_t      servo_numeric_temp_ms[STS3215_SERVO_COUNT];
    bool          fault_snapshot_valid;
    uint8_t       fault_snapshot_servo_id;
    uint8_t       fault_snapshot_protocol_error;
    uint8_t       fault_snapshot_status_flags;
    uint8_t       fault_snapshot_temperature_c;
    uint8_t       fault_snapshot_temperature_limit_c;
} robot_runtime_t;

static robot_runtime_t g_robot;
static volatile uint32_t g_milliseconds;
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

static void capture_lift_hold_target (void)
{
    if (g_robot.lift_zero_valid && g_robot.lift_feedback_valid)
    {
        g_robot.lift_target = (g_robot.lift_position > 0) ? g_robot.lift_position : 0;
    }
    g_robot.lift_manual_active = false;
}

static void set_zero_motion (void)
{
    capture_lift_hold_target();
    memset(&g_robot.command, 0, sizeof(g_robot.command));
    robot_kinematics_stop(&g_robot.wheel_ramp);
    memset(g_robot.wheel_speed, 0, sizeof(g_robot.wheel_speed));
    memset(g_robot.servo_speed, 0, sizeof(g_robot.servo_speed));
}

static void enter_fault (uint32_t fault)
{
    g_robot.fault_bits |= fault;
    g_robot.state = ROBOT_STATE_FAULT;
    g_robot.fault_auto_clear_pending = false;
    set_zero_motion();
}

static void enter_disarmed (void)
{
    g_robot.state = (0U != g_robot.fault_bits) ? ROBOT_STATE_FAULT :
                    (g_robot.lift_zero_valid ? ROBOT_STATE_DISARMED_HOMED : ROBOT_STATE_DISARMED_UNHOMED);
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
    g_robot.command.lift = 0;
    capture_lift_hold_target();
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
        g_robot.fault_auto_clear_pending = false;
        g_robot.temperature_fault_latched = false;
        (void) sts3215_set_torque(true);
        enter_disarmed();
    }
}

static void attempt_servo_fault_auto_clear (uint32_t now_ms, bool servo_fault_active)
{
    if ((ROBOT_STATE_FAULT != g_robot.state) ||
        (ROBOT_FAULT_SERVO_STATUS != g_robot.fault_bits) || g_robot.temperature_fault_latched ||
        servo_fault_active || !sts3215_all_online() || sts3215_any_serious_fault())
    {
        g_robot.fault_auto_clear_pending = false;
        return;
    }

    if (!g_robot.fault_auto_clear_pending)
    {
        g_robot.fault_auto_clear_pending = true;
        g_robot.fault_auto_clear_ms = now_ms;
        return;
    }
    if ((uint32_t) (now_ms - g_robot.fault_auto_clear_ms) < ROBOT_SERVO_FAULT_AUTO_CLEAR_MS)
    {
        return;
    }

    /* Keep the fault snapshot for diagnostics, but clear the live fault and
     * return to DISARMED. A recovered servo fault never automatically re-arms. */
    g_robot.fault_bits &= ~(uint32_t) ROBOT_FAULT_SERVO_STATUS;
    g_robot.fault_auto_clear_pending = false;
    (void) sts3215_set_torque(true);
    enter_disarmed();
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
    /* HOME and CALIBRATE remain reserved in protocol v3 but intentionally do nothing. */
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
            else
            {
                /* CONFIG and unsupported messages are intentionally ignored. */
            }
        }
    }
}

static void update_lift_feedback (void)
{
    sts3215_status_t const * status = sts3215_status(3U);
    if ((NULL == status) || !status->online)
    {
        if (g_robot.lift_was_online)
        {
            g_robot.lift_zero_valid = false;
        }
        g_robot.lift_was_online = false;
        g_robot.lift_feedback_valid = false;
        return;
    }

    if (!g_robot.lift_was_online)
    {
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
    g_robot.lift_position += delta * ROBOT_LIFT_FEEDBACK_DIRECTION;
    g_robot.previous_lift_raw = raw;
}

static int16_t lift_output_speed (int16_t logical_speed)
{
    return (int16_t) (logical_speed * ROBOT_LIFT_MOTOR_DIRECTION);
}

static int16_t lift_manual_speed (int16_t command)
{
    int32_t speed = ((int32_t) command * ROBOT_LIFT_MAX_SPEED) / 1000;
    speed = clamp_i32(speed, -ROBOT_LIFT_MAX_SPEED, ROBOT_LIFT_MAX_SPEED);
    /* There is no lower-limit switch. The operator places the lift at its true
     * lowest position before power-up, and the unfolded encoder position is
     * then the only lower-bound reference. */
    if ((g_robot.lift_position <= 0) && (speed < 0))
    {
        speed = 0;
    }
    return (int16_t) speed;
}

static int16_t lift_hold_speed (void)
{
    if (!g_robot.lift_zero_valid || !g_robot.lift_feedback_valid)
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

static void evaluate_servo_faults (bool * hard_fault, bool * recoverable_fault, bool * temperature_fault)
{
    uint8_t const recoverable_mask = STS3215_STATUS_VOLTAGE | STS3215_STATUS_CURRENT |
                                     STS3215_STATUS_OVERLOAD;
    uint8_t const hard_status_mask = (uint8_t) ~(recoverable_mask | STS3215_STATUS_TEMPERATURE);

    *hard_fault = false;
    *recoverable_fault = false;
    *temperature_fault = false;
    for (uint8_t i = 0U; i < STS3215_SERVO_COUNT; i++)
    {
        sts3215_status_t const * status = sts3215_status(i);
        if ((NULL == status) || !status->online)
        {
            g_robot.servo_numeric_temp_ms[i] = 0U;
            continue;
        }

        uint8_t const flags = (uint8_t) (status->protocol_error | status->status_flags);
        bool const numeric_over_temperature = status->temperature_c >= ROBOT_SERVO_FAULT_TEMP_C;
        if ((0U != flags) || numeric_over_temperature)
        {
            capture_servo_fault_snapshot(i, status);
        }

        bool confirmed_numeric_over_temperature = false;
        if (numeric_over_temperature)
        {
            uint32_t const elapsed = (uint32_t) g_robot.servo_numeric_temp_ms[i] + ROBOT_CONTROL_PERIOD_MS;
            g_robot.servo_numeric_temp_ms[i] = (uint16_t) ((elapsed > UINT16_MAX) ? UINT16_MAX : elapsed);
            confirmed_numeric_over_temperature =
                g_robot.servo_numeric_temp_ms[i] >= ROBOT_TEMP_NUMERIC_CONFIRM_MS;
        }
        else
        {
            g_robot.servo_numeric_temp_ms[i] = 0U;
        }

        bool const hard_status = 0U != (flags & hard_status_mask);
        bool const recoverable_status = 0U != (flags & recoverable_mask);
        if (confirmed_numeric_over_temperature || hard_status)
        {
            *hard_fault = true;
            capture_servo_fault_snapshot(i, status);
        }
        if (confirmed_numeric_over_temperature)
        {
            *temperature_fault = true;
            if (!g_robot.temperature_fault_latched)
            {
                /* Replace an older diagnostic-only temperature snapshot with
                 * the sample that actually satisfied the 500 ms trip rule. */
                g_robot.fault_snapshot_valid = false;
                capture_servo_fault_snapshot(i, status);
            }
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
    update_lift_feedback();

    bool const servo_offline = (now_ms >= ROBOT_SERVO_STARTUP_GRACE_MS) && !sts3215_all_online();
    bool hard_servo_fault;
    bool recoverable_servo_fault;
    bool temperature_servo_fault;
    evaluate_servo_faults(&hard_servo_fault, &recoverable_servo_fault, &temperature_servo_fault);

    if (servo_offline)
    {
        enter_fault(ROBOT_FAULT_SERVO_OFFLINE);
    }
    else if (hard_servo_fault)
    {
        enter_fault(ROBOT_FAULT_SERVO_STATUS);
        if (temperature_servo_fault)
        {
            g_robot.temperature_fault_latched = true;
        }
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
    attempt_servo_fault_auto_clear(now_ms, servo_offline || hard_servo_fault || recoverable_servo_fault);
    if (g_robot.control_received && ((uint32_t) (now_ms - g_robot.last_control_ms) > ROBOT_LINK_TIMEOUT_MS) &&
        ((ROBOT_STATE_ARMED == g_robot.state) || (ROBOT_STATE_RECOVERING == g_robot.state)))
    {
        enter_fault(ROBOT_FAULT_LINK_TIMEOUT);
    }

    if (ROBOT_STATE_RECOVERING == g_robot.state)
    {
        stop_outputs_preserve_command();
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
        if (g_robot.lift_zero_valid && g_robot.lift_feedback_valid)
        {
            int16_t logical_lift_speed;
            if (0 != g_robot.command.lift)
            {
                g_robot.lift_manual_active = true;
                g_robot.lift_target = (g_robot.lift_position > 0) ? g_robot.lift_position : 0;
                logical_lift_speed = lift_manual_speed(g_robot.command.lift);
            }
            else
            {
                if (g_robot.lift_manual_active)
                {
                    capture_lift_hold_target();
                }
                logical_lift_speed = lift_hold_speed();
            }
            g_robot.servo_speed[3] = lift_output_speed(logical_lift_speed);
        }
        else
        {
            g_robot.lift_manual_active = false;
            g_robot.servo_speed[3] = 0;
        }
    }
    else if ((ROBOT_STATE_FAULT == g_robot.state) &&
             (ROBOT_FAULT_LINK_TIMEOUT == (g_robot.fault_bits & ROBOT_FAULT_LINK_TIMEOUT)))
    {
        memset(g_robot.servo_speed, 0, sizeof(g_robot.servo_speed));
        g_robot.servo_speed[3] = lift_output_speed(lift_hold_speed());
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
    payload[11] = 0U; /* Reserved lower-switch field; no switch is installed. */
    payload[12] = g_robot.lift_zero_valid ? 1U : 0U;
    robot_protocol_put_i32(&payload[13], g_robot.lift_position);
    robot_protocol_put_i32(&payload[17], g_robot.lift_target);
    robot_protocol_put_i32(&payload[21], 0);
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
    g_robot.state = ROBOT_STATE_DISARMED_HOMED;
    g_robot.lift_zero_valid = true;
    g_robot.lift_position = 0;
    g_robot.lift_target = 0;
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
    /* Retained only because the generated FSP configuration references this
     * symbol. g_home_irq is never opened, so P502 has no runtime effect. */
    FSP_PARAMETER_NOT_USED(p_args);
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
