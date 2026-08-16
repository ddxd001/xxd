#include "sts3215.h"

#include <string.h>

#include "hal_data.h"
#include "robot_config.h"

#define STS_HEADER                    (0xFFU)
#define STS_BROADCAST_ID              (0xFEU)
#define STS_INST_READ                 (0x02U)
#define STS_INST_WRITE                (0x03U)
#define STS_INST_SYNC_WRITE           (0x83U)
#define STS_REG_TORQUE_ENABLE         (40U)
#define STS_REG_ACCELERATION          (41U)
#define STS_REG_MAX_TEMPERATURE       (13U)
#define STS_REG_PRESENT_POSITION      (56U)
#define STS_FEEDBACK_LENGTH           (15U)
#define STS_DIRECTION_BIT             (0x8000U)
#define STS_RESPONSE_TIMEOUT_MS       (8U)
#define STS_POLL_INTERVAL_MS          (10U)
#define STS_RX_RING_SIZE              (128U)
#define STS_MAX_PACKET_SIZE           (48U)

typedef enum e_sts_read_kind
{
    STS_READ_FEEDBACK,
    STS_READ_TEMPERATURE_LIMIT
} sts_read_kind_t;

typedef struct st_sts_driver
{
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    uint8_t           rx_ring[STS_RX_RING_SIZE];
    uint8_t           rx_byte;
    uint8_t           tx_buffer[STS_MAX_PACKET_SIZE];
    volatile bool     tx_busy;
    bool              uart_open;
    bool              awaiting_response;
    bool              torque_request_pending;
    bool              torque_enable_request;
    uint8_t           pending_id;
    sts_read_kind_t   pending_read_kind;
    uint8_t           poll_index;
    uint32_t          request_time_ms;
    uint32_t          last_poll_ms;
    uint8_t           packet[32];
    uint8_t           packet_count;
    uint8_t           packet_expected;
    sts3215_status_t  status[STS3215_SERVO_COUNT];
} sts_driver_t;

static sts_driver_t g_sts;

static uint8_t checksum (uint8_t const * data, uint8_t first, uint8_t last)
{
    uint16_t sum = 0U;
    for (uint8_t i = first; i < last; i++)
    {
        sum += data[i];
    }
    return (uint8_t) (~sum);
}

static uint16_t encode_signed (int16_t value)
{
    if (value < 0)
    {
        return (uint16_t) (-value) | STS_DIRECTION_BIT;
    }
    return (uint16_t) value;
}

static int16_t decode_signed (uint8_t low, uint8_t high, uint16_t direction_bit)
{
    uint16_t value = (uint16_t) low | ((uint16_t) high << 8);
    if (0U != (value & direction_bit))
    {
        value &= (uint16_t) ~direction_bit;
        return (int16_t) (-(int32_t) value);
    }
    return (int16_t) value;
}

static bool send_packet (uint8_t const * data, uint8_t length)
{
    if (g_sts.tx_busy || (length > sizeof(g_sts.tx_buffer)))
    {
        return false;
    }
    memcpy(g_sts.tx_buffer, data, length);
    g_sts.tx_busy = true;
    fsp_err_t const error = g_uart8.p_api->write(g_uart8.p_ctrl, g_sts.tx_buffer, length);
    if (FSP_SUCCESS != error)
    {
        g_sts.tx_busy = false;
        return false;
    }
    return true;
}

static bool send_read_request (uint8_t id, uint32_t now_ms)
{
    uint8_t packet[8];
    packet[0] = STS_HEADER;
    packet[1] = STS_HEADER;
    packet[2] = id;
    packet[3] = 4U;
    packet[4] = STS_INST_READ;
    packet[5] = STS_REG_PRESENT_POSITION;
    packet[6] = STS_FEEDBACK_LENGTH;
    packet[7] = checksum(packet, 2U, 7U);
    if (!send_packet(packet, sizeof(packet)))
    {
        return false;
    }
    g_sts.awaiting_response = true;
    g_sts.pending_id = id;
    g_sts.pending_read_kind = STS_READ_FEEDBACK;
    g_sts.request_time_ms = now_ms;
    return true;
}

static bool send_temperature_limit_request (uint8_t id, uint32_t now_ms)
{
    uint8_t packet[8];
    packet[0] = STS_HEADER;
    packet[1] = STS_HEADER;
    packet[2] = id;
    packet[3] = 4U;
    packet[4] = STS_INST_READ;
    packet[5] = STS_REG_MAX_TEMPERATURE;
    packet[6] = 1U;
    packet[7] = checksum(packet, 2U, 7U);
    if (!send_packet(packet, sizeof(packet)))
    {
        return false;
    }

    uint8_t const index = (uint8_t) (id - 1U);
    if ((index < STS3215_SERVO_COUNT) && (g_sts.status[index].temperature_limit_attempts < UINT8_MAX))
    {
        g_sts.status[index].temperature_limit_attempts++;
    }
    g_sts.awaiting_response = true;
    g_sts.pending_id = id;
    g_sts.pending_read_kind = STS_READ_TEMPERATURE_LIMIT;
    g_sts.request_time_ms = now_ms;
    return true;
}

static bool send_torque_request (bool enable)
{
    uint8_t packet[8];
    packet[0] = STS_HEADER;
    packet[1] = STS_HEADER;
    packet[2] = STS_BROADCAST_ID;
    packet[3] = 4U;
    packet[4] = STS_INST_WRITE;
    packet[5] = STS_REG_TORQUE_ENABLE;
    packet[6] = enable ? 1U : 0U;
    packet[7] = checksum(packet, 2U, 7U);
    return send_packet(packet, sizeof(packet));
}

static void mark_timeout (void)
{
    uint8_t const index = (uint8_t) (g_sts.pending_id - 1U);
    if ((index < STS3215_SERVO_COUNT) && (STS_READ_FEEDBACK == g_sts.pending_read_kind))
    {
        if (g_sts.status[index].missed_responses < UINT8_MAX)
        {
            g_sts.status[index].missed_responses++;
        }
        if (g_sts.status[index].missed_responses >= 3U)
        {
            g_sts.status[index].online = false;
            g_sts.status[index].temperature_limit_valid = false;
            g_sts.status[index].temperature_limit_attempts = 0U;
        }
    }
    g_sts.awaiting_response = false;
}

static void accept_packet (uint32_t now_ms)
{
    uint8_t const id = g_sts.packet[2];
    uint8_t const length = g_sts.packet[3];
    uint8_t const expected_data_length = (STS_READ_FEEDBACK == g_sts.pending_read_kind) ? STS_FEEDBACK_LENGTH : 1U;
    if ((id != g_sts.pending_id) || (length < (uint8_t) (expected_data_length + 2U)))
    {
        return;
    }
    if (g_sts.packet[g_sts.packet_expected - 1U] != checksum(g_sts.packet, 2U,
                                                             (uint8_t) (g_sts.packet_expected - 1U)))
    {
        return;
    }

    uint8_t const index = (uint8_t) (id - 1U);
    if (index >= STS3215_SERVO_COUNT)
    {
        return;
    }
    uint8_t const * data = &g_sts.packet[5];
    sts3215_status_t * status = &g_sts.status[index];
    if (STS_READ_TEMPERATURE_LIMIT == g_sts.pending_read_kind)
    {
        status->temperature_limit_c = data[0];
        status->temperature_limit_valid = true;
        g_sts.awaiting_response = false;
        return;
    }

    status->protocol_error = g_sts.packet[4];
    status->position = decode_signed(data[0], data[1], STS_DIRECTION_BIT);
    status->speed = decode_signed(data[2], data[3], STS_DIRECTION_BIT);
    status->load = decode_signed(data[4], data[5], 0x0400U);
    status->voltage_tenths = data[6];
    status->temperature_c = data[7];
    status->status_flags = data[9];
    status->current = decode_signed(data[13], data[14], STS_DIRECTION_BIT);
    status->missed_responses = 0U;
    status->online = true;
    status->last_seen_ms = now_ms;
    g_sts.awaiting_response = false;
}

static void parse_byte (uint8_t byte, uint32_t now_ms)
{
    if (0U == g_sts.packet_count)
    {
        if (STS_HEADER == byte)
        {
            g_sts.packet[g_sts.packet_count++] = byte;
        }
        return;
    }
    if (1U == g_sts.packet_count)
    {
        if (STS_HEADER == byte)
        {
            g_sts.packet[g_sts.packet_count++] = byte;
        }
        else
        {
            g_sts.packet_count = 0U;
        }
        return;
    }
    if (g_sts.packet_count >= sizeof(g_sts.packet))
    {
        g_sts.packet_count = 0U;
        return;
    }
    g_sts.packet[g_sts.packet_count++] = byte;
    if (4U == g_sts.packet_count)
    {
        uint8_t const length = g_sts.packet[3];
        g_sts.packet_expected = (uint8_t) (length + 4U);
        if ((length < 2U) || (g_sts.packet_expected > sizeof(g_sts.packet)))
        {
            g_sts.packet_count = 0U;
        }
    }
    else if ((g_sts.packet_expected > 0U) && (g_sts.packet_count == g_sts.packet_expected))
    {
        accept_packet(now_ms);
        g_sts.packet_count = 0U;
        g_sts.packet_expected = 0U;
    }
}

bool sts3215_init (void)
{
    memset(&g_sts, 0, sizeof(g_sts));
    if (FSP_SUCCESS != g_uart8.p_api->open(g_uart8.p_ctrl, g_uart8.p_cfg))
    {
        return false;
    }
    g_sts.uart_open = true;
    if (FSP_SUCCESS != g_uart8.p_api->read(g_uart8.p_ctrl, &g_sts.rx_byte, 1U))
    {
        (void) g_uart8.p_api->close(g_uart8.p_ctrl);
        g_sts.uart_open = false;
        return false;
    }
    (void) sts3215_set_torque(true);
    return true;
}

void sts3215_uart_callback (uart_callback_args_t const * args)
{
    if (UART_EVENT_RX_COMPLETE == args->event)
    {
        uint16_t const next = (uint16_t) ((g_sts.rx_head + 1U) % STS_RX_RING_SIZE);
        if (next != g_sts.rx_tail)
        {
            g_sts.rx_ring[g_sts.rx_head] = g_sts.rx_byte;
            g_sts.rx_head = next;
        }
        if (g_sts.uart_open)
        {
            (void) g_uart8.p_api->read(g_uart8.p_ctrl, &g_sts.rx_byte, 1U);
        }
    }
    else if (UART_EVENT_TX_COMPLETE == args->event)
    {
        g_sts.tx_busy = false;
    }
    else
    {
        g_sts.tx_busy = false;
    }
}

void sts3215_process (uint32_t now_ms)
{
    while (g_sts.rx_tail != g_sts.rx_head)
    {
        uint8_t const byte = g_sts.rx_ring[g_sts.rx_tail];
        g_sts.rx_tail = (uint16_t) ((g_sts.rx_tail + 1U) % STS_RX_RING_SIZE);
        parse_byte(byte, now_ms);
    }

    if (g_sts.awaiting_response && ((uint32_t) (now_ms - g_sts.request_time_ms) >= STS_RESPONSE_TIMEOUT_MS))
    {
        mark_timeout();
    }

    if (g_sts.torque_request_pending && !g_sts.awaiting_response && !g_sts.tx_busy)
    {
        if (send_torque_request(g_sts.torque_enable_request))
        {
            g_sts.torque_request_pending = false;
        }
        return;
    }

    if (!g_sts.awaiting_response && !g_sts.tx_busy &&
        ((uint32_t) (now_ms - g_sts.last_poll_ms) >= STS_POLL_INTERVAL_MS))
    {
        uint8_t const id = (uint8_t) (g_sts.poll_index + 1U);
        sts3215_status_t const * status = &g_sts.status[g_sts.poll_index];
        bool const read_limit = status->online && !status->temperature_limit_valid &&
                                (status->temperature_limit_attempts < 3U);
        bool const sent = read_limit ? send_temperature_limit_request(id, now_ms) : send_read_request(id, now_ms);
        if (sent)
        {
            g_sts.last_poll_ms = now_ms;
            g_sts.poll_index = (uint8_t) ((g_sts.poll_index + 1U) % STS3215_SERVO_COUNT);
        }
    }
}

bool sts3215_send_speeds (int16_t const speed[STS3215_SERVO_COUNT], uint8_t acceleration)
{
    if (g_sts.awaiting_response)
    {
        return false;
    }
    uint8_t packet[40];
    packet[0] = STS_HEADER;
    packet[1] = STS_HEADER;
    packet[2] = STS_BROADCAST_ID;
    packet[3] = 36U;
    packet[4] = STS_INST_SYNC_WRITE;
    packet[5] = STS_REG_ACCELERATION;
    packet[6] = 7U;
    for (uint8_t i = 0U; i < STS3215_SERVO_COUNT; i++)
    {
        uint8_t const offset = (uint8_t) (7U + (i * 8U));
        uint16_t const encoded = encode_signed(speed[i]);
        packet[offset] = (uint8_t) (i + 1U);
        packet[offset + 1U] = acceleration;
        packet[offset + 2U] = 0U;
        packet[offset + 3U] = 0U;
        packet[offset + 4U] = 0U;
        packet[offset + 5U] = 0U;
        packet[offset + 6U] = (uint8_t) encoded;
        packet[offset + 7U] = (uint8_t) (encoded >> 8);
    }
    packet[39] = checksum(packet, 2U, 39U);
    return send_packet(packet, sizeof(packet));
}

bool sts3215_send_zero (void)
{
    int16_t const speed[STS3215_SERVO_COUNT] = {0, 0, 0, 0};
    return sts3215_send_speeds(speed, ROBOT_SERVO_ACCELERATION);
}

bool sts3215_set_torque (bool enable)
{
    g_sts.torque_enable_request = enable;
    g_sts.torque_request_pending = true;
    if (!g_sts.awaiting_response && !g_sts.tx_busy && send_torque_request(enable))
    {
        g_sts.torque_request_pending = false;
    }
    return true;
}

sts3215_status_t const * sts3215_status (uint8_t index)
{
    return (index < STS3215_SERVO_COUNT) ? &g_sts.status[index] : NULL;
}

bool sts3215_all_online (void)
{
    for (uint8_t i = 0U; i < STS3215_SERVO_COUNT; i++)
    {
        if (!g_sts.status[i].online)
        {
            return false;
        }
    }
    return true;
}

bool sts3215_any_serious_fault (void)
{
    return sts3215_any_hard_fault() || sts3215_any_recoverable_fault();
}

bool sts3215_any_hard_fault (void)
{
    uint8_t const recoverable_mask = STS3215_STATUS_VOLTAGE | STS3215_STATUS_CURRENT | STS3215_STATUS_OVERLOAD;
    uint8_t const hard_status_mask = (uint8_t) ~(recoverable_mask | STS3215_STATUS_TEMPERATURE);
    for (uint8_t i = 0U; i < STS3215_SERVO_COUNT; i++)
    {
        sts3215_status_t const * status = &g_sts.status[i];
        uint8_t const flags = (uint8_t) (status->protocol_error | status->status_flags);
        if (status->online && ((status->temperature_c >= ROBOT_SERVO_FAULT_TEMP_C) ||
                              (0U != (flags & hard_status_mask))))
        {
            return true;
        }
    }
    return false;
}

bool sts3215_any_recoverable_fault (void)
{
    uint8_t const recoverable_mask = STS3215_STATUS_VOLTAGE | STS3215_STATUS_CURRENT | STS3215_STATUS_OVERLOAD;
    for (uint8_t i = 0U; i < STS3215_SERVO_COUNT; i++)
    {
        sts3215_status_t const * status = &g_sts.status[i];
        uint8_t const flags = (uint8_t) (status->protocol_error | status->status_flags);
        if (status->online && (0U != (flags & recoverable_mask)))
        {
            return true;
        }
    }
    return false;
}
