#ifndef STS3215_H
#define STS3215_H

#include <stdbool.h>
#include <stdint.h>

#include "r_uart_api.h"

#define STS3215_SERVO_COUNT (4U)

enum
{
    STS3215_STATUS_VOLTAGE     = (1U << 0),
    STS3215_STATUS_SENSOR      = (1U << 1),
    STS3215_STATUS_TEMPERATURE = (1U << 2),
    STS3215_STATUS_CURRENT     = (1U << 3),
    STS3215_STATUS_ANGLE       = (1U << 4),
    STS3215_STATUS_OVERLOAD    = (1U << 5)
};

typedef struct st_sts3215_status
{
    bool     online;
    uint8_t  missed_responses;
    uint8_t  protocol_error;
    uint8_t  status_flags;
    uint8_t  temperature_c;
    uint8_t  temperature_limit_c;
    uint8_t  temperature_limit_attempts;
    bool     temperature_limit_valid;
    uint8_t  voltage_tenths;
    int16_t  position;
    int16_t  speed;
    int16_t  load;
    int16_t  current;
    uint32_t last_seen_ms;
} sts3215_status_t;

bool sts3215_init(void);
void sts3215_uart_callback(uart_callback_args_t const * args);
void sts3215_process(uint32_t now_ms);
bool sts3215_send_speeds(int16_t const speed[STS3215_SERVO_COUNT], uint8_t acceleration);
bool sts3215_send_zero(void);
bool sts3215_set_torque(bool enable);
sts3215_status_t const * sts3215_status(uint8_t index);
bool sts3215_all_online(void);
bool sts3215_any_serious_fault(void);
bool sts3215_any_hard_fault(void);
bool sts3215_any_recoverable_fault(void);

#endif
