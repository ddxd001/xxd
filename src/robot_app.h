#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#include <stdbool.h>
#include <stdint.h>

typedef enum e_robot_state
{
    ROBOT_STATE_DISARMED_UNHOMED = 0,
    ROBOT_STATE_HOMING           = 1, /* Reserved by protocol v3; no longer entered. */
    ROBOT_STATE_DISARMED_HOMED   = 2,
    ROBOT_STATE_ARMED            = 3,
    ROBOT_STATE_FAULT            = 4,
    ROBOT_STATE_CALIBRATING      = 5, /* Reserved by protocol v3; no longer entered. */
    ROBOT_STATE_RECOVERING       = 6
} robot_state_t;

enum
{
    ROBOT_FAULT_LINK_TIMEOUT  = (1UL << 0),
    ROBOT_FAULT_SERVO_OFFLINE = (1UL << 1),
    ROBOT_FAULT_SERVO_STATUS  = (1UL << 2),
    ROBOT_FAULT_HOME_TIMEOUT  = (1UL << 3), /* Reserved by protocol v3. */
    ROBOT_FAULT_DRIVER_INIT   = (1UL << 4),
    ROBOT_FAULT_HOME_STALL    = (1UL << 5)  /* Reserved by protocol v3. */
};

bool     robot_app_init(void);
void     robot_app_process(void);
uint32_t robot_app_millis(void);

#endif
