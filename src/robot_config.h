#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

#define ROBOT_CONTROL_PERIOD_MS          (10U)
#define ROBOT_SERVO_COMMAND_PERIOD_MS    (20U)
#define ROBOT_TELEMETRY_PERIOD_MS        (200U)
#define ROBOT_LINK_TIMEOUT_MS            (800U)
#define ROBOT_SERVO_STARTUP_GRACE_MS     (1500U)
#define ROBOT_RECOVERY_CLEAR_MS           (500U)
#define ROBOT_RECOVERY_TIMEOUT_MS        (3000U)
#define ROBOT_RECOVERY_WINDOW_MS        (10000U)
#define ROBOT_RECOVERY_MAX_ATTEMPTS         (3U)
#define ROBOT_SERVO_FAULT_AUTO_CLEAR_MS   (2000U)
#define ROBOT_TEMP_NUMERIC_CONFIRM_MS       (500U)

#define ROBOT_WHEEL_MAX_SPEED            (2400)
#define ROBOT_WHEEL_DEADZONE              (25)
#define ROBOT_WHEEL_SLEW_PER_10MS         (80)
#define ROBOT_LIFT_MAX_SPEED              (800)
#define ROBOT_LIFT_POSITION_KP_NUM        (1)
#define ROBOT_LIFT_POSITION_KP_DEN        (3)
#define ROBOT_LIFT_HOLD_DEADBAND_COUNTS   (8)

/* Logical lift coordinates are positive upward. Keep these signs equal for a
 * normal STS3215 so motor motion and encoder feedback use the same polarity. */
#define ROBOT_LIFT_MOTOR_DIRECTION         (-1)
#define ROBOT_LIFT_FEEDBACK_DIRECTION      (-1)

#if ((ROBOT_LIFT_MOTOR_DIRECTION != 1) && (ROBOT_LIFT_MOTOR_DIRECTION != -1))
 #error "ROBOT_LIFT_MOTOR_DIRECTION must be 1 or -1"
#endif
#if ((ROBOT_LIFT_FEEDBACK_DIRECTION != 1) && (ROBOT_LIFT_FEEDBACK_DIRECTION != -1))
 #error "ROBOT_LIFT_FEEDBACK_DIRECTION must be 1 or -1"
#endif

#define ROBOT_SERVO_WARN_TEMP_C           (60U)
#define ROBOT_SERVO_FAULT_TEMP_C          (70U)
#define ROBOT_SERVO_ACCELERATION          (30U)

/* Change only these signs after the chassis is safely tested with its wheels off the ground. */
static const int8_t g_robot_wheel_direction[3] = {1, 1, 1};

#endif
