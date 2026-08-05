#include "robot_kinematics.h"

#include "robot_config.h"

static int32_t absolute_i32 (int32_t value)
{
    return (value < 0) ? -value : value;
}

void robot_kinematics_calculate (int16_t vx, int16_t vy, int16_t omega, int16_t output[3])
{
    /* Fixed-point 120-degree omni matrix. Inputs and intermediate values use a scale of 1000. */
    int32_t wheel[3];
    wheel[0] = (int32_t) vy + (int32_t) omega;
    wheel[1] = ((-866 * (int32_t) vx) / 1000) - ((500 * (int32_t) vy) / 1000) + (int32_t) omega;
    wheel[2] = ((866 * (int32_t) vx) / 1000) - ((500 * (int32_t) vy) / 1000) + (int32_t) omega;

    int32_t maximum = 1000;
    for (uint8_t i = 0U; i < 3U; i++)
    {
        int32_t const magnitude = absolute_i32(wheel[i]);
        if (magnitude > maximum)
        {
            maximum = magnitude;
        }
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        int32_t value = (wheel[i] * ROBOT_WHEEL_MAX_SPEED) / maximum;
        value *= g_robot_wheel_direction[i];
        if (absolute_i32(value) < ROBOT_WHEEL_DEADZONE)
        {
            value = 0;
        }
        output[i] = (int16_t) value;
    }
}

void robot_kinematics_ramp (robot_wheel_ramp_t * ramp, int16_t const target[3], int16_t output[3])
{
    for (uint8_t i = 0U; i < 3U; i++)
    {
        int32_t delta = (int32_t) target[i] - ramp->speed[i];
        if (delta > ROBOT_WHEEL_SLEW_PER_10MS)
        {
            delta = ROBOT_WHEEL_SLEW_PER_10MS;
        }
        else if (delta < -ROBOT_WHEEL_SLEW_PER_10MS)
        {
            delta = -ROBOT_WHEEL_SLEW_PER_10MS;
        }
        ramp->speed[i] = (int16_t) ((int32_t) ramp->speed[i] + delta);
        output[i] = ramp->speed[i];
    }
}

void robot_kinematics_stop (robot_wheel_ramp_t * ramp)
{
    for (uint8_t i = 0U; i < 3U; i++)
    {
        ramp->speed[i] = 0;
    }
}
