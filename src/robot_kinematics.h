#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <stdint.h>

typedef struct st_robot_wheel_ramp
{
    int16_t speed[3];
} robot_wheel_ramp_t;

void robot_kinematics_calculate(int16_t vx, int16_t vy, int16_t omega, int16_t output[3]);
void robot_kinematics_ramp(robot_wheel_ramp_t * ramp, int16_t const target[3], int16_t output[3]);
void robot_kinematics_stop(robot_wheel_ramp_t * ramp);

#endif
