#ifndef UAV_MOTOR_H
#define UAV_MOTOR_H

#include "state.h"

extern int motor_nums;

extern int motor[];

extern int speed[];

extern int throttle[];

void set_motor(State current_state);

#endif