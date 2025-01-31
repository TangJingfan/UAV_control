#ifndef UAV_MOTOR_H
#define UAV_MOTOR_H

#include "state.h"

/**
 * @brief number of motors
 * exactly 4.
 */
extern int motor_nums;

/**
 * @brief motor array
 */
extern int motor[];

/**
 * @brief speed array
 */
extern int speed[];

/**
 * @brief minimum speed needed to make uav fly
 */
extern int throttle[];

/**
 * @brief set motor speed according to speed[]
 * @param current_state
 * * STOP when current_state == STATE_STOP
 * * RUN when current_state == STATE_RUN
 */
void set_motor(State current_state);

#endif