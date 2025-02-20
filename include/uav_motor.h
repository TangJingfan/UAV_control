#ifndef UAV_MOTOR_H
#define UAV_MOTOR_H

#include "state.h"

/**
 * @brief unbalanced direction
 */
enum direction { negative_roll, positive_roll, negative_pitch, positive_pitch };

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
 * for inner pid and motor
 */
extern int speed[];

/**
 * @brief target speed array
 * for outer pid
 */
extern int target_speed[];

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

/**
 * @brief after compensation, restore the initial value of throttle
 */
void reset_throttle();

const double motor_factor = 0.88;

const int motor_compensation = 9;

const int motor_least_speed = 10;

const int motor_greatest_speed = 200;

#endif