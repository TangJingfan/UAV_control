#include "uav_motor.h"
#include "port.h"
#include "state.h"
#include <Arduino.h>

// # of motors
int motor_nums = 4;

// array of motor
int motor[] = {PWM1, PWM2, PWM3, PWM4};

// array of speed
// int of 0 - 255
int speed[] = {0, 0, 0, 0};

// array of throttle
// definition: minimum voltage for motor to make uav fly
int throttle[] = {80, 80, 80, 80};

/**
 * @brief set motor speed according to speed[]
 * @param current_state
 * * STOP when current_state == STATE_STOP
 * * RUN when current_state == STATE_RUN
 */
void set_motor(State current_state) {
  if (current_state == STATE_RUN) {
    for (int i = 0; i < motor_nums; i++) {
      analogWrite(motor[i], speed[i]);
    }
  } else {
    for (int j = 0; j < motor_nums; j++) {
      analogWrite(motor[j], 0);
    }
  }
}

void reset_throttle() {
  for (int i = 0; i < motor_nums; i++) {
    throttle[i] = 80;
  }
}