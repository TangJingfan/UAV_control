#include "config.h"
#include "port.h"
#include <Arduino.h>

int motor[] = {PWM1, PWM2, PWM3, PWM4};

int speed[] = {-1, -1, -1, -1};

void motor_set_speed() {
  for (int i = 0; i < motor_num; i++) {
    analogWrite(motor[i], speed[i]);
  }
}

void motor_set_speed(int speed_fwd, int speed_left, int speed_bwd,
                     int speed_right) {
  int speed_temp[] = {speed_fwd, speed_left, speed_bwd, speed_right};
  for (int i = 0; i < motor_num; i++) {
    analogWrite(motor[i], speed_temp[i]);
  }
}