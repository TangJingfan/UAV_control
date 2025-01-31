#include "config.h"
#include "imu.h"
#include "pid.h"
#include "port.h"
#include "state.h"
#include "uav_motor.h"
#include <Arduino.h>

State current_state = STATE_STOP;

void setup() {
  // 1. setup board
  board_setup();
  // 2. setup imu
  imu_setup();
}

void loop() {
  // 1. read sensor
  read_sensor();

  // 2. calculate Euler Angle
  calculate_euler_angle();

  // 3. according to target attitude, calculate set points
  calculate_setpoints();

  // 4. calculate errors
  calculate_errors();

  // 5. according to target attitude, use pid_controller
  // 6. set motor speed
  switch (current_state) {
  case STATE_RUN:
    pid_controller();
    set_motor_speed();
    break;

  default:
    set_motor_speed();
    break;
  }

  static unsigned long last_time = 0;
  unsigned long current_time = millis();
  if (current_time - last_time < 20) {
    return;
  }
  last_time = current_time;
}