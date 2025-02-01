#include "config.h"
#include "imu.h"
#include "message_format.h"
#include "pid.h"
#include "port.h"
#include "state.h"
#include "uav_motor.h"
#include <Arduino.h>

// Finite State Machine for execution
State current_state;

// Finite State Machine for receiving information
bool is_receiving;

// Target attitude
// receive information from Serial2
String target_attitude_info;

// target attitude of uav
// in order of roll, pitch, yaw
float target_attitude[3];

float Kp[] = {1, 1, 1};
float Ki[] = {1, 1, 1};
float Kd[] = {1, 1, 1};

pid_controller uav_attitude_control(Kp, Ki, Kd);

void setup() {
  // 1. setup board
  board_setup();
  // 2. setup imu
  imu_setup();
  // 3. initialize received information
  target_attitude_info = "";
  // 4. reset pid controller
  uav_attitude_control.reset();
  // 5. initialize current state
  current_state = STATE_STOP;
  // 6. initialize information receiving state
  delay(1000);
}

void loop() {
  // 1. read sensor
  // 2. calculate Euler Angle
  calculate_euler_angle();

  // 3. receive information
  while (Serial2.available()) {
    char temp = Serial2.read();

    // start receiving
    if (temp == '<') {
      // throw old message
      target_attitude_info = "";
      // update state
      is_receiving = true;
    }

    // update string
    if (is_receiving) {
      target_attitude_info += temp;
    }

    // end receiving
    if (temp == '>') {
      is_receiving = false;

      // check whether information is correct
      if (target_attitude_info.startsWith("<") &&
          target_attitude_info.endsWith(">")) {
        continue;
      } else {
        // refresh
        target_attitude_info = "";
      }
    }
  }

  // 4. change state
  if (is_target_attitude_format(target_attitude_info)) {
    current_state = STATE_RUN;
  } else {
    current_state = STATE_STOP;
  }

  Serial1.println(current_state);
  Serial1.println(target_attitude_info);

  // 5. store targetted attitude
  if (target_attitude_info.length() > 0) {
    int begin = target_attitude_info.indexOf('<');
    int end = target_attitude_info.indexOf('>') + 1;
    if (begin != -1 && end != -1 && begin < end) {
      // assign value to attitude array
      sscanf(target_attitude_info.substring(begin, end).c_str(),
             "<y:%f,p:%f,r:%f>", &target_attitude[uav_yaw],
             &target_attitude[uav_pitch], &target_attitude[uav_roll]);
    }
  }

  // 6. according to target attitude, use pid_controller
  // 7. set motor speed
  switch (current_state) {
  case STATE_STOP:
    set_motor(STATE_STOP);
    break;

  case STATE_RUN:
    uav_attitude_control.compute(target_attitude, current_attitude);
    set_motor(STATE_RUN);
    break;

  default:
    set_motor(STATE_STOP);
    break;
  }

  static unsigned long last_time = 0;
  unsigned long current_time = millis();
  if (current_time - last_time < 20) {
    return;
  }
  last_time = current_time;
}