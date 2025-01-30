#include "config.h"
#include "port.h"
#include <Arduino.h>

int motor[] = {PWM1, PWM2, PWM3, PWM4};

int speed[] = {-1, -1, -1, -1};

int current_yaw_pitch_roll[] = {0, 0, 0};
int target_yaw_pitch_roll[] = {0, 0, 0};
int error_yaw_pitch_roll[] = {0, 0, 0};

int throttle = 150;
int min_motor_speed = 0;
int max_motor_speed = 255;

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

void update_motor_speeds(float roll_output, float pitch_output,
                         float yaw_output) {
  // calculate speed of each motor
  int M1 = static_cast<int>(throttle - roll_output - pitch_output + yaw_output);
  int M2 = static_cast<int>(throttle + roll_output - pitch_output - yaw_output);
  int M3 = static_cast<int>(throttle - roll_output + pitch_output - yaw_output);
  int M4 = static_cast<int>(throttle + roll_output + pitch_output + yaw_output);

  // constrain voltage
  speed[0] = constrain(M1, min_motor_speed, max_motor_speed);
  speed[1] = constrain(M2, min_motor_speed, max_motor_speed);
  speed[2] = constrain(M3, min_motor_speed, max_motor_speed);
  speed[3] = constrain(M4, min_motor_speed, max_motor_speed);
}