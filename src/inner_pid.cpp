#include "inner_pid.h"
#include "uav_motor.h"
#include <Arduino.h>

inner_pid_controller::inner_pid_controller(float p[], float i[], float d[]) {
  for (int j = 0; j < motor_nums; j++) {
    kp[j] = p[j];
    ki[j] = i[j];
    kd[j] = d[j];
    prev_error[j] = 0;
    error[j] = 0;
    integral[j] = 0;
    derivative[j] = 0;
  }
}

void inner_pid_controller::compute(int setpoint[], int measured_value[]) {
  for (int j = 0; j < motor_nums; j++) {
    error[j] = setpoint[j] - measured_value[j];
    integral[j] += error[j];
    derivative[j] = error[j] - prev_error[j];
    prev_error[j] = error[j];
    speed[j] += kp[j] * error[j] + ki[j] * integral[j] + kd[j] * derivative[j],
        motor_least_speed, motor_greatest_speed;
  }
}