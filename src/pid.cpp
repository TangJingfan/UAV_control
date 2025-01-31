#include "pid.h"

pid_controller::pid_controller(float p[], float i[], float d[]) {
  for (int j = 0; j < 3; j++) {
    prev_error[j] = 0;
    error[j] = 0;
    integral[j] = 0;
    derivative[j] = 0;

    kp[j] = p[j];
    ki[j] = i[j];
    kd[j] = d[j];
  }
}

void pid_controller::set_parameters(float p[], float i[], float d[]) {
  for (int j = 0; j < 3; j++) {
    this->kd[j] = d[j];
    this->ki[j] = i[j];
    this->kp[j] = p[j];
  }
}

void pid_controller::compute(float setpoint[], float measured_value[]) {
  int result[3];
  for (int i = 0; i < 3; i++) {
    error[i] = setpoint[i] - measured_value[i];
    integral[i] += error[i];
    derivative[i] = error[i] - prev_error[i];
    prev_error[i] = error[i];
    result[i] = kp[i] * error[i] + ki[i] * integral[i] + kd[i] * derivative[i];
  }
  // what matters is roll and pitch
  speed[0] = throttle[0] - result[0] - result[1];
  speed[1] = throttle[1] + result[0] - result[1];
  speed[2] = throttle[2] - result[0] + result[1];
  speed[3] = throttle[3] + result[0] + result[1];
}