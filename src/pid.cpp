#include "pid.h"
#include "imu.h"

pid_controller::pid_controller(float **p, float **i, float **d) {
  for (int j = 0; j < 3; j++) {
    prev_error[j] = 0;
    error[j] = 0;
    integral[j] = 0;
    derivative[j] = 0;

    for (int k = 0; k < 4; k++) {
      kp[k][j] = p[k][j];
      ki[k][j] = i[k][j];
      kd[k][j] = d[k][j];
    }
  }
}

void pid_controller::set_parameters(float **p, float **i, float **d) {
  for (int j = 0; j < 3; j++) {
    for (int k = 0; k < 4; k++) {
      this->kd[k][j] = d[k][j];
      this->ki[k][j] = i[k][j];
      this->kp[k][j] = p[k][j];
    }
  }
}

void pid_controller::compute(float setpoint[], float measured_value[]) {
  int result[4][3];
  for (int i = 0; i < 3; i++) {
    error[i] = setpoint[i] - measured_value[i];
    integral[i] += error[i];
    derivative[i] = error[i] - prev_error[i];
    prev_error[i] = error[i];
    for (int j = 0; j < 4; j++) {
      result[j][i] = kp[j][i] * error[i] + ki[j][i] * integral[i] +
                     kd[j][i] * derivative[i];
    }
  }
  // what matters is roll and pitch
  speed[0] = throttle[0] - result[0][uav_roll] - result[0][uav_pitch];
  speed[1] = throttle[1] + result[1][uav_roll] - result[1][uav_pitch];
  speed[2] = throttle[2] - result[2][uav_roll] + result[2][uav_pitch];
  speed[3] = throttle[3] + result[3][uav_roll] + result[3][uav_pitch];
}

void pid_controller::reset() {
  for (int j = 0; j < 3; j++) {
    this->derivative[j] = 0;
    this->integral[j] = 0;
    this->error[j] = 0;
    this->prev_error[j] = 0;
  }
}