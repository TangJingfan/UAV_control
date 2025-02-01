#include "pid.h"
#include "imu.h"

pid_controller::pid_controller(float p[], float i[], float d[],
                               float p_extreme[]) {
  for (int j = 0; j < 3; j++) {
    prev_error[j] = 0;
    error[j] = 0;
    integral[j] = 0;
    derivative[j] = 0;

    for (int k = 0; k < 4; k++) {
      kp[3 * k + j] = p[3 * k + j];
      ki[3 * k + j] = i[3 * k + j];
      kd[3 * k + j] = d[3 * k + j];
      kp_extreme[3 * k + j] = p_extreme[3 * k + j];
    }
  }
}

void pid_controller::set_parameters(float p[], float i[], float d[]) {
  for (int j = 0; j < 3; j++) {
    for (int k = 0; k < 4; k++) {
      kp[3 * k + j] = p[3 * k + j];
      ki[3 * k + j] = i[3 * k + j];
      kd[3 * k + j] = d[3 * k + j];
    }
  }
}

void pid_controller::compute(float setpoint[], float measured_value[]) {
  int roll_result[4], pitch_result[4], yaw_result[4];
  for (int i = 0; i < 3; i++) {
    /**
     * * in order of roll, pitch, yaw
     */
    error[i] = setpoint[i] - measured_value[i];
    integral[i] += error[i];
    derivative[i] = error[i] - prev_error[i];
    prev_error[i] = error[i];

    // ! in case of integral windup
    if (error[i] > 90) {
      error[i] = 90;
    } else if (error[i] < -90) {
      error[i] = -90;
    }
  }
  for (int j = 0; j < 4; j++) {
    if (error[uav_roll] < 10 || error[uav_roll] > 30) {
      roll_result[j] = kp_extreme[j * 3] * error[uav_roll] +
                       ki[j * 3] * integral[uav_roll] +
                       kd[j * 3] * derivative[uav_roll];
    } else {
      roll_result[j] = kp[j * 3] * error[uav_roll] +
                       ki[j * 3] * integral[uav_roll] +
                       kd[j * 3] * derivative[uav_roll];
    }

    pitch_result[j] = kp[j * 3 + 1] * error[uav_pitch] +
                      ki[j * 3 + 1] * integral[uav_pitch] +
                      kd[j * 3 + 1] * derivative[uav_pitch];
    yaw_result[j] = kp[j * 3 + 2] * error[uav_yaw] +
                    ki[j * 3 + 2] * integral[uav_yaw] +
                    kd[j * 3 + 2] * derivative[uav_yaw];
  }

  /**
   *  (m2)   (m1)
   *      \ /
   *       x
   *      / \
   *  (m4)   (m3)
   *
   *
   *
   *   x ------(x) z
   *            |
   *            |
   *            y
   */
  // what matters is roll and pitch
  speed[0] = throttle[0] + roll_result[0] + pitch_result[0];
  speed[1] = throttle[1] + roll_result[1] - pitch_result[1];
  speed[2] = throttle[2] - roll_result[3] + pitch_result[3];
  speed[3] = throttle[3] - roll_result[2] - pitch_result[2];
}

void pid_controller::reset() {
  for (int j = 0; j < 3; j++) {
    this->derivative[j] = 0;
    this->integral[j] = 0;
    this->error[j] = 0;
    this->prev_error[j] = 0;
  }
}