#include "pid.h"
#include "imu.h"

pid_controller::pid_controller(float p[], float i[], float d[], float p_mild[],
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
      kp_mild[3 * k + j] = p_mild[3 * k + j];
      kp_extreme[3 * k + j] = p_extreme[3 * k + j];
    }
  }
}

void pid_controller::debug_info(int i) {
  Serial1.print("error ");
  Serial1.print(i);
  Serial1.print(": ");
  Serial1.println(error[i]);
  Serial1.print("previous error ");
  Serial1.print(i);
  Serial1.print(": ");
  Serial1.println(prev_error[i]);
  Serial1.print("integral ");
  Serial1.print(i);
  Serial1.print(": ");
  Serial1.println(integral[i]);
  Serial1.print("derivative ");
  Serial1.print(i);
  Serial1.print(": ");
  Serial1.println(derivative[i]);
}

void pid_controller::compute(float setpoint[], float measured_value[]) {
  int roll_result[4], pitch_result[4], yaw_result[4];
  for (int i = 0; i < 3; i++) {
    /**
     * * in order of roll, pitch, yaw
     */
    error[i] = setpoint[i] - measured_value[i];
    if (fabs(error[i]) < 3) {
      error[i] = -error[i];
    }

    integral[i] += error[i];
    derivative[i] = error[i] - prev_error[i];
    prev_error[i] = error[i];
    // ! DEBUG
    debug_info(i);
  }

  for (int j = 0; j < 4; j++) {
    // * non-linear pid control
    // * roll angle
    if (fabs(error[uav_roll]) < 7) {
      roll_result[j] = kp_extreme[j * 3] * error[uav_roll] +
                       ki[j * 3] * integral[uav_roll] +
                       kd[j * 3] * derivative[uav_roll];
    } else if (fabs(error[uav_roll]) < 20) {
      roll_result[j] = kp_mild[j * 3] * error[uav_roll] +
                       ki[j * 3] * integral[uav_roll] +
                       kd[j * 3] * derivative[uav_roll];
      pitch_result[j] = kp_mild[j * 3 + 1] * error[uav_pitch] +
                        ki[j * 3 + 1] * integral[uav_pitch] +
                        kd[j * 3 + 1] * derivative[uav_pitch];
    } else {
      roll_result[j] = kp[j * 3] * error[uav_roll] +
                       ki[j * 3] * integral[uav_roll] +
                       kd[j * 3] * derivative[uav_roll];
    }

    // * pitch angle
    if (fabs(error[uav_pitch]) < 7) {
      pitch_result[j] = kp_extreme[j * 3 + 1] * error[uav_pitch] +
                        ki[j * 3 + 1] * integral[uav_pitch] +
                        kd[j * 3 + 1] * derivative[uav_pitch];
    } else if (fabs(error[uav_pitch]) < 20) {
      pitch_result[j] = kp_mild[j * 3 + 1] * error[uav_pitch] +
                        ki[j * 3 + 1] * integral[uav_pitch] +
                        kd[j * 3 + 1] * derivative[uav_pitch];
    } else {
      pitch_result[j] = kp[j * 3 + 1] * error[uav_pitch] +
                        ki[j * 3 + 1] * integral[uav_pitch] +
                        kd[j * 3 + 1] * derivative[uav_pitch];
    }

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

  // * throttle compensation
  if (current_attitude[uav_roll] > 10) {
    throttle[0] += motor_compensation;
    throttle[1] += motor_compensation;
    throttle[2] -= motor_compensation;
    throttle[3] -= motor_compensation;
  }
  if (current_attitude[uav_roll] < -10) {
    throttle[0] -= motor_compensation;
    throttle[1] -= motor_compensation;
    throttle[2] += motor_compensation;
    throttle[3] += motor_compensation;
  }
  if (current_attitude[uav_pitch] > 10) {
    throttle[0] += motor_compensation;
    throttle[1] -= motor_compensation;
    throttle[2] += motor_compensation;
    throttle[3] -= motor_compensation;
  }
  if (current_attitude[uav_pitch] < -10) {
    throttle[0] -= motor_compensation;
    throttle[1] += motor_compensation;
    throttle[2] -= motor_compensation;
    throttle[3] += motor_compensation;
  }

  // what matters is roll and pitch
  target_speed[0] = constrain(throttle[0] - roll_result[0] - pitch_result[0],
                              motor_least_speed, motor_greatest_speed);
  target_speed[1] = constrain(throttle[1] - roll_result[1] + pitch_result[1],
                              motor_least_speed, motor_greatest_speed);
  target_speed[2] = constrain(throttle[2] + roll_result[2] - pitch_result[2],
                              motor_least_speed, motor_greatest_speed);
  target_speed[3] = constrain(throttle[3] + roll_result[3] + pitch_result[3],
                              motor_least_speed, motor_greatest_speed);

  // * special case
  if (fabs(error[uav_pitch]) > 13 && fabs(error[uav_roll]) > 13) {
    target_speed[0] =
        constrain(throttle[0] - 1.15 * roll_result[0] - 1.15 * pitch_result[0],
                  motor_least_speed, motor_greatest_speed + 20);
    target_speed[1] =
        constrain(throttle[1] - 1.15 * roll_result[1] + 1.15 * pitch_result[1],
                  motor_least_speed, motor_greatest_speed + 20);
    target_speed[2] =
        constrain(throttle[2] + 1.15 * roll_result[2] - 1.15 * pitch_result[2],
                  motor_least_speed, motor_greatest_speed + 20);
    target_speed[3] =
        constrain(throttle[3] + 1.15 * roll_result[3] + 1.15 * pitch_result[3],
                  motor_least_speed, motor_greatest_speed + 20);
  }
}

void pid_controller::reset() {
  for (int j = 0; j < 3; j++) {
    this->derivative[j] = 0;
    this->integral[j] = 0;
    this->error[j] = 0;
    this->prev_error[j] = 0;
  }
}