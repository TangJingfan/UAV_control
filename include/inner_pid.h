#ifndef INNER_PID_H
#define INNER_PID_H

/**
 * @brief inner pid controller class
 * * deal with speed[array]
 * * modify speed[] array
 */
class inner_pid_controller {
private:
  float kp[4], ki[4], kd[4];
  float error[4], prev_error[4], integral[4], derivative[4];

public:
  /**
   * @brief constructor
   */
  inner_pid_controller(float p[], float i[], float d[]);

  /**
   * @brief pid algorithm
   */
  void compute(int setpoint[], int measured_value[]);
};

#endif