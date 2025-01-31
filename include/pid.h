#ifndef PID_H
#define PID_H

#include "uav_motor.h"

/**
 * @brief pid controller class
 * * deal with roll and pitch at a time
 * * modify speed[] array
 */
class pid_controller {
public:
  /**
   * @brief constructor
   */
  pid_controller(float p[], float i[], float d[]);

  /**
   * @brief pid algorithm
   */
  void compute(float setpoint[], float measured_value[]);

  /**
   * @brief change parameters during main loop
   */
  void set_parameters(float p[], float i[], float d[]);

private:
  /**
   * @brief pid constant array
   * * follow the order of roll, pitch, yaw
   */
  float kp[3], ki[3], kd[3];
  /**
   * @brief other important data in pid algorithm
   */
  float error[3], prev_error[3], integral[3], derivative[3];
};

#endif