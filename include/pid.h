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
  pid_controller(float p[], float i[], float d[], float p_small_angle[]);

  /**
   * @brief pid algorithm
   */
  void compute(float setpoint[], float measured_value[]);

  /**
   * @brief change parameters during main loop
   */
  void set_parameters(float p[], float i[], float d[]);

  /**
   * @brief reset all terms
   */
  void reset();

private:
  /**
   * @brief pid constant array
   * * follow the order of
   * * 0-2: {m1: roll, pitch, yaw}
   * * 3-5: {m2: roll, pitch, yaw}
   * * 6-8: {m3: roll, pitch, yaw}
   * * 9-11: {m4: roll, pitch, yaw}
   */
  float kp[12], ki[12], kd[12];
  float kp_small_angle[12];
  /**
   * @brief other important data in pid algorithm
   */
  float error[3], prev_error[3], integral[3], derivative[3];
};

#endif