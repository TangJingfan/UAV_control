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
  pid_controller(float p[], float i[], float d[], float p_mild[],
                 float p_extreme[]);

  /**
   * @brief pid algorithm
   */
  void compute(float setpoint[], float measured_value[]);

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
  float kp[12], kp_mild[12], kp_extreme[12];
  float ki[12];
  float kd[12];
  /**
   * @brief other important data in pid algorithm
   */
  float error[3], prev_error[3], integral[3], derivative[3];
};

#endif