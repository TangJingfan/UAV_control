#ifndef PID_H
#define PID_H

#include "uav_motor.h"

class pid_controller {
public:
  pid_controller(float p[], float i[], float d[]);

  void compute(float setpoint[], float measured_value[]);

  void set_parameters(float p[], float i[], float d[]);

private:
  float kp[3], ki[3], kd[3];
  float error[3], prev_error[3], integral[3], derivative[3];
};

#endif