#include "pid.h"

pid_controller::pid_controller(float Kp, float Ki, float Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  prev_error = 0.0;
  integral = 0.0;
}

float pid_controller::compute(float setpoint, float actual, float dt) {
  float error = setpoint - actual;
  integral += error * dt;
  float derivative = (error - prev_error) / dt;

  float output = Kp * error + Ki * integral + Kd * derivative;
  prev_error = error;

  return output;
}
