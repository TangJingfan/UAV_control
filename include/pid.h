class pid_controller {
private:
  float Kp, Ki, Kd;
  float prev_error, integral;

public:
  pid_controller(float Kp, float Ki, float Kd);

  float compute(float setpoint, float actual, float dt);
};
