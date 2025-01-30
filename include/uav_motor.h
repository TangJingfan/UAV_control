#ifndef UAV_MOTOR_H
#define UAV_MOTOR_H

extern int motor[];
extern int speed[];

extern int current_yaw_pitch_roll[];
extern int target_yaw_pitch_roll[];
extern int error_yaw_pitch_roll[];

extern float throttle;
extern float min_motor_speed;
extern float max_motor_speed;

void motor_set_speed();

void motor_set_speed(int speed_fwd, int speed_left, int speed_bwd,
                     int speed_right);

void update_motor_speeds(float roll_output, float pitch_output,
                         float yaw_output);
#endif