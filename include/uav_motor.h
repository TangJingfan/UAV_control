#ifndef UAV_MOTOR_H
#define UAV_MOTOR_H

extern int motor[];

extern int speed[];

extern int yaw_pitch_roll[];

extern int target_yaw_pitch_roll[];

void motor_set_speed();

void motor_set_speed(int speed_fwd, int speed_left, int speed_bwd,
                     int speed_right);

#endif