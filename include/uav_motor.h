#ifndef UAV_MOTOR_H
#define UAV_MOTOR_H

extern int motor[];

extern int speed[];

void motor_set_speed();

void motor_set_speed(int speed_fwd, int speed_left, int speed_bwd,
                     int speed_right);

#endif