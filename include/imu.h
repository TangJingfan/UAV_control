#ifndef IMU_H
#define IMU_H

#include <MPU6500_WE.h>

void imu_setup();

void calculate_euler_angle();

extern MPU6500_WE MPU6500_on_drone;

extern float last_gyro_roll, last_gyro_pitch;
extern float last_acc_roll, last_acc_pitch;

extern float rad2deg;

#endif