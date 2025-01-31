#ifndef IMU_H
#define IMU_H

#include <MPU6500_WE.h>

#define imu_x_axis 0
#define imu_y_axis 1
#define imu_z_axis 2

#define uav_roll 0
#define uav_pitch 1
#define uav_yaw 2

/**
 * we get roll angle from x_axis
 * we get pitch angle from y_axis
 */
extern float current_attitude[];

/**
 * @brief basic setup of imu MPU6500
 */
void imu_setup();

/**
 * @brief calculate Euler angle
 * ! the value of yaw is not reliable
 */
void calculate_euler_angle();

#endif