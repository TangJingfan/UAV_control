#ifndef IMU_H
#define IMU_H

#include <MPU6500_WE.h>

/**
 * @brief macro definition here
 * * work as index of array
 * * all array should follow this order
 */
#define imu_x_axis 0
#define imu_y_axis 1
#define imu_z_axis 2

#define uav_roll 0
#define uav_pitch 1
#define uav_yaw 2

/**
 * @brief get current attitude
 * * roll angle value from x_axis
 * * raised +x-axis leads to positive roll
 *
 * * pitch angle value from y_axis
 * * raised +y-axis leads to positive pitch
 */
extern float current_attitude[];

/**
 * @brief basic setup of imu MPU6500
 */
void imu_setup();

/**
 * @brief calculate Euler angle
 * ! the value of yaw is not reliable
 * ! yaw is not addressed in this project with mpu6500
 * ! please use mpu9250 instead
 */
void calculate_euler_angle();

#endif