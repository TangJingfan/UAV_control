#ifndef IMU_H
#define IMU_H

#include <MPU6500_WE.h>

/**
 * @brief basic setup of imu MPU6500
 */
void imu_setup();

/**
 * @brief get raw message from sensor
 * TODO: read message
 */
void read_sensor();

/**
 * @brief calcualte euler angle by using raw data
 * TODO: calculation
 */
void calculate_euler_angle();

#endif