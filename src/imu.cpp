#include "imu.h"
#include "port.h"
#include <Arduino.h>
#include <MPU6500_WE.h>
#include <Wire.h>

#define MPU6500_ADDR 0x68

MPU6500_WE MPU6500_on_drone = MPU6500_WE(MPU6500_ADDR);

// calculate offset
// 计算偏移量
// a variable to keep record the time of loops
// 用来记录循环时间的变量
float i;

// parameters
// 参数
// radian to angle conversion factor
// 弧度到角度的换算系数
float rad2deg = 57.29578;

// angular speed on x-axis and y-axis
// x轴和y轴上的角速度
float roll_v = 0, pitch_v = 0;

// define differential time
// 定义微分时间
float now = 0, lasttime = 0, dt = 0;

// three state
// 三个状态
// priori state, observed state, optimal estimation state
// 先验状态，观测状态，最优估计状态
// angle calculated by gyroscope integration, which is priori state
// 通过陀螺仪积分计算出的角度，作为先验状态
float gyro_roll = 0, gyro_pitch = 0;
// angle observed by accelerometer, which is observation status
// 通过加速度计观测到的角度，作为观测状态
float acc_roll = 0, acc_pitch = 0;
// best-estimated angle by Kalman filter
// 卡尔曼滤波计算出的最优估计角度，作为最优估计状态
float k_roll = 0, k_pitch = 0;

// Error covariance matrix P
// 误差协方差矩阵P
// Error covariance matrix, where e_P is both the priori estimate of P and the
// final updated P.
// 误差协方差矩阵，这里的e_P既是先验估计的P，也是最后更新的P
float e_P[2][2] = {{1, 0}, {0, 1}};

// Kalman gain matrix K
// 卡尔曼增益矩阵K
// here matrix is a 2 * 2 square matrix
// 这里的卡尔曼增益矩阵K是一个2X2的方阵
float k_k[2][2] = {{0, 0}, {0, 0}};

void imu_setup() {}