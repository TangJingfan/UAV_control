#include "imu.h"
#include "port.h"
#include "uav_motor.h"
#include <Arduino.h>
#include <MPU6500_WE.h>
#include <Wire.h>
#include <cmath>

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
float now = 0, last_time = 0, dt = 0;

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

float yaw = 0, roll = 0, pitch = 0;

static void initialize_kalman_filter() {
  e_P[0][0] = 1;
  e_P[0][1] = 0;
  e_P[1][0] = 0;
  e_P[1][1] = 1;
  k_k[0][0] = 0;
  k_k[0][1] = 0;
  k_k[1][0] = 0;
  k_k[1][1] = 0;
}

void imu_setup() {
  // init kalman filter
  initialize_kalman_filter();

  // set I2C pin
  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin();

  // init MPU6500
  if (!MPU6500_on_drone.init()) {
    Serial2.println("MPU6500 does not respond");
  } else {
    Serial2.println("MPU6500 is connected");
  }

  // calibrating imu
  Serial2.println(
      "Position you MPU6500 flat and don't move it - calibrating...");
  // wait for sensor to be stable
  delay(1000);
  MPU6500_on_drone.autoOffsets();
  Serial2.println("Done!");

  // setup gyroscope
  MPU6500_on_drone.enableGyrDLPF();
  MPU6500_on_drone.setGyrDLPF(MPU6500_DLPF_6);
  MPU6500_on_drone.setSampleRateDivider(5);
  MPU6500_on_drone.setGyrRange(MPU6500_GYRO_RANGE_250);

  // setup accelerometer
  MPU6500_on_drone.setAccRange(MPU6500_ACC_RANGE_2G);
  MPU6500_on_drone.enableAccDLPF(true);
  MPU6500_on_drone.setAccDLPF(MPU6500_DLPF_6);
}

void calculate_euler_angle() {
  // get accelerometer
  xyzFloat gValue = MPU6500_on_drone.getGValues();
  // get gyroscope
  xyzFloat gyr = MPU6500_on_drone.getGyrValues();

  now = millis();
  dt = (now - last_time) / 1000.0;
  last_time = now;

  // calculate angular velocity on roll and pitch
  roll_v = gyr.x + ((sin(k_pitch) * sin(k_roll)) / cos(k_pitch)) * gyr.y +
           ((sin(k_pitch) * cos(k_roll)) / cos(k_pitch)) * gyr.z;
  pitch_v = cos(k_roll) * gyr.y - sin(k_roll) * gyr.z;
  // priori roll angle and pitch angle
  gyro_roll = k_roll + dt * roll_v;
  gyro_pitch = k_pitch + dt * pitch_v;

  // calculate priori converiance matrix
  e_P[0][0] = e_P[0][0] + 0.0025;
  e_P[0][1] = e_P[0][1] + 0;
  e_P[1][0] = e_P[1][0] + 0;
  e_P[1][1] = e_P[1][1] + 0.0025;

  // calcualte karman gain matrix
  k_k[0][0] = e_P[0][0] / (e_P[0][0] + 0.3);
  k_k[0][1] = 0;
  k_k[1][0] = 0;
  k_k[1][1] = e_P[1][1] / (e_P[1][1] + 0.3);

  acc_roll = atan(gValue.y / gValue.z) * rad2deg;
  acc_pitch = -1 * atan(gValue.x / sqrt(sq(gValue.y) + sq(gValue.z))) * rad2deg;

  k_roll = gyro_roll + k_k[0][0] * (acc_roll - gyro_roll);
  k_pitch = gyro_pitch + k_k[1][1] * (acc_pitch - gyro_pitch);

  roll = 3.0 * k_roll;
  pitch = 3.0 * k_pitch;
  yaw += gyr.z * dt;
  current_yaw_pitch_roll[0] = yaw;
  current_yaw_pitch_roll[1] = pitch;
  current_yaw_pitch_roll[2] = roll;
}
