#include "imu.h"
#include "port.h"
#include <Arduino.h>
#include <MPU6500_WE.h>
#include <Wire.h>
#include <cmath>

// MPU6500 address
#define MPU6500_ADDR 0x68

// an object of MPU6500
MPU6500_WE MPU6500_on_drone = MPU6500_WE(MPU6500_ADDR);

// attitude array of uav
float current_attitude[] = {0, 0, 0};

void imu_setup() {
  // 1. set I2C pin
  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin();

  // 2. init MPU6500
  if (!MPU6500_on_drone.init()) {
    Serial2.println("MPU6500 does not respond...");
  } else {
    Serial2.println("MPU6500 is connected!");
  }

  // 3. calibrating imu
  Serial2.println(
      "Position you MPU6500 flat and don't move it - calibrating...");
  // wait for sensor to be stable
  delay(1000);
  MPU6500_on_drone.autoOffsets();
  Serial2.println("Done!");

  // 4. setup gyroscope
  MPU6500_on_drone.enableGyrDLPF();
  MPU6500_on_drone.setGyrDLPF(MPU6500_DLPF_6);
  // sampling frequency: 50 Hz
  MPU6500_on_drone.setSampleRateDivider(19);
  MPU6500_on_drone.setGyrRange(MPU6500_GYRO_RANGE_250);

  // 5. setup accelerometer
  MPU6500_on_drone.setAccRange(MPU6500_ACC_RANGE_2G);
  MPU6500_on_drone.enableAccDLPF(true);
  MPU6500_on_drone.setAccDLPF(MPU6500_DLPF_6);
}

void calculate_euler_angle() {
  // 1. use getAngles()
  xyzFloat angle = MPU6500_on_drone.getAngles();

  // 2. assign values
  current_attitude[imu_x_axis] = angle.x;
  current_attitude[imu_y_axis] = angle.y;
  current_attitude[imu_z_axis] = angle.z;
}