#include "imu.h"
#include "port.h"
#include "uav_motor.h"
#include <Arduino.h>
#include <MPU6500_WE.h>
#include <Wire.h>
#include <cmath>

#define MPU6500_ADDR 0x68

MPU6500_WE MPU6500_on_drone = MPU6500_WE(MPU6500_ADDR);

void imu_setup() {
  // set I2C pin
  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin();

  // init MPU6500
  if (!MPU6500_on_drone.init()) {
    Serial2.println("MPU6500 does not respond...");
  } else {
    Serial2.println("MPU6500 is connected!");
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
