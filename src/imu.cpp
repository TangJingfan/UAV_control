#include "imu.h"
#include "port.h"
#include <Arduino.h>
#include <MPU6500_WE.h>
#include <Wire.h>

#define MPU6500_ADDR 0x68

MPU6500_WE MPU6500_on_drone = MPU6500_WE(MPU6500_ADDR);

void imu_setup() {}