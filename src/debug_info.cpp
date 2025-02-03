#include "debug_info.h"
#include "config.h"
#include "imu.h"
#include "port.h"
#include "uav_motor.h"
#include <Arduino.h>

void print_info() {
  Serial1.print("roll:");
  Serial1.println(current_attitude[uav_roll]);
  Serial1.print("pitch:");
  Serial1.println(current_attitude[uav_pitch]);
  for (int y = 0; y < motor_nums; y++) {
    Serial1.print("motor ");
    Serial1.print(y + 1);
    Serial1.print(": ");
    Serial1.println(speed[y]);
    Serial1.print("ideal motor ");
    Serial1.print(y + 1);
    Serial1.print(": ");
    Serial1.println(target_speed[y]);
  }
  Serial1.println("");
}
