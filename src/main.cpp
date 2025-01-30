#include "config.h"
#include "imu.h"
#include "port.h"
#include "uav_motor.h"
#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>

enum state { waiting, executing };

state current_state = waiting;
String received_command = "";
String executing_command = "";

void setup() {
  config_setup();
  imu_setup();
}

void loop() {
  while (Serial2.available() > 0) {
    char temp_received = char(Serial2.read());
    received_command += temp_received;
    current_state = executing;
  }

  /*
    use Euler angle to represent UAV's attitude
    basic format: <y:0,p:0,r:0>
    use speed to control uav
    basic format: <m1,m2,m3,m4>
  */

  // calculate Euler angle by MPU6500
  calculate_euler_angle();

  Serial2.print("yaw: ");
  Serial2.println(yaw_pitch_roll[0]);
  Serial2.print("pitch: ");
  Serial2.println(yaw_pitch_roll[1]);
  Serial2.print("roll: ");
  Serial2.println(yaw_pitch_roll[2]);

  switch (current_state) {
  case waiting:
    motor_set_speed();
    break;
  case executing:
    // update executing command
    executing_command = received_command;
    received_command = "";
    // assign speed to speed array
    if (executing_command.length() > 0) {
      int begin = executing_command.indexOf('<');
      int end = executing_command.indexOf('>') + 1;
      if (begin != -1 && end != -1 && begin < end) {
        // assign value to attitude array
        /*
          sscanf(executing_command.substring(begin, end).c_str(),
            "<y:%d,p:%d,r:%d>", &target_yaw_pitch_roll[0],
            &target_yaw_pitch_roll[1], &target_yaw_pitch_roll[2]);
        */

        // assign value to speed array
        sscanf(executing_command.substring(begin, end).c_str(), "<%d,%d,%d,%d>",
               &speed[0], &speed[1], &speed[2], &speed[3]);
      }
    }

    motor_set_speed();
    break;

  default:
    break;
  }

  delay(20);
}