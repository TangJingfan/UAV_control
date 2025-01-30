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

  // use Euler angle to represent UAV's attitude
  // basic format: <y:0,p:0,r:0>
  // use speed to control uav
  // basic format: <m1,m2,m3,m4>

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
        // sscanf(executing_command.substring(begin, end).c_str(),
        //        "<y:%d,p:%d,r:%d>", &attitude[0], &attitude[1], &attitude[2]);

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