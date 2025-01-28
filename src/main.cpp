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

  switch (current_state) {
  case waiting:
    motor_set_speed();
    break;
  case executing:
    // print executing command
    Serial2.println("Executing Message:");
    Serial2.println(executing_command);
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
        //        "<y:%d,r:%d,p:%d>", &attitude[0], &attitude[1], &attitude[2]);

        // assign value to speed array
        sscanf(executing_command.substring(begin, end).c_str(), "<%d,%d,%d,%d>",
               &speed[0], &speed[1], &speed[2], &speed[3]);
      }
    }

    // print attitude value
    Serial2.printf("yaw:%d\n", yaw_pitch_roll[0]);
    Serial2.printf("roll:%d\n", yaw_pitch_roll[1]);
    Serial2.printf("pitch:%d\n", yaw_pitch_roll[2]);

    motor_set_speed();
    break;

  default:
    break;
  }

  // digitalWrite(LED1, HIGH);
  // delay(1000);
  // digitalWrite(LED1, LOW);
  // delay(1000);

  delay(20);
}