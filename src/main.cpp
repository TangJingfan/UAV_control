#include "config.h"
#include "imu.h"
#include "pid.h"
#include "port.h"
#include "uav_motor.h"
#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>

pid_controller pid_yaw(0.8, 0.1, 0.05);
pid_controller pid_pitch(0.8, 0.1, 0.05);
pid_controller pid_roll(0.8, 0.1, 0.05);

enum state { waiting, executing, stop };

state current_state = waiting;
String received_command = "";
String executing_command = "";

float dt_pid;
float roll_output;
float pitch_output;
float yaw_output;

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

  if (received_command == "<STOP>") {
    current_state = stop;
  }

  /*
    use Euler angle to represent UAV's attitude
    basic format: <y:0,p:0,r:0>
    use speed to control uav
    basic format: <m1,m2,m3,m4>
  */

  // calculate Euler angle by MPU6500
  calculate_euler_angle();

  // for (int k = 0; k < 3; k++) {
  //   Serial2.println(current_yaw_pitch_roll[k]);
  // }

  // for (int l = 0; l < motor_num; l++) {
  //   Serial2.println(speed[l]);
  // }

  switch (current_state) {
  case waiting:

    // dt_pid = 0.01;
    // roll_output = pid_roll.compute(target_yaw_pitch_roll[2],
    //                                current_yaw_pitch_roll[2], dt_pid);
    // pitch_output = pid_pitch.compute(target_yaw_pitch_roll[1],
    //                                  current_yaw_pitch_roll[1], dt_pid);
    // yaw_output = pid_yaw.compute(target_yaw_pitch_roll[0],
    //                              current_yaw_pitch_roll[0], dt_pid);

    // update_motor_speeds(roll_output, pitch_output, yaw_output);
    motor_set_speed();
    break;

  case executing:
    // update executing command
    executing_command = received_command;
    received_command = "";
    current_state = waiting;
    // assign speed to speed array
    if (executing_command.length() > 0) {
      int begin = executing_command.indexOf('<');
      int end = executing_command.indexOf('>') + 1;
      if (begin != -1 && end != -1 && begin < end) {
        // assign value to attitude array
        // sscanf(executing_command.substring(begin, end).c_str(),
        //        "<y:%d,p:%d,r:%d>", &target_yaw_pitch_roll[0],
        //        &target_yaw_pitch_roll[1], &target_yaw_pitch_roll[2]);

        // assign value to speed array

        sscanf(executing_command.substring(begin, end).c_str(), "<%d,%d,%d,%d>",
               &speed[0], &speed[1], &speed[2], &speed[3]);
      }
    }

    // dt_pid = 0.01;
    // roll_output = pid_roll.compute(target_yaw_pitch_roll[2],
    //                                current_yaw_pitch_roll[2], dt_pid);
    // pitch_output = pid_pitch.compute(target_yaw_pitch_roll[1],
    //                                  current_yaw_pitch_roll[1], dt_pid);
    // yaw_output = pid_yaw.compute(target_yaw_pitch_roll[0],
    //                              current_yaw_pitch_roll[0], dt_pid);

    // update_motor_speeds(roll_output, pitch_output, yaw_output);
    motor_set_speed();
    break;

  case stop:

    for (int j = 0; j < motor_num; j++) {
      speed[j] = 0;
    }
    motor_set_speed();

  default:
    break;
  }

  delay(20);
}