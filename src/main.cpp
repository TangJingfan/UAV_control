#include "config.h"
#include "port.h"
#include "uav_motor.h"
#include <Arduino.h>

enum state { waiting, executing };

state current_state = waiting;
String received_command = "";
String executing_command = "";

void setup() {
  config_setup();
  digitalWrite(LED1, HIGH);
  delay(5000);
}

void loop() {
  while (Serial1.available() > 0) {
    char temp_received = char(Serial1.read());
    received_command += temp_received;
    current_state = executing;
  }

  // use Euler angle to represent UAV's attitude
  // basic format: <y:0,r:0,p:0>
  Serial1.println("Received Message:");
  Serial1.println(received_command);
}