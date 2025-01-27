#include "config.h"
#include "port.h"
#include <Arduino.h>

HardwareSerial Serial1(RX1, TX1);

int motor_num = 4;

void config_setup() {
  // initialize PWM pins
  // PWM pins are for motor
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM4, OUTPUT);

  // initialize battery pin
  pinMode(ADC_BATTERY, INPUT_ANALOG);

  // initialize LED lights
  // lights can be used to debug
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  // initialize Serial1
  // Serial1 is for Micro USB wired connection
  Serial1.begin(115200);

  // initialize Serial2
  // Serial2 is for LoRa communication
  Serial2.setRx(RX2);
  Serial2.setTx(TX2);
  Serial2.begin(115200);
}