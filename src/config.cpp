#include "config.h"
#include "port.h"
#include <Arduino.h>
#include <SoftwareSerial.h>

// Hardware Serial Port
// Micro USB
HardwareSerial Serial1(RX1, TX1);

// Software Serial Port
// HC14
SoftwareSerial hc_14(RX2, TX2);

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

  // initialize hc_14
  // hc_14 is for LoRa communication
  hc_14.begin(115200);
}