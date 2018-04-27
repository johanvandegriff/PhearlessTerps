#include "Arduino.h"
#include "Hardware.h"

Motor::Motor(int8_t directionPin1, int8_t pwmPin1, int8_t enablePin1, int8_t analogPin1, int8_t statusPin1, boolean brake1) {
  directionPin = directionPin1;
  pwmPin = pwmPin1;
  enablePin = enablePin1;
  analogPin = analogPin1;
  statusPin = statusPin1;
  power = 0;
  brake = brake1;
  currentDraw = NONE;
  isOK = true;
  init();
}

Motor::Motor() {
  directionPin = NONE;
  pwmPin = NONE;
  enablePin = NONE;
  analogPin = NONE;
  statusPin = NONE;
  power = 0;
  brake = true;
  currentDraw = NONE;
  isOK = true;
  init();
}

void Motor::init() {
  //initialize the motor pins
  if (directionPin != NONE) {
    pinMode(directionPin, OUTPUT);
  }
  if (pwmPin != NONE) {
    pinMode(pwmPin, OUTPUT);
  }
  if (enablePin != NONE) {
    pinMode(enablePin, OUTPUT);
  }
  if (analogPin != NONE) {
    pinMode(analogPin, INPUT);
  }
  if (statusPin != NONE) {
    pinMode(statusPin, INPUT);
  }
}

boolean Motor::setPower(int16_t power1) {
  if (power1 < -255 || power1 > 255) return false;
  if (directionPin == NONE || pwmPin == NONE || enablePin == NONE) return false;

  power = power1; //constrain(power1, -255, 255);

  return true;
}

void Motor::update() {
  if (pwmPin == NONE) return; //if it is not attached

  //update pin outputs based on motor power
  uint8_t directionOut, pwmOut, enableOut;
  if (power > 0) { //FORWARD
    enableOut = 1; //EN on the motor controller
    directionOut = 0; //IN1 on the motor controller
    pwmOut = power;
  } else if (power < 0) { //REVERSED
    enableOut = 1; //EN on the motor controller
    directionOut = 1; //IN1 on the motor controller
    pwmOut = 255 + power;
  } else { //STOPPED
    enableOut = brake; //EN on the motor controller
    directionOut = 1; //IN1 on the motor controller
    pwmOut = 255;
  }
  //write to the pwm pin
  //12 us
  analogWrite(pwmPin, pwmOut); //IN2 on the motor controller

  //write to the digital pins
  digitalWrite(directionPin, directionOut);
  digitalWrite(enablePin, enableOut);

  //read the status if the pin os connected
  if (statusPin != NONE) {
    isOK = digitalRead(statusPin);
  }

  //read the current draw if it has an analog pin
  if (analogPin != NONE) {
    currentDraw = analogRead(analogPin);
  }
}

