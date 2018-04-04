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

ServoControl::ServoControl(uint8_t pin1, uint16_t minMicros1, uint16_t maxMicros1, uint16_t position1) {
  Servo servo1;
  servo = servo1;
  pin = pin1;
  minMicros = minMicros1;
  maxMicros = maxMicros1;
  micros = SERVO_DEFAULT_MICROS;
  position = position1;
  target = position1;
  speed = 0;
  moveMask = 0;
  isDone = true;
  init();
}

ServoControl::ServoControl(uint8_t pin1, uint16_t minMicros1, uint16_t maxMicros1) {
  Servo servo1;
  servo = servo1;
  pin = pin1;
  minMicros = minMicros1;
  maxMicros = maxMicros1;
  position = SERVO_DEFAULT_POSITION;
  target = SERVO_DEFAULT_POSITION;
  speed = 0;
  moveMask = 0;
  isDone = true;
  init();
}

ServoControl::ServoControl(uint8_t pin1, uint16_t position1) {
  Servo servo1;
  servo = servo1;
  pin = pin1;
  minMicros = SERVO_DEFAULT_MIN_MICROS;
  maxMicros = SERVO_DEFAULT_MAX_MICROS;
  position = position1;
  target = SERVO_DEFAULT_POSITION;
  speed = 0;
  moveMask = 0;
  isDone = true;
  init();
}

ServoControl::ServoControl(uint8_t pin1) {
  Servo servo1;
  servo = servo1;
  pin = pin1;
  minMicros = SERVO_DEFAULT_MIN_MICROS;
  maxMicros = SERVO_DEFAULT_MAX_MICROS;
  position = SERVO_DEFAULT_POSITION;
  target = SERVO_DEFAULT_POSITION;
  speed = 0;
  moveMask = 0;
  isDone = true;
  init();
}

ServoControl::ServoControl() {
  Servo servo1;
  servo = servo1;
  pin = NONE;
  minMicros = SERVO_DEFAULT_MIN_MICROS;
  maxMicros = SERVO_DEFAULT_MAX_MICROS;
  position = SERVO_DEFAULT_POSITION;
  target = SERVO_DEFAULT_POSITION;
  speed = 0;
  moveMask = 0;
  isDone = true;
  init();
}

void ServoControl::init() {
  if (pin != NONE) {
    servo.attach(pin);
    servo.writeMicroseconds(micros);
  }
}

/*
   address   0-7     which servo
   position  0-1440  1/8 of a degree
*/
boolean ServoControl::set(uint16_t position1) {
  //  if (address >= MAX_NUM_MOTORS) return false;
  //the servo must be attached
  if (pin == NONE) return false;

  if (position1 > SERVO_TICKS_PER_RANGE) position1 = SERVO_TICKS_PER_RANGE;

  //set the position and target position to the same thing
  target = position1;
  position = position1;

  //the mask will not be checked anyway
  moveMask = 0;

  //the servo is at its target
  isDone = true;

  return true;
}

/*
   address   0-7     which servo
   position  0-1440  1/8 of a degree
   speed     1-64    ticks per 128 milliseconds

   a value of 0 for the speed will result in no command sent
   a value greater than 64 will call the other setServo function (max speed)
*/
boolean ServoControl::set(uint16_t position1, uint8_t speed1) {
  //speed cannot be 0
  if (speed1 == 0) return false;
  //  if (speed == 0 || address >= MAX_NUM_MOTORS) return false;

  //anything higher than 64 is faster than the servo can go
  if (speed1 > 64) speed1 = 64;

  //the servo must be attached
  if (pin == NONE) return false;

  if (position1 > SERVO_TICKS_PER_RANGE) position1 = SERVO_TICKS_PER_RANGE;

  //set the target
  target = position1;
  isDone = (position == target);

  if (speed != speed1) {
    speed = speed1;

    //set moveMask to have evenly spaced bits set
    //the total number of bits set is equal to the speed
    uint64_t servoMoveMask = 0;
    uint64_t mask = 1;
    uint8_t j = 0, oldJ = 255;
    for (uint8_t i = 0; i < 64; i++) {
      j = i * speed1 / 64;
      if (j != oldJ ) {
        servoMoveMask |= mask;
        oldJ = j;
      }
      mask = mask << 1;
    }
    moveMask = servoMoveMask;
  }
  return true;
}


uint16_t ServoControl::calculateMicros() {
  return constrain(map(position, 0, SERVO_TICKS_PER_RANGE, minMicros, maxMicros), minMicros, maxMicros);
}

void ServoControl::update(uint64_t servoMask) {
  //12 us
  //update servo
  //note: delta time is always 1 millisecond
  //if it is time to update the servos (see above) and the servo position is not at the target
  //and the servo is set to move on this tick based on the moveMask
  if ((position != target) && (moveMask & servoMask)) {
    //move the servo 1 step toward the target
    if (position > target) {
      position -= min(SERVO_INC_PER_MILLI, position - target);
    } else {
      position += min(SERVO_INC_PER_MILLI, target - position);
    }
  }
  //send the command to the Servo library
  uint16_t micros1 = calculateMicros();
  if (micros1 != micros) {
    micros = micros1;
    servo.writeMicroseconds(micros);
  }
  //record whether or not the servo is done for external purposes
  isDone = (position == target);
}

