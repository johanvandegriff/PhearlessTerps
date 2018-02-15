#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//Faster digital IO:
//https://github.com/mmarchetti/DirectIO
#include <DirectIO.h>
//#include <FastAnalogRead.h>
#include <Servo.h>
#include <Encoder.h>

#define NONE -1

#define MAX_NUM_MOTORS 8

#define SERVO_DEFAULT_MIN_MICROS 1000
#define SERVO_DEFAULT_MAX_MICROS 2000
#define SERVO_DEFAULT_POSITION 720
#define SERVO_DEFAULT_MICROS 1500
#define SERVO_INC_PER_MILLI 4
#define SERVO_TICKS_PER_RANGE 1440


//shift register pins
Output<49> shiftClock(LOW);
Output<50> shiftLatch(LOW);
Input<51> shiftDataIn(true);
Output<52> shiftDataOut1(LOW);
Output<53> shiftDataOut2(LOW);

Output<39> togglePin(LOW);

/*
 * Motor
 * contains the motor's pins, state, and feedback
 */
typedef struct {
  /*
   * pin connected to IN2 on the motor controller
   * controls the power sent to the motor
   */
  signed pwmPin : 8;
  /*
   * pin connected to FB on the motor controller
   * reads the current the motor is drawing
   */
  signed analogPin : 8;
  /*
   * connected to the encoder pins directly on the motor
   */
  unsigned hasEncoder : 1;
  Encoder encoder;
//  InputPin encoderPin1;
//  InputPin encoderPin2;

  /*
   * the motor's power, including direction
   * ranges from -255 to 255 (-256 is illegal)
   */
  signed power : 9;
  /*
   * true to brake when stopping
   * false to coast when stopping
   */
  unsigned brake : 1;

  /*
   * the value from the encoder
   */
  int32_t encoderPos : 32;
  /*
   * the current being drawn by the motor
   * a negative value means there is no sensor or it has not been read yet
   */
  signed currentDraw : 11;
  /*
   * the status flag of the motor controller
   */
  unsigned isOK : 1;
  
} Motor;

/*
 * ServoControl
 * controls the speed of a servo
 */
typedef struct {
  Servo servo;
  /*
   * Which pin the servo is connected to
   */
  signed pin : 8;
  /*
   * The minimum microseconds to send the servo to set it to its minimum position
   * 0-2048, default = 1000
   */
  unsigned minMicros : 11;
  /*
   * The maximum microseconds to send the servo to set it to its maximum position
   * 0-4096, default = 2000
   */
  unsigned maxMicros : 12;
  /*
   * The last command sent to the servo
   */
  unsigned micros : 12;
  /*
   * The current position of the servo in 1/8 degree ticks.
   * 0-1440, default = 360 (180 degrees with 1/8 degree precision)
   */
  unsigned position : 11;
  /*
   * The target position where the servo is headed when using the speed control
   * 0-1440 (180 degrees with 1/8 degree precision)
   */
  unsigned target : 11;
  /*
   * A bitmask where each bit tells whether or not to increment the servo on that millisecond
   * Every millisecond, the mask is checked, and if it is a 1 at that position, the servo increments 4 ticks (0.5 degrees)
   */
  uint64_t moveMask : 64;
  /*
   * For speed control, tells whether the servo's position has reached its target
   * 0 = not done, 1 = done
   */
  unsigned isDone : 1;
} ServoControl;

//with encoders and analog
Motor newMotor(int8_t pwmPin, int8_t analogPin, int8_t encoderPin1, int8_t encoderPin2, boolean brake) {
  return {pwmPin, analogPin, true, Encoder(encoderPin1, encoderPin2), 0, brake, 0, NONE, true};
}

//with encoders only
Motor newMotor(int8_t pwmPin, int8_t encoderPin1, int8_t encoderPin2, boolean brake) {
  return {pwmPin, NONE, true, Encoder(encoderPin1, encoderPin2), 0, brake, 0, NONE, true};
}

//with analog only
Motor newMotor(int8_t pwmPin, int8_t analogPin, boolean brake) {
  return {pwmPin, analogPin, false, Encoder(0,0), 0, brake, 0, NONE, true};
}

//with neither encoders or analog
Motor newMotor(int8_t pwmPin, boolean brake) {
  return {pwmPin, NONE, false, Encoder(0,0), 0, brake, 0, NONE, true};
}

//no motor
Motor newMotor() {
  return {NONE, NONE, false, Encoder(0,0), 0, true, 0, NONE, true};
}

Motor motors[MAX_NUM_MOTORS] = {
// PORT          PWM|ANALOG|ENCODER1|ENCODER2|BRAKE
  /*0*/ newMotor( 2,  A8,                     true ),
  /*1*/ newMotor( 3,  A9,                     true ),
  /*2*/ newMotor( 4, A10,                     true ),
  /*3*/ newMotor( 5, A11,                     true ),
  /*4*/ newMotor( 6, A12,                     true ),
  /*5*/ newMotor( 7, A13,                     true ),
  /*6*/ newMotor( 8, A14,                     true ),
  /*7*/ newMotor( 9, A15,                     true )
};

uint16_t calculateServoMicros(ServoControl servo) {
  return constrain(map(servo.position, 0, SERVO_TICKS_PER_RANGE, servo.minMicros, servo.maxMicros), servo.minMicros, servo.maxMicros);
}

ServoControl newServo(uint8_t pin, uint16_t minMicros, uint16_t maxMicros, uint16_t position) {
  Servo servo;
  return {servo, pin, minMicros, maxMicros, position, position, 0, true};
}

ServoControl newServo(uint8_t pin, uint16_t minMicros, uint16_t maxMicros) {
  Servo servo;
  return {servo, pin, minMicros, maxMicros, SERVO_DEFAULT_MICROS, SERVO_DEFAULT_POSITION, SERVO_DEFAULT_POSITION, 0, true};
}

ServoControl newServo(uint8_t pin, uint16_t position) {
  Servo servo;
  return {servo, pin, SERVO_DEFAULT_MIN_MICROS, SERVO_DEFAULT_MAX_MICROS, SERVO_DEFAULT_MICROS, position, position, 0, true};
}

ServoControl newServo(uint8_t pin) {
  Servo servo;
  return {servo, pin, SERVO_DEFAULT_MIN_MICROS, SERVO_DEFAULT_MAX_MICROS, SERVO_DEFAULT_MICROS, SERVO_DEFAULT_POSITION, SERVO_DEFAULT_POSITION, 0, true};
}

ServoControl newServo() {
  Servo servo;
  return {servo, NONE, SERVO_DEFAULT_MIN_MICROS, SERVO_DEFAULT_MAX_MICROS, SERVO_DEFAULT_MICROS, SERVO_DEFAULT_POSITION, SERVO_DEFAULT_POSITION, 0, true};
}


/*
 * holds all the servos
 * if a servo is unused, set the pin to NONE
 * the serco library only allows MIN as low as 544 and MAX as high as 2400
 */
ServoControl servos[MAX_NUM_MOTORS] = {
// PORT          PIN|MIN|MAX|POS (0-1440)
  /*0*/ newServo(22, 544,2400         ),
  /*1*/ newServo(23                   ),
  /*2*/ newServo(24                   ),
  /*3*/ newServo(25                   ),
  /*4*/ newServo(26                   ),
  /*5*/ newServo(                     ),
  /*6*/ newServo(                     ),
  /*7*/ newServo(                     )
};


//call this from the setup() function to speed up analogRead
void EnableFastAnalogRead() {
  // set prescale to 16
  sbi(ADCSRA,ADPS2);
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
}


String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() {
  EnableFastAnalogRead(); //enable fast analog reading

  Serial.begin(9600); //TODO increase baud rate if possible

  //initialize shift registers
  for (uint8_t i=0; i<8; i++) {
    shiftClock.pulse(HIGH);
  }
  shiftLatch = HIGH;
  
  for(uint8_t i=0; i<MAX_NUM_MOTORS; i++) {
    //initialize the motor pins
    if(motors[i].pwmPin != NONE) {
      pinMode(motors[i].pwmPin, OUTPUT);
      if(motors[i].analogPin != NONE) {
        pinMode(motors[i].analogPin, INPUT);
      }
      //the status flag and the 2 digital outputs are on the shift register
    }
    if (servos[i].pin != NONE) {
      servos[i].servo.attach(servos[i].pin);
      servos[i].servo.writeMicroseconds(servos[i].micros);
    }
  }

  inputString.reserve(200);

  //set up a timer interrupt every millisecond
//  OCR0A = 0x01; //millis() counter uses 0
//  TIMSK0 |= _BV(OCIE0A);
}

//uint32_t timer;
uint64_t servoMask = 0;

void updateDevices() {
//timer interrupt function (every millisecond)
//SIGNAL(TIMER0_COMPA_vect) {
//  togglePin.toggle();
//  return;
//  uint32_t start = micros();
  //shift the mask to the left by 1, wrapping around when it reaches the end
  //4 us
  servoMask = servoMask << 1;
  if (servoMask == 0) servoMask = 1;
  
  //update motors (including shift registers)
  shiftLatch = LOW;
  for(uint8_t i=0; i<MAX_NUM_MOTORS; i++) {
    if(motors[i].pwmPin != NONE) { //if it is attached
      //16 us
      //read the encoders if it has them
      if(motors[i].hasEncoder) {
        //TODO use the encoder readings
        motors[i].encoderPos = motors[i].encoder.read();
      }
      //read the current draw if it has an analog pin
      if(motors[i].analogPin != NONE) {
        motors[i].currentDraw = analogRead(motors[i].analogPin);
      }

      //0 us??
      //update pin outputs based on motor power
      uint8_t pwmOutput;
      if(motors[i].power > 0) { //FORWARD
        //write to the shift register pins
        shiftDataOut1 = 1; //EN on the motor controller
        shiftDataOut2 = 0; //IN1 on the motor controller
        //write to the pwm pin
        pwmOutput = motors[i].power;
      } else if(motors[i].power < 0) { //REVERSED
        //write to the shift register pins
        shiftDataOut1 = 1; //EN on the motor controller
        shiftDataOut2 = 1; //IN1 on the motor controller
        pwmOutput = 255 + motors[i].power;
      } else { //STOPPED
        //write to the shift register pins
        shiftDataOut1 = 1; //EN on the motor controller
        shiftDataOut2 = 1; //IN1 on the motor controller
        if (motors[i].brake) {
          pwmOutput = 255;
        } else {
          pwmOutput = 0;
        }
      }
      //write to the pwm pin
      //12 us
      analogWrite(motors[i].pwmPin, pwmOutput); //IN2 on the motor controller

      //shift register clock
      //4 us
      shiftClock = HIGH;
      motors[i].isOK = shiftDataIn;
    } else {
      //still pulse the shift register clock if there was no motor
      shiftDataOut1 = LOW;
      shiftDataOut2 = LOW;
      shiftClock = HIGH;
    }
    shiftClock = LOW;

    //12 us
    //update servo
    //note: delta time is always 1 millisecond
    //if it is time to update the servos (see above) and the servo position is not at the target
    //and the servo is set to move on this tick based on the moveMask
    if ((servos[i].position != servos[i].target) && (servos[i].moveMask & servoMask)) {
      //move the servo 1 step toward the target
      if (servos[i].position > servos[i].target) {
          servos[i].position -= min(SERVO_INC_PER_MILLI, servos[i].position - servos[i].target);
      } else {
          servos[i].position += min(SERVO_INC_PER_MILLI, servos[i].target - servos[i].position);
      }
    }
    //send the command to the Servo library
    uint16_t micros = calculateServoMicros(servos[i]);
    if (micros != servos[i].micros) {
      servos[i].micros = micros;
      servos[i].servo.writeMicroseconds(micros);
    }
    //record whether or not the servo is done for external purposes
    servos[i].isDone = (servos[i].position == servos[i].target);
  }
  shiftLatch = HIGH;
  //timer = micros() - start;
}

/*
 * address   0-7     which servo
 * position  0-1440  1/8 of a degree
 */
boolean setServo(uint8_t address, uint16_t position) {
  if (address >= MAX_NUM_MOTORS) return false;
  //the servo must be attached
  if (servos[address].pin == NONE) return false;

  if (position > SERVO_TICKS_PER_RANGE) position = SERVO_TICKS_PER_RANGE;

  //set the position and target position to the same thing
  servos[address].target = position;
  servos[address].position = position;

  //the mask will not be checked anyway
  servos[address].moveMask = 0;

  //the servo is at its target
  servos[address].isDone = true;

  return true;
}

/*
 * address   0-7     which servo
 * position  0-1440  1/8 of a degree
 * speed     1-64    ticks per 128 milliseconds
 * 
 * a value of 0 for the speed will result in no command sent
 * a value greater than 64 will call the other setServo function (max speed)
 */
boolean setServo(uint8_t address, uint16_t position, uint8_t speed) {
  //speed cannot be 0
  if (speed == 0 || address >= MAX_NUM_MOTORS) return false;

  //anything higher than 64 is faster than the servo can go
  if (speed > 64) return setServo(address, position);
  
  //the servo must be attached
  if (servos[address].pin == NONE) return false;

  if (position > SERVO_TICKS_PER_RANGE) position = SERVO_TICKS_PER_RANGE;
  
  //set the target
  servos[address].target = position;
  
  servos[address].isDone = (servos[address].position == servos[address].target);

  //set moveMask to have evenly spaced bits set
  //the total number of bits set is equal to the speed
  uint64_t servoMoveMask = 0;
  uint64_t mask = 1;
  uint8_t j = 0, oldJ = 255;
  for (uint8_t i=0; i<64; i++) {
    j = i * speed / 64;
    if (j != oldJ ) {
      servoMoveMask |= mask;
      oldJ = j;
    }
    mask = mask << 1;
  }
  
  servos[address].moveMask = servoMoveMask;
  
  return true;
}

boolean setMotorPower(uint8_t address, int16_t power) {
  if (address >= MAX_NUM_MOTORS || power < -255 || power > 255) return false;
  if (motors[address].pwmPin == NONE) return false;

  motors[address].power = constrain(power, -255, 255);
  
  return true;
}

boolean setMotorBrake(uint8_t address, boolean brake) {
  if (address >= MAX_NUM_MOTORS) return false;
  if (motors[address].pwmPin == NONE) return false;

  motors[address].brake = brake;
  
  return true;
}

//int power = 0;

/*commands:
M - run 'M'otor
  char | meaning   | limits
     0 | M
     1 | address     0-8
     2 | direction   0-1
     3 | power       0-255
     4 | power
  M31FF - motor 3 forward 255
  M40FF - motor 4 backward 255
  M5180 - motor 5 forward 128
  M6005 - motor 6 backward 5
B - set motor 'B'rake
  char | meaning   | limits
     0 | B
     1 | address     0-8
     2 | brake?      0-1
  B20 - motor 2 brake off (coast)
  B31 - motor 3 brake on (float)

P - set servo 'P'osition
  char | meaning   | limits
     0 | P
     1 | address     0-8
     2 | position    0-180
     3 | position
   P3B4 - servo 3 to 180 (B4)
   P400 - servo 4 to 0

S - set servo position and 'S'peed
  char | meaning   | limits
     0 | P
     1 | address     0-8
     2 | position    0-180
     3 | position
     4 | speed       1-64
     5 | speed
*/
uint32_t loopTimer = 0;

//boolean isSpeedIncreasing = true;
//boolean isPosIncreasing = false;
//boolean speedWrapAround = true;
//
//int speed = 1 + 63 * (!isSpeedIncreasing);
//int pos = 1440 * (!isPosIncreasing);
int pos = 0;

//int pos = 0;    // variable to store the servo position
//// the loop function runs over and over again forever
//uint32_t timer = 0;
//boolean isIncreasing;
void loop() {
  if(servos[0].isDone) {
    pos = 1440 - pos;
    setServo(0, pos, 5);
  }
//  if(loopTimer % 50 == 0) {
//    Serial.println(servos[0].micros);
//  }
/*  if(servos[0].isDone) {
    setServo(0, pos, speed);
    //Serial.println(speed);
    if (isPosIncreasing) {
      pos += 22;
    } else {
      pos -= 22;
    }
    if (isSpeedIncreasing) {
      speed += 1;
      if (speed > 64) {
        isPosIncreasing = !isPosIncreasing;
        if(speedWrapAround) {
          speed = 1;
        } else {
          speed = 64;
          isSpeedIncreasing = false;
        }
      }
    } else {
      speed -= 1;
      if (speed <= 0) {
        isPosIncreasing = !isPosIncreasing;
        if(speedWrapAround) {
          speed = 64;
        } else {
          speed = 1;
          isSpeedIncreasing = true;
        }
      }
    }
  }

  if(loopTimer % 100 == 0) {
    Serial.print("s");
    Serial.print(speed);
    Serial.print(" p");
    Serial.print(pos);
    Serial.print(" si");
    Serial.print(isSpeedIncreasing);
    Serial.print(" pi");
    Serial.print(isPosIncreasing);
    Serial.println();
  }*/
  
/*  if (millis() > timer) {
    timer = millis() + 15;
    if(isIncreasing) {
      pos += 8;
      if (pos >= 1440) {
        isIncreasing = false;
        for(int i=0; i<8; i++) {
          setMotorPower(i, 30*(i+1));
        }
      }
    } else {
      pos -= 8;
      if (pos <= 0) {
        isIncreasing = true;
        for(int i=0; i<8; i++) {
          setMotorPower(i, 0);
        }
      }
    }
    setServo(0, pos);
//    servos[0].position = pos;
//    tmp();
  }*/
    // print the string when a newline arrives:
  if (stringComplete) {
//    Serial.println(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }

  updateDevices();

//  delayMicroseconds(200);
//  delayMicroseconds(150);

  //make sure the loop runs no faster than once every 1 millisecond
  int16_t difference = millis() - loopTimer;
  if (difference >= 100) {
    Serial.print(loopTimer);
    Serial.print(" skip ");
    Serial.println(difference);
    loopTimer += difference;
  } else {
    loopTimer += 1;
    while (millis() < loopTimer);
  }
  
  if(loopTimer % 1000 == 0) {
    Serial.print(loopTimer);
    Serial.print(" ");
    Serial.println(micros());
  }
}

/*
  https://www.arduino.cc/en/Tutorial/SerialEvent
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    //get the new byte:
    char inChar = Serial.read();
    //if the incoming character is a newline
    if (inChar == '\n') {
      //set a flag so the main loop can do something about it:
      stringComplete = true;
    } else {
      //otherwise, add it to the inputString:
      inputString += inChar;
    }
  }
}



  /*
  for (pos = 0; pos <= 1440; pos += 8) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servos[0].position = pos;              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  //Serial.println(timer-4);
  tmp();
  }
//    for(int i=0; i<8; i++) {
//      setMotorPower(i, 0);
//    }
  for (pos = 1440; pos >= 0; pos -= 8) { // goes from 180 degrees to 0 degrees
    servos[0].position = pos;              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  //Serial.println(timer-4);
  tmp();
  }*/

//  return;
/*  if(Serial.available()) {
    char c = Serial.read();
    Serial.print("recieved: ");
    Serial.print(c);
    Serial.print(" ");
    Serial.println(c, DEC);
    if (c == 'M') {
      uint8_t address = Serial.read() - 48;
      if (address < MAX_NUM_MOTORS) {
        uint8_t direction = Serial.read() - 48;
        if (direction <= 1) {
          uint8_t power1 = Serial.read() - 48;
          if (power1 >= 17) power1 -= 17 - 10;
          if (power1 >= 42) power1 -= 42;
          if (power1 <= 16) {
            uint8_t power2 = Serial.read() - 48;
            if (power2 >= 17) power2 -= 17 - 10;
            if (power2 >= 42) power2 -= 42;
            if (power2 <= 16) {
              int16_t power = power1 * 16 + power2;
              if (direction == 0) power *= -1;
              Serial.print("SetPower - ");
              Serial.print(address);
              Serial.print(" ");
              Serial.print(direction);
              Serial.print(" ");
              Serial.println(power);
              setMotorPower(address, power);
            }
          }
        }
      }
    } else
    if (c == 'B') {
      uint8_t address = Serial.read() - 48;
      if (address < MAX_NUM_MOTORS) {
        uint8_t brake = Serial.read() - 48;
        if (brake <= 1) {
          setMotorBrake(address, brake);
        }
      }
      
    } else
    if (c == 'P') {
      
    } else
    if (c == 'S') {
      
    }
  }
  
  if (millis() < 150) {
    for(int i=0; i<8; i++) {
      setMotorPower(i, 30*(i+1));
    }
//    Serial.println(servos[0].micros);
    servos[0].servo.writeMicroseconds(1100);
//    setServo(0, 100);
  }
  if (millis() > 350 && millis() < 400) {
    for(int i=0; i<8; i++) {
      setMotorPower(i, 0);
    }
  }
  
  if (millis() > 1350 && millis() < 1450) {
    servos[0].servo.writeMicroseconds(1900);
//    Serial.println(setServo(0, 600));
  }

//  if (servos[0].micros < 1400) servos[0].micros+=20;

//  Serial.print(motors[0].power);
//  Serial.print(" ");
//  Serial.print(motors[0].isOK);
//  Serial.print(" ");
//  Serial.print(analogRead(A8));
//  Serial.print(" ");
//  Serial.println(motors[0].currentDraw);
  delay(10);*/

