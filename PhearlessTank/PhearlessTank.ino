#include <Enes100.h>

#include <dfr_tank.h>

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

/* Create a new Enes100 object
 * Parameters:
 *  string teamName
 *  int teamType
 *  int markerId
 *  int rxPin
 *  int txPin
 */
Enes100 enes("pHearless Terps", CHEMICAL, 3, 8, 9);

DFRTank tank;

#define NONE -1

#define MAX_NUM_MOTORS 8
#define MAX_NUM_SERVOS 8

#define SERVO_DEFAULT_MIN_MICROS 1000
#define SERVO_DEFAULT_MAX_MICROS 2000
#define SERVO_DEFAULT_POSITION 720
#define SERVO_DEFAULT_MICROS 1500
#define SERVO_INC_PER_MILLI 4
#define SERVO_TICKS_PER_RANGE 1440

/*
 * Motor
 * contains the motor's pins, state, and feedback
 */
class Motor {
  public:
    /*
     * the motor's power, including direction
     * ranges from -255 to 255 (-256 is illegal)
     */
    signed power : 16;
    unsigned isLeft : 8;
    unsigned isConnected : 8;
  
    Motor(boolean isLeft1) {
      isLeft = isLeft1;
      isConnected = true;
    }
  
    Motor() {
      isLeft = true;
      isConnected = false;
    }
  
    boolean setPower(int16_t power1) {
      if (power1 < -255 || power1 > 255) return false;
      if (!isConnected) return false;
    
      power = power1; //constrain(power1, -255, 255);
      
      return true;
    }
  
    void update() {
      if (!isConnected) return; //if it is not attached
      if (isLeft) {
        tank.setLeftMotorPWM(power);
      } else {
        tank.setRightMotorPWM(power);
      }
    }
};

/*
 * ServoControl
 * controls the speed of a servo
 */
class ServoControl {
  public:
    Servo servo;
    /*
     * Which pin the servo is connected to
     */
    signed pin : 8;
    /*
     * The minimum microseconds to send the servo to set it to its minimum position
     * 0-2048, default = 1000
     */
    unsigned minMicros : 16;
    /*
     * The maximum microseconds to send the servo to set it to its maximum position
     * 0-4096, default = 2000
     */
    unsigned maxMicros : 16;
    /*
     * The last command sent to the servo
     */
    unsigned micros : 16;
    /*
     * The current position of the servo in 1/8 degree ticks.
     * 0-1440, default = 360 (180 degrees with 1/8 degree precision)
     */
    unsigned position : 16;
    /*
     * The target position where the servo is headed when using the speed control
     * 0-1440 (180 degrees with 1/8 degree precision)
     */
    unsigned target : 16;
  
    /*
     * stores the last speed set for this servo
     * prevents the moveMask from being recreated every time
     */
    unsigned speed : 8;
    /*
     * A bitmask where each bit tells whether or not to increment the servo on that millisecond
     * Every millisecond, the mask is checked, and if it is a 1 at that position, the servo increments 4 ticks (0.5 degrees)
     */
    uint64_t moveMask : 64;
    /*
     * For speed control, tells whether the servo's position has reached its target
     * 0 = not done, 1 = done
     */
    unsigned isDone : 8;
  
    ServoControl(uint8_t pin1, uint16_t minMicros1, uint16_t maxMicros1, uint16_t position1) {
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
    }
    
    ServoControl(uint8_t pin1, uint16_t minMicros1, uint16_t maxMicros1) {
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
    }
    
    ServoControl(uint8_t pin1, uint16_t position1) {
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
    }
    
    ServoControl(uint8_t pin1) {
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
    }
    
    ServoControl() {
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
    }
  
    /*
     * address   0-7     which servo
     * position  0-1440  1/8 of a degree
     */
    boolean set(uint16_t position1) {
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
     * address   0-7     which servo
     * position  0-1440  1/8 of a degree
     * speed     1-64    ticks per 128 milliseconds
     * 
     * a value of 0 for the speed will result in no command sent
     * a value greater than 64 will call the other setServo function (max speed)
     */
    boolean set(uint16_t position1, uint8_t speed1) {
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
        for (uint8_t i=0; i<64; i++) {
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
    
  
    uint16_t calculateMicros() {
      return constrain(map(position, 0, SERVO_TICKS_PER_RANGE, minMicros, maxMicros), minMicros, maxMicros);
    }
  
    void update(uint64_t servoMask) {
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
};


Motor* motors[MAX_NUM_MOTORS] = {
// PORT           IN1|PWM|EN| FB | SF |BRAKE
//  /*0*/ new Motor(  2,  3, 4,NONE,NONE, true), //left motor
//  /*1*/ new Motor(  7,  5,12,NONE,NONE, true), //right motor
  /*0*/ new Motor(true), //left motor
  /*1*/ new Motor(false), //right motor
  /*2*/ new Motor(),
  /*3*/ new Motor(),
  /*4*/ new Motor(),
  /*5*/ new Motor(),
  /*6*/ new Motor(),
  /*7*/ new Motor()
};


/*
 * holds all the servos
 * if a servo is unused, set the pin to NONE
 * the serco library only allows MIN as low as 544 and MAX as high as 2400
 */
ServoControl* servos[MAX_NUM_SERVOS] = {
// PORT                 PIN| MIN| MAX|POS (0-1440)
  /*0*/ new ServoControl( 6                   ), //neutralization syringe servo
  /*1*/ new ServoControl(10                   ), //collection syringe servo
  /*2*/ new ServoControl(13                   ), //arm servo
  /*3*/ new ServoControl(11                   ), //lidar servo
  /*4*/ new ServoControl(                     ),
  /*5*/ new ServoControl(                     ),
  /*6*/ new ServoControl(                     ),
  /*7*/ new ServoControl(                     )
};

//call this from the setup() function to speed up analogRead
void EnableFastAnalogRead() {
  // set prescale to 16
  sbi(ADCSRA,ADPS2);
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
}

void setup() {
  EnableFastAnalogRead(); //enable fast analog reading

  Serial.begin(9600);
  
  tank.init();
        
  for(uint8_t i=0; i<MAX_NUM_MOTORS; i++) {
    if (servos[i]->pin != NONE) {
      servos[i]->servo.attach(servos[i]->pin);
      servos[i]->servo.writeMicroseconds(servos[i]->micros);
    }
  }

  // Retrieve the destination
  while (!enes.retrieveDestination()) {
    enes.println("Unable to retrieve location");
  }

  enes.print("My destination is at ");
  enes.print(enes.destination.x);
  enes.print(",");
  enes.println(enes.destination.y);


  //set up a timer interrupt every millisecond
//  OCR0A = 0x01; //millis() counter uses 0
//  TIMSK0 |= _BV(OCIE0A);
}

//uint32_t timer;
uint64_t servoMask = 0;

void updateDevices(uint32_t loopTimer) {
//timer interrupt function (every millisecond)
//SIGNAL(TIMER0_COMPA_vect) {
//  togglePin.toggle();
//  return;
//  uint32_t start = micros();
  
  //update motors (including shift registers) every 16 millis
  if (loopTimer % 16 == 0) {
    for(uint8_t i=0; i<MAX_NUM_MOTORS; i++) {
      motors[i]->update();
    }
  }
  

  //shift the mask to the left by 1, wrapping around when it reaches the end
  servoMask = servoMask << 1;
  if (servoMask == 0) servoMask = 1;
  for(uint8_t i=0; i<MAX_NUM_SERVOS; i++) {
    servos[i]->update(servoMask);
  }
  //timer = micros() - start;
}



/*
 * returns -1 when an invalid char is passed in
 */
int8_t charToHex(char c) {
  if      (c >= '0' && c <= '9') return (c - '0');
  else if (c >= 'a' && c <= 'f') return (c - 'a') + 10;
  else if (c >= 'A' && c <= 'F') return (c - 'A') + 10;
  else                           return -1;
}

const char hexChars[16] = {
  '0', '1', '2', '3',
  '4', '5', '6', '7',
  '8', '9', 'A', 'B',
  'C', 'D', 'E', 'F'
};

char hexToChar(uint8_t h) {
  if (h >= 16) return -1;
  return hexChars[h];
}

uint32_t loopTimer = 0;

//int pos = 0;

//Encoder myEnc(3, 40);
void loop() {
  
//  motors[0]->setPower(100);
//  Serial.print(motors[0]->encoderPos);
//  Serial.print("   ");
//  Serial.println(motors[0]->currentDraw);
//  Serial.println(myEnc.read());

//  if(servos[0]->isDone) {
//    pos = 1440 - pos;
//    servos[0]->set(pos, 5);
//  }
  
    // print the string when a newline arrives:
//  if (stringComplete) {
//    stringComplete = false;
//    Serial.println(inputString);
//    //TODO: commands
//    // clear the string:
//    inputString = "";
//  }

  updateDevices(loopTimer);

//  delayMicroseconds(200);
//  delayMicroseconds(150);

  //make sure the loop runs no faster than once every 1 millisecond
  int16_t difference = millis() - loopTimer;
  if (difference >= 100) {
//    Serial.print(loopTimer);
//    Serial.print(" skip ");
//    Serial.println(difference);
    loopTimer += difference;
  } else {
    loopTimer += 1;
    while (millis() < loopTimer);
  }

  // Update the OSV's current location
  if (enes.updateLocation()) {
    enes.println("Huzzah! Location updated!");
    enes.print("My x coordinate is ");
    enes.println(enes.location.x);
    enes.print("My y coordinate is ");
    enes.println(enes.location.y);
    enes.print("My theta is ");
    enes.println(enes.location.theta); //-pi to pi
  } else {
    enes.println("Sad trombone... I couldn't update my location");
  }

  enes.navigated();

  // Transmit the initial pH of the pool
  enes.baseObjective(2.7);

  // Transmit the final pH of the pool
  enes.baseObjective(7.0);
}

