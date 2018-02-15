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

#define MOTOR_CONTROL_MAX_OUT 255

//speed is encoder ticks of error
//p = power per tick per 16 millis
//at 45 tick error, power is 255
//p = 255/45 = 5.666666667
#define MOTOR_CONTROL_P_NUMERATOR 48
#define MOTOR_CONTROL_P_DENOMINATOR 32

//i = power added per tick of error per 16 millis
#define MOTOR_CONTROL_I_NUMERATOR 8
#define MOTOR_CONTROL_I_DENOMINATOR 32
#define MOTOR_CONTROL_MAX_I (MOTOR_CONTROL_MAX_OUT * MOTOR_CONTROL_I_DENOMINATOR) / MOTOR_CONTROL_I_NUMERATOR
//#define MOTOR_CONTROL_MAX_I 32767

//d = power per tick of delta input
#define MOTOR_CONTROL_D_NUMERATOR 0
#define MOTOR_CONTROL_D_DENOMINATOR 32




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
class Motor {
  public:
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
    unsigned hasEncoder : 8;
    /*
     * the motor's power, including direction
     * ranges from -255 to 255 (-256 is illegal)
     */
    signed power : 16;
    /*
     * speed in encoder ticks per second
     * ranges from -255 to 255 (-256 is illegal)
     */
    signed speed : 16;
    /*
     * true to brake when stopping
     * false to coast when stopping
     */
    unsigned brake : 8;
    /*
     * the value from the encoder
     */
    signed encoderPos : 32;
    /*
     * the encoder position last time the speed control loop was run
     */
    signed previousEncPos : 32;
    /*
     * the current being drawn by the motor
     * a negative value means there is no sensor or it has not been read yet
     */
    signed currentDraw : 16;
    /*
     * the status flag of the motor controller
     */
    unsigned isOK : 8;
    /*
     * 0 for motor cuttent control
     * 1 for speed control
     */
    unsigned useSpeedControl : 8;
  
    Motor(int8_t pwmPin1, int8_t analogPin1, boolean brake1) {
      pwmPin = pwmPin1;
      analogPin = analogPin1;
      power = 0;
      speed = 0;
      brake = brake1;
      encoderPos = 0;
      previousEncPos = 0;
      currentDraw = NONE;
      isOK = true;
  
      hasEncoder = false;
      useSpeedControl = false;
    }
  
    Motor(int8_t pwmPin1, boolean brake1) {
      pwmPin = pwmPin1;
      analogPin = NONE;
      power = 0;
      speed = 0;
      brake = brake1;
      encoderPos = 0;
      previousEncPos = 0;
      currentDraw = NONE;
      isOK = true;
  
      hasEncoder = false;
      useSpeedControl = false;
    }
  
    Motor() {
      pwmPin = NONE;
      analogPin = NONE;
      power = 0;
      speed = 0;
      brake = true;
      encoderPos = 0;
      previousEncPos = 0;
      currentDraw = NONE;
      isOK = true;
  
      hasEncoder = false;
      useSpeedControl = false;
    }
  
    boolean setPower(int16_t power1) {
      if (power1 < -255 || power1 > 255) return false;
      if (pwmPin == NONE) return false;
      if (useSpeedControl) return false;
    
      power = power1; //constrain(power1, -255, 255);
      
      return true;
    }
  
    boolean setSpeed(int16_t speed1) {
      if (speed1 < -255 || speed1 > 255) return false;
      if (pwmPin == NONE) return false;
      if (!useSpeedControl) return false;
    
      speed = speed1; //constrain(speed1, -255, 255);
      
      return true;
    }
  
    virtual void update() {
      //read the current draw if it has an analog pin
      if(analogPin != NONE) {
        currentDraw = analogRead(analogPin);
      }
    }

    boolean setUseSpeedControl(boolean useSpeedControl1) {
      if (hasEncoder) {
        useSpeedControl = useSpeedControl1;
        return true;
      } else {
        return false;
      }
    }
};


class MotorEnc : public Motor {
  public:
    /*
     * used for the speed control loop to keep track of 
     * the change in speed
     */
    signed encoderDelta : 16;
    /*
     * used for the speed control loop to keep track of
     * the constant speed needed
     */
    signed iTerm : 16;

    int16_t encoderDeltaHistory[4];
    uint8_t encoderHistoryIndex;
    
    Encoder* encoder;
  
    MotorEnc(int8_t pwmPin1, int8_t analogPin1, uint8_t encoderPin1, uint8_t encoderPin2, boolean brake1)
      :Motor(pwmPin1, analogPin1, brake1)
      {
  
      encoder = new Encoder(encoderPin1, encoderPin2);
      hasEncoder = true;
      encoderDelta = 0;
      iTerm = 0;
      for(encoderHistoryIndex = 4; encoderHistoryIndex > 0; encoderHistoryIndex--)
        encoderDeltaHistory[encoderHistoryIndex-1] = 0;
    
    }
    
  
    MotorEnc(int8_t pwmPin1, uint8_t encoderPin1, uint8_t encoderPin2, boolean brake1)
      :Motor(pwmPin1, brake1)
      {
  
      encoder = new Encoder(encoderPin1, encoderPin2);
      hasEncoder = true;
      encoderDelta = 0;
      iTerm = 0;
      for(encoderHistoryIndex = 4; encoderHistoryIndex > 0; encoderHistoryIndex--)
        encoderDeltaHistory[encoderHistoryIndex-1] = 0;
    }
  
    void update() {
      Motor::update();
      encoderPos = encoder->read();

      //encoder for a neverest does 3 ticks per millisecond
      //the max is 4 ticks per millisecond -> 64 ticks per 16 millis
      int16_t lastEncoderDelta = encoderDelta;
      int16_t tempDelta = (encoderPos - previousEncPos);
      encoderDelta -= encoderDeltaHistory[encoderHistoryIndex];
      encoderDeltaHistory[encoderHistoryIndex++] = tempDelta;
      encoderHistoryIndex &= 3;
      encoderDelta += tempDelta;
      
      if (useSpeedControl) {
        if (speed == 0) {
          power = 0;
        } else {
          // Compute all the working error variables
          int16_t error = speed - encoderDelta;
          
          iTerm += error;
          if (iTerm >  MOTOR_CONTROL_MAX_I) iTerm =  MOTOR_CONTROL_MAX_I;
          if (iTerm < -MOTOR_CONTROL_MAX_I) iTerm = -MOTOR_CONTROL_MAX_I;

          // compute dInput instead of dError to avoid spikes
          int16_t dInput = encoderDelta - lastEncoderDelta;
    
          // Compute PID Output
          power = (error * MOTOR_CONTROL_P_NUMERATOR) / MOTOR_CONTROL_P_DENOMINATOR
            + (iTerm * MOTOR_CONTROL_I_NUMERATOR) / MOTOR_CONTROL_I_DENOMINATOR
            - (dInput * MOTOR_CONTROL_D_NUMERATOR) / MOTOR_CONTROL_D_DENOMINATOR;

          if (power >  MOTOR_CONTROL_MAX_OUT) power =  MOTOR_CONTROL_MAX_OUT;
          if (power < -MOTOR_CONTROL_MAX_OUT) power = -MOTOR_CONTROL_MAX_OUT;
          
          Serial.print(speed);
          Serial.print(",");
          Serial.print(encoderDelta);
          Serial.print(",");
          Serial.print(error);
          Serial.print(",");
          Serial.print(power);
          Serial.print("\n");
          
        
        }
      }
      // Remember some variables for next time
      previousEncPos = encoderPos;
      
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

class AnalogPort {
  public:
    /*
     * Which pin the servo is connected to
     */
    signed pin : 8;
    /*
     * the reading from that pin
     * a negative value means there is no sensor or it has not been read yet
     */
    signed reading : 16;

    AnalogPort(uint8_t pin1) {
      pin = pin1;
      reading = -1;
    }
    
    AnalogPort(){
      pin = NONE;
      reading = -1;
    }

    void update() {
      if (pin != NONE) {
        reading = analogRead(pin);
      }
    }
};


Motor* motors[MAX_NUM_MOTORS] = {
// PORT              PWM|ANALOG|ENCODER1|ENCODER2|BRAKE
  /*0*/ new MotorEnc(10,  A8,         3,      40, true ),
  /*1*/ new MotorEnc(11,  A9,         2,      41, true ),
  /*2*/ new MotorEnc( 4, A10,        18,      42, true ),
  /*3*/ new MotorEnc( 5, A11,        19,      43, true ),
  /*4*/ new Motor(    6, A12,                     true ),
  /*5*/ new Motor(    7, A13,                     true ),
  /*6*/ new Motor(    8, A14,                     true ),
  /*7*/ new Motor(    9, A15,                     true )
};


/*
 * holds all the servos
 * if a servo is unused, set the pin to NONE
 * the serco library only allows MIN as low as 544 and MAX as high as 2400
 */
ServoControl* servos[MAX_NUM_MOTORS] = {
// PORT                  PIN|MIN|MAX|POS (0-1440)
  /*0*/ new ServoControl(22, 544,2400         ),
  /*1*/ new ServoControl(23                   ),
  /*2*/ new ServoControl(24                   ),
  /*3*/ new ServoControl(25                   ),
  /*4*/ new ServoControl(26                   ),
  /*5*/ new ServoControl(                     ),
  /*6*/ new ServoControl(                     ),
  /*7*/ new ServoControl(                     )
};

AnalogPort* analogPorts[MAX_NUM_MOTORS] = {
  /*0*/ new AnalogPort(A0),
  /*1*/ new AnalogPort(A1),
  /*2*/ new AnalogPort(A2),
  /*3*/ new AnalogPort(A3),
  /*4*/ new AnalogPort(A4),
  /*5*/ new AnalogPort(A5),
  /*6*/ new AnalogPort(A6),
  /*7*/ new AnalogPort(A7)
};


//call this from the setup() function to speed up analogRead
void EnableFastAnalogRead() {
  // set prescale to 16
  sbi(ADCSRA,ADPS2);
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
}


//String inputString = "";         // a String to hold incoming data
//boolean stringComplete = false;  // whether the string is complete

char circularBuffer[256];
uint8_t writeAddress = 0, readAddress = 0;
uint8_t commandsReady = 0;

void setup() {
  EnableFastAnalogRead(); //enable fast analog reading

  Serial.begin(9600);
  Serial2.begin(9600); //TODO increase baud rate if possible

  //initialize shift registers
  for (uint8_t i=0; i<8; i++) {
    shiftClock.pulse(HIGH);
  }
  shiftLatch = HIGH;
  
  for(uint8_t i=0; i<MAX_NUM_MOTORS; i++) {
    //initialize the motor pins
    if(motors[i]->pwmPin != NONE) {
      pinMode(motors[i]->pwmPin, OUTPUT);
      if(motors[i]->analogPin != NONE) {
        pinMode(motors[i]->analogPin, INPUT);
      }
      //the status flag and the 2 digital outputs are on the shift register
    }
    if (servos[i]->pin != NONE) {
      servos[i]->servo.attach(servos[i]->pin);
      servos[i]->servo.writeMicroseconds(servos[i]->micros);
    }
  }

  uint8_t i=0;
  do {
    circularBuffer[i] = '*';
    i++;
  } while(i != 0);
  
//  inputString.reserve(200);

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
    shiftLatch = LOW;
    for(uint8_t i=0; i<MAX_NUM_MOTORS; i++) {
      if(motors[i]->pwmPin != NONE) { //if it is attached

        motors[i]->update();
        int16_t power = motors[i]->power;
        
        //update pin outputs based on motor power
        uint8_t pwmOutput;
        if(power > 0) { //FORWARD
          //write to the shift register pins
          shiftDataOut1 = 1; //EN on the motor controller
          shiftDataOut2 = 0; //IN1 on the motor controller
          //write to the pwm pin
          pwmOutput = power;
        } else if(power < 0) { //REVERSED
          //write to the shift register pins
          shiftDataOut1 = 1; //EN on the motor controller
          shiftDataOut2 = 1; //IN1 on the motor controller
          pwmOutput = 255 + power;
        } else { //STOPPED
          //write to the shift register pins
          shiftDataOut1 = motors[i]->brake; //EN on the motor controller
          shiftDataOut2 = 1; //IN1 on the motor controller
          pwmOutput = 255;
        }
        //write to the pwm pin
        //12 us
        analogWrite(motors[i]->pwmPin, pwmOutput); //IN2 on the motor controller
  
        //shift register clock
        //4 us
        shiftClock = HIGH;
        motors[i]->isOK = shiftDataIn;
      } else {
        //still pulse the shift register clock if there was no motor
        shiftDataOut1 = LOW;
        shiftDataOut2 = LOW;
        shiftClock = HIGH;
      }
      shiftClock = LOW;
    }
    shiftLatch = HIGH;
  }
  

  //shift the mask to the left by 1, wrapping around when it reaches the end
  servoMask = servoMask << 1;
  
  if (servoMask == 0) servoMask = 1;
  for(uint8_t i=0; i<MAX_NUM_MOTORS; i++) {
    servos[i]->update(servoMask);
    analogPorts[i]->update();
  }
  //timer = micros() - start;
}

//int power = 0;

///*
// * only accepts uppercase hex digits
// * returns 255 when an invalid char is passed in
// */
//uint8_t charToHex(char c) {
//  if(c > 47 && c < 58) { //this covers 0-9
//    return c - 48;
//  } else if (c > 64 && c < 71) { // this covers A-F
//    return c - 55;
//  } else {
//    return 255;
//  }
//}


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

int pos = 0;

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
  
//  if(loopTimer % 1000 == 0) {
//    Serial.print(loopTimer);
//    Serial.print(" ");
//    Serial.println(micros());
//    uint8_t i=0;
//    do {  
//      Serial.print(circularBuffer[i]);
//      i++;
//    } while(i != 0);
//    Serial.println();
//    Serial.println(readAddress);
//    Serial.println(writeAddress);
//    Serial.println(commandsReady);
//  }


  while(readAddress != writeAddress && commandsReady) {
    commandsReady--;
    
    uint8_t address;
    uint8_t direction;
    uint8_t power16; //16's place
    uint8_t power1; //1's place
    
    uint8_t brake;
    uint8_t useSpeedControl;

    uint16_t current;
    int32_t encoderPos;
    int16_t encoderDelta;

    uint8_t servoPos256; //256's place
    uint8_t servoPos16; //16's place
    uint8_t servoPos1; //1's place
    uint8_t speed16;
    uint8_t speed1;
    
    char c = circularBuffer[readAddress++]; //readAddress++ increments after evaluating
    switch (c) {
      case 'M':
        c = circularBuffer[readAddress++];
        address = charToHex(c);
        if (address >= MAX_NUM_MOTORS) break;
        
        c = circularBuffer[readAddress++];
        direction = charToHex(c);
        if (direction >= 2) break;
        
        c = circularBuffer[readAddress++];
        power16 = charToHex(c);
        if (power16 >= 16) break;
                
        c = circularBuffer[readAddress++];
        power1 = charToHex(c);
        if (power1 >= 16) break;

        //                           dir=0,1 -> 1,-1    * 16's=0-15 * 16 + 1's=0-15
        motors[address]->setPower( (1 - (2*direction))  *  (power16 * 16 + power1)  );
        
        break;
      case 'D':
        c = circularBuffer[readAddress++];
        address = charToHex(c);
        if (address >= MAX_NUM_MOTORS) break;
        
        c = circularBuffer[readAddress++];
        direction = charToHex(c);
        if (direction >= 2) break;
        
        c = circularBuffer[readAddress++];
        power16 = charToHex(c);
        if (power16 >= 16) break;
                
        c = circularBuffer[readAddress++];
        power1 = charToHex(c);
        if (power1 >= 16) break;

        //                           dir=0,1 -> 1,-1    * 16's=0-15 * 16 + 1's=0-15
        motors[address]->setSpeed( (1 - (2*direction))  *  (power16 * 16 + power1)  );
        
        break;
      case 'L':
        c = circularBuffer[readAddress++];
        address = charToHex(c);
        if (address >= MAX_NUM_MOTORS) break;
        
        c = circularBuffer[readAddress++];
        useSpeedControl = charToHex(c);
        if (useSpeedControl >= 2) break;

        motors[address]->setUseSpeedControl(useSpeedControl);
        break;
      case 'B':
        c = circularBuffer[readAddress++];
        address = charToHex(c);
        if (address >= MAX_NUM_MOTORS) break;
        
        c = circularBuffer[readAddress++];
        brake = charToHex(c);
        if (brake >= 2) break;

        motors[address]->brake = brake;
        break;
      case 'C':
        c = circularBuffer[readAddress++];
        address = charToHex(c);
        if (address >= MAX_NUM_MOTORS) break;

/*
1024
0000000000
AABBBBCCCC

AA = (x >> 8) & 0b1111
BBBB = (x >> 4) & b0b1111
CCCC = x & 0b1111
 */

        if(motors[address]->currentDraw < 0) {
          //4095 for invalid current
          Serial2.print('F'); //256's place
          Serial2.print('F'); //16's place
          Serial2.print('F'); //1's place
        } else {
          current = motors[address]->currentDraw;
          Serial2.print(hexToChar((current >> 8) & 0b1111)); //256's place
          Serial2.print(hexToChar((current >> 4) & 0b1111)); //16's place
          Serial2.print(hexToChar(current & 0b1111)); //1's place
        }
      case 'E':
        c = circularBuffer[readAddress++];
        address = charToHex(c);
        if (address >= MAX_NUM_MOTORS) break;
        
        encoderPos = motors[address]->encoderPos;
        for (int8_t i=7; i>=0; i--) {
          Serial2.print(hexToChar((encoderPos >> (4*i)) & 0b1111)); //16^i's place
        }
//        Serial2.print(hexToChar((encoderPos >> 4*7) & 0b1111)); //16^7's place
//        Serial2.print(hexToChar((encoderPos >> 4*6) & 0b1111)); //16^6's place
//        Serial2.print(hexToChar((encoderPos >> 4*5) & 0b1111)); //16^5's place
//        Serial2.print(hexToChar((encoderPos >> 4*4) & 0b1111)); //16^4's place
//        Serial2.print(hexToChar((encoderPos >> 4*3) & 0b1111)); //16^3's place
//        Serial2.print(hexToChar((encoderPos >> 4*2) & 0b1111)); //256's place
//        Serial2.print(hexToChar((encoderPos >> 4*1) & 0b1111)); //16's place
//        Serial2.print(hexToChar(encoderPos & 0b1111)); //1's place
        break;
      case 'T':
        c = circularBuffer[readAddress++];
        address = charToHex(c);
        if (address >= MAX_NUM_MOTORS) break;

        if (!motors[address]->hasEncoder) break;
        
        encoderDelta = ((MotorEnc*) motors[address])->encoderDelta;
        for (int8_t i=3; i>=0; i--) {
          Serial2.print(hexToChar((encoderDelta >> (4*i)) & 0b1111)); //16^i's place
        }
        
        break;
      case 'W':
        c = circularBuffer[readAddress++];
        address = charToHex(c);
        if (address >= MAX_NUM_MOTORS) break;

        power1 = abs(motors[address]->power);
        direction = (motors[address]->power < 0); // negative->1  positive or zero->0

        Serial2.print(hexToChar(direction));
        for (int8_t i=1; i>=0; i--) {
          Serial2.print(hexToChar((power1 >> (4*i)) & 0b1111)); //16^i's place
        }
        
        break;
      case 'R':
        c = circularBuffer[readAddress++];
        address = charToHex(c);
        if (address >= MAX_NUM_MOTORS) break;

        motors[address]->previousEncPos -= encoderPos;
        motors[address]->encoderPos = 0;
        break;
      case 'P':
        c = circularBuffer[readAddress++];
        address = charToHex(c);
        if (address >= MAX_NUM_MOTORS) break;

        c = circularBuffer[readAddress++];
        servoPos256 = charToHex(c);
        if (servoPos256 >= 16) break;

        c = circularBuffer[readAddress++];
        servoPos16 = charToHex(c);
        if (servoPos16 >= 16) break;
                
        c = circularBuffer[readAddress++];
        servoPos1 = charToHex(c);
        if (servoPos1 >= 16) break;

        servos[address]->set(servoPos256*256 + servoPos16*16 + servoPos1);
        
        break;
      case 'S':
        c = circularBuffer[readAddress++];
        address = charToHex(c);
        if (address >= MAX_NUM_MOTORS) break;

        c = circularBuffer[readAddress++];
        
        servoPos256 = charToHex(c);
        if (servoPos256 >= 16) break;

        c = circularBuffer[readAddress++];
        servoPos16 = charToHex(c);
        if (servoPos16 >= 16) break;
                
        c = circularBuffer[readAddress++];
        servoPos1 = charToHex(c);
        if (servoPos1 >= 16) break;

        
        c = circularBuffer[readAddress++];
        speed16 = charToHex(c);
        if (speed16 >= 16) break;
        
        c = circularBuffer[readAddress++];
        speed1 = charToHex(c);
        if (speed1 >= 16) break;

        servos[address]->set(servoPos256*256 + servoPos16*16 + servoPos1, speed16*16 + speed1);
        break;
      default:
        break;
    }
    //if the command is just a newline or starts with an invalid char, it will be skipped
    while (c != '\n' && c != ';') {
      c = circularBuffer[readAddress++];
    }
  }
}

/*commands:
M - set 'M'otor Power
  char | meaning   | limits
     0 | M
     1 | address     0-7
     2 | direction   0-1
     3 | power       0-255
     4 | power
   No Response
  M30FF - motor 3 forward 255
  M41FF - motor 4 backward 255
  M5080 - motor 5 forward 128
  M6105 - motor 6 backward 5

D - set motor spee'D'
  char | meaning   | limits
     0 | D
     1 | address     0-7
     2 | direction   0-1
     3 | speed       0-255
     4 | speed
   No Response
  D30FF - motor 3 forward 255
  D41FF - motor 4 backward 255
  D5080 - motor 5 forward 128
  D6105 - motor 6 backward 5

L - enable or disable speed control 'L'oop
  char | meaning   | limits
     0 | L
     1 | address         0-7
     2 | speed control?  0-1
   No Response
  L20 - motor 2 speed control off (coast)
  L31 - motor 3 speed control on (float)
  
B - set motor 'B'rake
  char | meaning   | limits
     0 | B
     1 | address     0-7
     2 | brake?      0-1
   No Response
  B20 - motor 2 brake off (coast)
  B31 - motor 3 brake on (float)

E - get 'E'ncoder value
  char | meaning   | limits
     0 | E
     1 | address     0-7
   Response:
  char | meaning   | limits
     0 | encoderPos -2147483648 to 2147483647
     1 | encoderPos = -2^31 to 2^31-1
     2 | encoderPos
     3 | encoderPos
     4 | encoderPos
     5 | encoderPos
     6 | encoderPos
     7 | encoderPos

T - get encoder ticks per 16 millis
  char | meaning   | limits
     0 | T
     1 | address     0-7
   Response:
  char | meaning   | limits
     0 | encoderDelta -32768 to 32767
     1 | encoderDelta = -2^15 to 2^15-1
     2 | encoderDelta
     3 | encoderDelta

W - get power
  char | meaning   | limits
     0 | T
     1 | address     0-7
   Response:
  char | meaning   | limits
     0 | direction   0-1
     1 | power       0-255
     2 | power

R - reset encoder
  char | meaning   | limits
     0 | R
     1 | address     0-7
   No Response
  R2 - reset motor 2's encoder
  R3 - reset motor 3's encoder


C - get 'C'urrent draw
  char | meaning   | limits
     0 | C
     1 | address     0-7
   Response:
  char | meaning   | limits
     0 | current     0-1023
     1 | current
     2 | current
   

     

P - set servo 'P'osition
  char | meaning   | limits
     0 | P
     1 | address     0-7
     2 | position    0-1440
     3 | position
     4 | position
   No Response
   P3B4 - servo 3 to 180 (B4)
   P400 - servo 4 to 0

S - set servo position and 'S'peed
  char | meaning   | limits
     0 | S
     1 | address     0-7
     2 | position    0-1440
     3 | position
     4 | position
     5 | speed       1-64
     6 | speed
   No Response
*/

/*
  https://www.arduino.cc/en/Reference/SerialEvent
  https://www.arduino.cc/en/Tutorial/SerialEvent
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent2() {
  while (Serial2.available()) {
    circularBuffer[writeAddress] = Serial2.read();
    if(circularBuffer[writeAddress] == '\n' || circularBuffer[writeAddress] == ';') commandsReady++;
    writeAddress++;
    
    //get the new byte:
//    char inChar = Serial2.read();
//    //if the incoming character is a newline
//    if (inChar == '\n') {
//      //set a flag so the main loop can do something about it:
//      stringComplete = true;
//    } else {
//      //otherwise, add it to the inputString:
//      inputString += inChar;
//    }
  }
}
