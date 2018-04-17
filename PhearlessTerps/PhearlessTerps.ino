#include <Enes100.h>
#include <Servo.h>
#include "Hardware.h"
#include <Stepper.h>


#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define TIME_LIMIT  1000L * 60 * 5 //5 minutes
#define TIME_BUFFER 5000 //5 seconds

#define STEPS 200
#define STEPPER_SPEED 60

//#include "Util.h"
#define PH_SENSOR_PIN A0          //pH meter Analog output to Arduino Analog Input 0
unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10], temp;
float phValue = 0;
uint32_t navigatedTime = 0;
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

Stepper stepper(STEPS, 4,5,6,7);

/* Create a new Enes100 object
   Parameters:
    string teamName
    int teamType
    int markerId
    int rxPin
    int txPin
*/
Enes100 enes("pHearless Terps", CHEMICAL, 5, 8, 9);

double dabs(double val) {
  if (val > 0) return val;
  if (val < 0) return -val;
  return 0;
}

static inline double sgn(double val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

#define MAX_NUM_MOTORS 2
//#define MAX_NUM_SERVOS 4

Motor* motors[MAX_NUM_MOTORS] = {
  // PORT           IN1|PWM|EN| FB | SF |BRAKE
//  /*0*/ new Motor(  5,  7, 12, NONE, NONE, true), //left motor
//  /*1*/ new Motor(  2,  3, 4, NONE, NONE, true) //right motor
  /*0*/ new Motor(  3,  2, 4, NONE, NONE, true), //right motor
  /*1*/ new Motor(  7,  5, 12, NONE, NONE, true) //left motor
};

///*
//   holds all the servos
//   if a servo is unused, set the pin to NONE
//   the serco library only allows MIN as low as 544 and MAX as high as 2400
//*/
//ServoControl* servos[MAX_NUM_SERVOS] = {
//  // PORT                 PIN| MIN| MAX|POS (0-1440)
//  /*0*/ new ServoControl( 6                   ), //neutralization syringe servo
//  /*1*/ new ServoControl(10                   ), //collection syringe servo
//  /*2*/ new ServoControl(13                   ), //arm servo
//  /*3*/ new ServoControl(11                   ) //lidar servo
//};

Servo armServo;
Servo lidarServo;
Servo collectServo;

//call this from the setup() function to speed up analogRead
void EnableFastAnalogRead() {
  // set prescale to 16
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
}

void setup() {

  EnableFastAnalogRead(); //enable fast analog reading

  Serial.begin(9600);

//  for (uint8_t i = 0; i < MAX_NUM_SERVOS; i++) {
//    if (servos[i]->pin != NONE) {
//      servos[i]->servo.attach(servos[i]->pin);
//      servos[i]->servo.writeMicroseconds(servos[i]->micros);
//    }
//  }

  armServo.attach(13);
  lidarServo.attach(11);
  collectServo.attach(10);

  stepper.setSpeed(STEPPER_SPEED);
  lox.begin();

  while (!enes.updateLocation())
  {
    enes.println("Unable to retrieve location");
  }


  enes.print("osv: ");
  enes.print(enes.location.x);
  enes.print(",");
  enes.println(enes.location.y);

  while (!enes.retrieveDestination())
  {
    enes.println("Unable to retrieve destination");
  }


  enes.print("My destination is at ");
  enes.print(enes.destination.x);
  enes.print(",");
  enes.println(enes.destination.y);
}

//uint64_t servoMask = 0;

void updateDevices(uint32_t loopTimer) {
  //every 16 millis
  if (loopTimer % 16 == 0) {
    //update motors
    for (uint8_t i = 0; i < MAX_NUM_MOTORS; i++) {
      motors[i]->update();
    }
  }

//  //shift the mask to the left by 1, wrapping around when it reaches the end
//  servoMask = servoMask << 1;
//  if (servoMask == 0) servoMask = 1;
//  for (uint8_t i = 0; i < MAX_NUM_SERVOS; i++) {
//    servos[i]->update(servoMask);
//  }

  //make sure the loop runs no faster than once every 1 millisecond
  int16_t difference = millis() - loopTimer;
  if (difference >= 100) {
    loopTimer += difference;
  } else {
    loopTimer += 1;
    while (millis() < loopTimer);
  }
}

/*
   returns -1 when an invalid char is passed in
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

#define ROCKS_X_POS 1.5
#define GOAL_X_POS 3
#define DRIVE_TO_AVOID_DIST .25

#define DRIVE_OVER_ROCKS 0
#define TURN_DOWNSTREAM 1
#define DRIVE_DOWNSTREAM 2
#define TURN_LEFT 3
#define TURN_RIGHT 4
#define DRIVE_TO_AVOID 5
#define TURN_TO_GOAL 6
#define DRIVE_TO_GOAL 7

#define ARMDOWN 100
#define MEASUREPH 101
#define BASE_COLLECTION 102
#define CHECK_PH 103
#define MIX_FORWARD 104
#define MIX_BACKWARD 105

#define STOP 255

uint8_t state = DRIVE_OVER_ROCKS;

uint32_t stateTimer = 0;
uint8_t stateCounter = 0;

#define LIDAR_NONE 0
#define LIDAR_LEFT 1
#define LIDAR_RIGHT 2

#define LIDAR_THRESHOLD 300

uint8_t lidarScan() {
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  double value = 1000;
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    value = measure.RangeMilliMeter;
    //    if (measure.RangeMilliMeter < 300) {
    //      double value = measure.RangeMilliMeter;
    //      enes.print("Object! (mm): "); enes.println(value);
    //    }
  }

  uint32_t timer = millis() % 1024;
  if (timer > 512) {
    lidarServo.write(45);
//    servos[3]->set(200);
    if (timer > 512 + 200 && value < LIDAR_THRESHOLD) return LIDAR_LEFT;
  } else {
    lidarServo.write(135);
//    servos[3]->set(1200);
    if (timer > 200 && value < LIDAR_THRESHOLD) return LIDAR_RIGHT;
  }
  return LIDAR_NONE;
}

uint8_t phIndex = 0;
boolean getPH () {
  if (millis() >= stateTimer + 10) {
    buf[phIndex] = analogRead(PH_SENSOR_PIN);
    phIndex++;
    stateTimer = millis();
    if (phIndex >= 10) {
      for (int i = 0; i < 9; i++) //sort the analog from small to large
      {
        for (int j = i + 1; j < 10; j++)
        {
          if (buf[i] > buf[j])
          {
            temp = buf[i];
            buf[i] = buf[j];
            buf[j] = temp;
          }
        }
      }
      avgValue = 0;
      for (int i = 2; i < 8; i++)               //take the average value of 6 center sample
        avgValue += buf[i];
      phValue = (float)avgValue * 5.0 / 1024 / 6; //convert the analog into millivolt
      phValue = 3.6 * phValue + 2.4;                //convert the millivolt into pH value
      return true;
    }
  }
  return false;
}
#define TURN_GAIN 0.8
#define TURN_DEADZONE 0.15
#define TURN_MIN_SPEED 0.7
#define TURN_MAX_SPEED 1
#define closeToZero 1.0e-3

boolean turn(double targetHeading) {
  boolean done = false;
  double targetHeadingX = cos(targetHeading);
  double targetHeadingY = sin(targetHeading);

  double directionX = cos(enes.location.theta);
  double directionY = sin(enes.location.theta);

  //find the "signed angular separation", the magnitude and direction of the error

  //cross product of two 2D vectors in 3D space
  double crossZ = targetHeadingX * directionY - targetHeadingY * directionX;

  double angleRadians = 0;
  // If the vectors are too closely aligned, then return zero for
  // separation.
  if (dabs(crossZ) >= closeToZero) {

    // To get the angle:
    // a dot b = a * b * cos(angle)
    // so angle = acos[ (a dot b) / (a * b) ]
    // Make sure a * b is not too close to zero 0
    double lengths = sqrt(targetHeadingX * targetHeadingX + targetHeadingY * targetHeadingY) * sqrt(directionX * directionX + directionY * directionY);
    if (lengths >= closeToZero) {
      // this is really an error, but to keep the OSV from crashing
      double dot = targetHeadingX * directionX + targetHeadingY * directionY;
      angleRadians = sgn(crossZ) * acos(dot / lengths);

    }
  }
  enes.print("signed angular separation: ");
  enes.println(angleRadians);

  //              This graph shows angle error vs. rotation correction
  //              ____________________________
  //              | correction.       ____   |
  //              |           .      /       |
  //              |           .   __/        |
  //              | ........__.__|.......... |
  //              |      __|  .     error    |
  //              |     /     .              |
  //              | ___/      .              |
  //              |__________________________|
  //
  //              The following code creates this graph:

  //scale the signedAngularSeparation by a constant
  double rotationCorrection = TURN_GAIN * angleRadians;
  //                rotationCorrection = GYRO_PID.computeCorrection(0, angleRadians);

  if (dabs(rotationCorrection) > TURN_MAX_SPEED) {
    //cap the rotationCorrection at +/- TURN_MAX_SPEED
    rotationCorrection = sgn(rotationCorrection) * TURN_MAX_SPEED;
  } else if (dabs(rotationCorrection) < TURN_DEADZONE) {
    //set it to 0 if it is in the deadzone
    rotationCorrection = 0;
    done = true;
  } else if (dabs(rotationCorrection) < TURN_MIN_SPEED) {
    //set it to the minimum if it is below
    rotationCorrection = sgn(rotationCorrection) * TURN_MIN_SPEED;
  }

  enes.print("rotationCorrection: ");
  enes.println(rotationCorrection);

  motors[0]->setPower(200 * rotationCorrection);
  motors[1]->setPower(-200 * rotationCorrection);

  return done;
}

double startX, startY;

void loop() {

  updateDevices(loopTimer);

  uint8_t lidarDetection = lidarScan();
  Serial.println(lidarDetection);

  //every 16 millis (on a different count than the motors) update OSV location
//  if (loopTimer % 16 == 8)
  enes.updateLocation();
  enes.print("state: ");
  enes.println(state);

  if (state == DRIVE_OVER_ROCKS) {
    motors[0]->setPower(200);
    motors[1]->setPower(200);
    if (enes.location.x >= ROCKS_X_POS) {
      state = TURN_DOWNSTREAM;
    }
  } else if (state == TURN_DOWNSTREAM) {
    if (turn(0)) {
      motors[0]->setPower(0);
      motors[1]->setPower(0);
      state = DRIVE_DOWNSTREAM;
    }
  } else if (state == DRIVE_DOWNSTREAM) {

    motors[0]->setPower(200);
    motors[1]->setPower(200);
    double thetaError = dabs(enes.location.theta);
    if (lidarDetection == LIDAR_LEFT) {
      state = TURN_RIGHT;
    } else if (lidarDetection == LIDAR_RIGHT) {
      state = TURN_LEFT;
    } else if (thetaError >= PI / 16) {
      state = TURN_DOWNSTREAM;
    } else if (enes.location.x >= GOAL_X_POS) {
      motors[0]->setPower(0);
      motors[1]->setPower(0);
      state = TURN_TO_GOAL;
    }
  } else if (state == TURN_LEFT) {
    if (turn(-PI / 4.0)) {
      motors[0]->setPower(0);
      motors[1]->setPower(0);
      startX = enes.location.x;
      startY = enes.location.y;
      state = DRIVE_TO_AVOID;
    }
  } else if (state == TURN_RIGHT) {
    if (turn(PI / 4.0)) {
      motors[0]->setPower(0);
      motors[1]->setPower(0);
      startX = enes.location.x;
      startY = enes.location.y;
      state = DRIVE_TO_AVOID;
    }
  } else if (state == DRIVE_TO_AVOID) {
    motors[0]->setPower(200);
    motors[1]->setPower(200);
    double dX = startX - enes.location.x;
    double dY = startY - enes.location.y;
    double dist = sqrt(dX * dX + dY * dY);

    if (lidarDetection == LIDAR_LEFT) {
      state = TURN_RIGHT;
    } else if (lidarDetection == LIDAR_RIGHT) {
      state = TURN_LEFT;
    } else if (dist > DRIVE_TO_AVOID_DIST) {
      state = TURN_DOWNSTREAM;
    }
  } else if (state == TURN_TO_GOAL) {
    double x = enes.destination.x - enes.location.x;
    double y = enes.destination.y - enes.location.y;
    double desiredTheta = atan(y / x);
    if (turn(desiredTheta))
    {
      motors[0]->setPower(0);
      motors[1]->setPower(0);
      state = STOP;
    }
  } else if (state == DRIVE_TO_GOAL) {

    motors[0]->setPower(200);
    motors[1]->setPower(200);
    double x = enes.destination.x - enes.location.x;
    double y = enes.destination.y - enes.location.y;
    double desiredTheta = atan(y / x);
    double thetaError = dabs(desiredTheta - enes.location.theta);
    double distance = sqrt(x * x + y * y);
    if (thetaError >= PI / 16)
    {
      state = TURN_TO_GOAL;
    } else if (distance <= .250) {
      motors[0]->setPower(0);
      motors[1]->setPower(0);
      stateTimer = millis();
      state = ARMDOWN;
      navigatedTime = millis();
      enes.navigated();
//      servos[2]->set(720); //should lower green arm 90 degrees
      armServo.write(90);
    }
  } else if (state == ARMDOWN) {
    if (millis() > stateTimer + 1000) {
      //delay(1000);
      state = MEASUREPH;
      stateTimer = millis();
      phIndex = 0;
    }
  } else if (state == MEASUREPH) {
    if (getPH()) {
      enes.baseObjective(phValue); //transmit the inital pH of the pool
//      servos[1]->set(720); //should raise the retaining servo arm 90 degrees to allow syring to move
      collectServo.write(90);
      stateTimer = millis();
      state = BASE_COLLECTION;
    }
  } else if (state == BASE_COLLECTION) {
    if (millis() > stateTimer + 1000) {
      state = CHECK_PH;
    }
  } else if (state == CHECK_PH) {
       if (phValue >= 6.5) {
        enes.baseObjective(phValue);
        state = STOP;
      }
      else {
        stepper.step(STEPS);
        stateTimer = millis();
        motors[0]->setPower(200);
        motors[1]->setPower(200);
        stateCounter = 0;
        state = MIX_FORWARD;
      }
  }
  else if (state == MIX_FORWARD){
    if (millis() > stateTimer + 250) {
       motors[0]->setPower(-200);
       motors[1]->setPower(-200);
       state = MIX_BACKWARD;
    }
  }
  else if (state == MIX_BACKWARD){
    if (millis() > stateTimer + 250) {
       motors[0]->setPower(200);
       motors[1]->setPower(200);
       stateCounter++;
       if (stateCounter >= 4){
        state = CHECK_PH; 
       }
       else {
        state = MIX_FORWARD;
       }
    }
  }
  else if (state == STOP)
  {
    motors[0]->setPower(0);
    motors[1]->setPower(0);
  }
  if (state != STOP && millis() >= navigatedTime + TIME_LIMIT - TIME_BUFFER) {
     enes.baseObjective(phValue);
     state = STOP;
  }
}
