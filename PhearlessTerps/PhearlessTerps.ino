#include <Enes100.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include <Servo.h>
#include "Hardware.h"
//#include "Util.h"
#define SensorPin 0          //pH meter Analog output to Arduino Analog Input 0
unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10], temp;
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

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
#define MAX_NUM_SERVOS 4

Motor* motors[MAX_NUM_MOTORS] = {
  // PORT           IN1|PWM|EN| FB | SF |BRAKE
  /*0*/ new Motor(  5,  7, 12, NONE, NONE, true), //left motor
  /*1*/ new Motor(  2,  3, 4, NONE, NONE, true), //right motor
};


/*
   holds all the servos
   if a servo is unused, set the pin to NONE
   the serco library only allows MIN as low as 544 and MAX as high as 2400
*/
ServoControl* servos[MAX_NUM_SERVOS] = {
  // PORT                 PIN| MIN| MAX|POS (0-1440)
  /*0*/ new ServoControl( 6                   ), //neutralization syringe servo
  /*1*/ new ServoControl(10                   ), //collection syringe servo
  /*2*/ new ServoControl(13                   ), //arm servo
  /*3*/ new ServoControl(11                   ) //lidar servo
};

//call this from the setup() function to speed up analogRead
void EnableFastAnalogRead() {
  // set prescale to 16
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
}

void setup() {

  EnableFastAnalogRead(); //enable fast analog reading

  for (uint8_t i = 0; i < MAX_NUM_SERVOS; i++) {
    if (servos[i]->pin != NONE) {
      servos[i]->servo.attach(servos[i]->pin);
      servos[i]->servo.writeMicroseconds(servos[i]->micros);
    }
  }

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

uint64_t servoMask = 0;

void updateDevices(uint32_t loopTimer) {
  //every 16 millis
  if (loopTimer % 16 == 0) {
    //update motors
    for (uint8_t i = 0; i < MAX_NUM_MOTORS; i++) {
      motors[i]->update();
    }
  }

  //shift the mask to the left by 1, wrapping around when it reaches the end
  servoMask = servoMask << 1;
  if (servoMask == 0) servoMask = 1;
  for (uint8_t i = 0; i < MAX_NUM_SERVOS; i++) {
    servos[i]->update(servoMask);
  }

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

const uint8_t FOURFWD = 0;
const uint8_t FOURTURN = 1;
const uint8_t TURN1 = 2;
const uint8_t TURN2 = 3;
const uint8_t TURN3 = 4;
const uint8_t STOP = 5;
const uint8_t NAVTURN = 6;
const uint8_t NAVFWD = 7;
const uint8_t BROADCAST = 8;

uint8_t state = FOURFWD;

uint32_t stateTimer = 0;
int stateLength = 1000;

const double TURN_GAIN = 0.8;
const double TURN_DEADZONE = 0.15;
const double TURN_MIN_SPEED = 0.7;
const double TURN_MAX_SPEED = 1;
const double closeToZero = 1.0e-3;

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

void loop() {

  updateDevices(loopTimer);
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    if (measure.RangeMilliMeter < 300) {
      double value = measure.RangeMilliMeter;
      enes.print("Object! (mm): "); enes.println(value);
    }
  }

  //every 16 millis (on a different count than the motors) update OSV location
  //if (loopTimer % 16 == 8)
  enes.updateLocation();
  enes.print("state: ");
  enes.println(state);
  if (state == FOURTURN)
  {


    enes.print("osv (x, y, theta): (");
    enes.print(enes.location.x);
    enes.print(", ");
    enes.print(enes.location.y);
    enes.print(", ");
    enes.print(enes.location.theta);
    enes.println(")");

    enes.print("dest (x, y, theta): (");
    enes.print(enes.destination.x);
    enes.print(", ");
    enes.print(enes.destination.y);
    enes.print(", ");
    enes.print(enes.destination.theta);
    enes.println(")");

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

    double desiredTheta = 0;
    if (turn(desiredTheta))
    {
      motors[0]->setPower(0);
      motors[1]->setPower(0);
      state = FOURFWD;
    }

  } else if (state == FOURFWD)
  {
    stateTimer = millis();
    motors[0]->setPower(200);
    motors[1]->setPower(200);
    double thetaError = dabs(enes.location.theta);
    if (thetaError >= PI / 16)
    {
      state = FOURTURN;
    } else if (enes.location.x <= 2)
    {

      enes.print("osv (x, y, theta): (");
      enes.print(enes.location.x);
      enes.print(", ");
      enes.print(enes.location.y);
      enes.print(", ");
      enes.print(enes.location.theta);
      enes.println(")");

      enes.print("dest (x, y, theta): (");
      enes.print(enes.destination.x);
      enes.print(", ");
      enes.print(enes.destination.y);
      enes.print(", ");
      enes.print(enes.destination.theta);
      enes.println(")");


      enes.print("thetaError: ");
      enes.println(thetaError);


    }
    else
    {
      motors[0]->setPower(0);
      motors[1]->setPower(0);
      state = NAVTURN;
    }
  } else if (state == TURN1)
  {
    if (turn(PI / 2.0))
    {
      state = TURN2;
    }

  } else if (state == TURN2)
  {
    if (turn(PI))
    {
      state = TURN3;
    }

  } else if (state == TURN3)
  {
    if (turn(-PI / 2.0))
    {
      motors[0]->setPower(0);
      motors[1]->setPower(0);
      state = STOP;
    }

  }
  else if (state == NAVTURN)
  {
    enes.println(enes.location.x);
    double x = enes.destination.x - enes.location.x;
    double y = enes.destination.y - enes.location.y;
    double desiredTheta = atan(y / x);
    if (turn(desiredTheta))
    {
      motors[0]->setPower(0);
      motors[1]->setPower(0);
      state = STOP;
    }

  } else if (state == NAVFWD)
  {

    stateTimer = millis();
    motors[0]->setPower(200);
    motors[1]->setPower(200);
    double x = enes.destination.x - enes.location.x;
    double y = enes.destination.y - enes.location.y;
    double desiredTheta = atan(y / x);
    double thetaError = dabs(desiredTheta - enes.location.theta);
    double distance = sqrt(x * x + y * y);
    if (thetaError >= PI / 16)
    {
      state = NAVTURN;
    } else if (distance >= .250)
    {

      enes.print("osv (x, y, theta): (");
      enes.print(enes.location.x);
      enes.print(", ");
      enes.print(enes.location.y);
      enes.print(", ");
      enes.print(enes.location.theta);
      enes.println(")");

      enes.print("dest (x, y, theta): (");
      enes.print(enes.destination.x);
      enes.print(", ");
      enes.print(enes.destination.y);
      enes.print(", ");
      enes.print(enes.destination.theta);
      enes.println(")");

      enes.print("x, y: ");
      enes.print(x);
      enes.print(", ");
      enes.println(y);


      enes.print("desiredTheta: ");
      enes.println(desiredTheta);
      enes.print("thetaError: ");
      enes.println(thetaError);
      enes.print("distance: ");
      enes.println(distance);


    }
    else
    {
      motors[0]->setPower(0);
      motors[1]->setPower(0);
      state = STOP;
    }
  }


  if (state == STOP)
  {
    motors[0]->setPower(0);
    motors[1]->setPower(0);
  }





}
