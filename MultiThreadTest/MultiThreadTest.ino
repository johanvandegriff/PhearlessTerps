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

#define TRIG_PIN_1 A4
#define ECHO_PIN_1 A5
#define TRIG_PIN_2 11
#define ECHO_PIN_2 10

#define STEPS 200
#define STEPPER_SPEED 60

//#include "Util.h"
#define PH_SENSOR_PIN A0          //pH meter Analog output to Arduino Analog Input 0
unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10], temp;
float phValue = 0;
uint32_t navigatedTime = 0;
//#include "Adafruit_VL53L0X.h"

//Adafruit_VL53L0X lox = Adafruit_VL53L0X();

Stepper stepper(STEPS, 4,5,6,7);

/* Create a new Enes100 object
   Parameters:
    string teamName
    int teamType
    int markerId
    int rxPin
    int txPin
*/
#define VISION_TARGET_NUMBER 7
Enes100 enes("pHearless Terps", CHEMICAL, VISION_TARGET_NUMBER, 8, 9);

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

Motor* motors[MAX_NUM_MOTORS] = {
  // PORT           IN1|PWM|EN| FB | SF |BRAKE
  /*0*/ new Motor(  3,  2, 4, NONE, NONE, true), //right motor
  /*1*/ new Motor(  7,  5, 12, NONE, NONE, true) //left motor
};

#define ARM_SERVO_PIN 13
//#define LIDAR_SERVO_PIN 11
#define COLLECT_SERVO_PIN 10

Servo armServo;
//Servo lidarServo;
Servo collectServo;

//call this from the setup() function to speed up analogRead
void EnableFastAnalogRead() {
  // set prescale to 16
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
}

boolean myUpdateLocation() {
    if (!enes.updateLocation()) return false;
//    double x2 =   enes.location.x * cos(enes.location.theta) + enes.location.y * sin(enes.location.theta);
//    double y2 = - enes.location.x * sin(enes.location.theta) + enes.location.y * cos(enes.location.theta);
//    x2 += 40;
//    y2 += 10;
//    enes.location.x =   x2 * cos(-enes.location.theta) + y2 * sin(-enes.location.theta);
//    enes.location.y = - x2 * sin(-enes.location.theta) + y2 * cos(-enes.location.theta);
    return true;
}

void printLocation() {
  enes.print("osv: ");
  enes.print(enes.location.x);
  enes.print(",");
  enes.print(enes.location.y);
  enes.print(",");
  enes.println(enes.location.theta);  
}

void setup() {

  EnableFastAnalogRead(); //enable fast analog reading

  //enesintln("test");

// Serial.begin(9600);

  armServo.attach(ARM_SERVO_PIN);
//  lidarServo.attach(LIDAR_SERVO_PIN);
  collectServo.attach(COLLECT_SERVO_PIN);

  armServo.write(165);

  stepper.setSpeed(STEPPER_SPEED);
//  lox.begin();

  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  
  while (!myUpdateLocation())
  {
    enes.println("Unable to retrieve location");
  }

  printLocation();

  while (!enes.retrieveDestination())
  {
    enes.println("Unable to retrieve destination");
  }


  enes.print("My destination is at ");
  enes.print(enes.destination.x);
  enes.print(",");
  enes.println(enes.destination.y);

  //set up a timer interrupt every millisecond
  OCR0A = 0x01; //millis() counter uses 0
  TIMSK0 |= _BV(OCIE0A);
}

#define ROCKS_X_POS 1
#define GOAL_X_POS 2
#define DRIVE_TO_AVOID_DIST .25

#define START 0
#define DRIVE_OVER_ROCKS 1
#define TURN_DOWNSTREAM 2
#define DRIVE_DOWNSTREAM 3
#define TURN_LEFT 4
#define TURN_RIGHT 5
#define BACK_UP 6
#define DRIVE_TO_AVOID 7
#define TURN_TO_GOAL 8
#define DRIVE_TO_GOAL 9

#define ARMDOWN 100
#define MEASUREPH 101
#define BASE_COLLECTION 102
#define CHECK_PH 103
#define MIX_FORWARD 104
#define MIX_BACKWARD 105

#define STOP 255

uint8_t state = START;

uint32_t stateTimer = 0;
uint8_t stateCounter = 0;

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
  //enesint("signed angular separation: ");
  //enesintln(angleRadians);

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

  //enesint("rotationCorrection: ");
  //enesintln(rotationCorrection);

  motors[0]->setPower(200 * rotationCorrection);
  motors[1]->setPower(-200 * rotationCorrection);

  return done;
}

#define OBJECT_NONE 0
#define OBJECT_LEFT 1
#define OBJECT_RIGHT 2
#define OBJECT_CLOSE 3
#define OBJECT_THRESHOLD_CM 50
#define OBJECT_THRESHOLD_CM_CLOSE 15

int detectObject() {
  digitalWrite(TRIG_PIN_1, LOW);
  //digitalWrite(TRIG_PIN_2, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN_1, HIGH);
  //digitalWrite(TRIG_PIN_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_1, LOW);
 // digitalWrite(TRIG_PIN_2, LOW);

  pinMode(ECHO_PIN_1, INPUT);
  double dist1 = pulseIn(ECHO_PIN_1, HIGH) / 58.2;
  //enes.print(dist1);


  digitalWrite(TRIG_PIN_2, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_2, LOW);
   
  pinMode(ECHO_PIN_2, INPUT);
  double dist2 = pulseIn(ECHO_PIN_2, HIGH) / 58.2;
  //enes.print("  ");
  //enes.println(dist2);
//  enes.print("  ");
  if (dist1 < OBJECT_THRESHOLD_CM || dist2 < OBJECT_THRESHOLD_CM_CLOSE) {
    return OBJECT_CLOSE;
  }
  if (dist1 < OBJECT_THRESHOLD_CM || dist2 < OBJECT_THRESHOLD_CM) {
    if (dist1 < dist2) {
      return OBJECT_LEFT;
    } else {
      return OBJECT_RIGHT;
    }
  } else {
    return OBJECT_NONE;
  }
}

char printQueue[256];
uint8_t writeAddress = 0, readAddress = 0;

boolean doNavigated = false;
boolean doBaseObjective = false;

double startX, startY;

void loop() {
  if (doNavigated) {
    enes.navigated();
    doNavigated = false;
  }
  if (doBaseObjective) {
    enes.baseObjective(phValue);
    doBaseObjective = false;
  }
  
  myUpdateLocation();

  printLocation();

  //write the printQueue to the enes system, then fill it with '*'
  uint8_t i=0;
  do {
    if (printQueue[i] != '*') {
      enes.print(printQueue[i]);
    }
    printQueue[i] = '*';
    i++;
  } while(i != 0);
  enes.println(' ');
}

void printQ(String s) {
  for (int i=0; i<s.length(); i++) {
    printQueue[writeAddress] = s.charAt(i);
    //if(printQueue[writeAddress] == '\n' || printQueue[writeAddress] == ';') commandsReady++;
    writeAddress++;
  }
}

//timer interrupt function (every millisecond)
SIGNAL(TIMER0_COMPA_vect) {
  
  if (doNavigated || doBaseObjective) {
       motors[0]->setPower(0);
       motors[1]->setPower(0);
  }
  
  for (uint8_t i = 0; i < MAX_NUM_MOTORS; i++) {
    motors[i]->update();
  }

  return;
  
  if (doNavigated || doBaseObjective) {
    return;
  }
  
  int objectDetection = detectObject();
  if(state==START)
  {
    if(turn(0))
    {
       motors[0]->setPower(0);
       motors[1]->setPower(0);
       state=DRIVE_OVER_ROCKS; 
    }
  } else
  if (state == DRIVE_OVER_ROCKS) {
    motors[0]->setPower(180);
    motors[1]->setPower(200);
    if (enes.location.x >= ROCKS_X_POS) {
      state = TURN_DOWNSTREAM;
    }
  } else if (state == TURN_DOWNSTREAM) {
    if (turn(0)) {
      motors[0]->setPower(0);
      motors[1]->setPower(0);
      state = DRIVE_DOWNSTREAM;
    } else 
    if (enes.location.y <= .3 && (objectDetection == OBJECT_LEFT || objectDetection == OBJECT_RIGHT)) {
      state = TURN_RIGHT;
    } else
    if (enes.location.y >= 1.7 && (objectDetection == OBJECT_LEFT || objectDetection == OBJECT_RIGHT)) {
      state = TURN_LEFT;
    } else
    if (objectDetection == OBJECT_LEFT) {
      state = TURN_RIGHT;
    } else if (objectDetection == OBJECT_RIGHT) {
      state = TURN_LEFT;
    } else if (objectDetection == OBJECT_CLOSE) {
      startX = enes.location.x;
      startY = enes.location.y;
      state = BACK_UP;
    }
  } else if (state == DRIVE_DOWNSTREAM) {

    motors[0]->setPower(180);
    motors[1]->setPower(200);
    double thetaError = dabs(enes.location.theta);
   
//    if (objectDetection == OBJECT_LEFT ) {
//      if(enes.location.y>=.3)
//      {
//        state = TURN_RIGHT;
//      }
//      else
//      {
//        state=TURN_LEFT;
//      }
//     
//    } else if (objectDetection == OBJECT_RIGHT) {
//      if (enes.location.y <= 1.7){
//        state = TURN_LEFT;
//      }
//      else {
//        state = TURN_RIGHT;
//      }
    
    
    if (enes.location.y <= .3 && (objectDetection == OBJECT_LEFT || objectDetection == OBJECT_RIGHT)) {
      state = TURN_RIGHT;
    } else
    if (enes.location.y >= 1.7 && (objectDetection == OBJECT_LEFT || objectDetection == OBJECT_RIGHT)) {
      state = TURN_LEFT;
    } else
    if (objectDetection == OBJECT_LEFT) {
      state = TURN_RIGHT;
    } else if (objectDetection == OBJECT_RIGHT) {
      state = TURN_LEFT;
    } else if (objectDetection == OBJECT_CLOSE) {
      startX = enes.location.x;
      startY = enes.location.y;
      state = BACK_UP;
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
  } else if (state == BACK_UP) {
    motors[0]->setPower(-180);
    motors[1]->setPower(-200);
    double dX = startX - enes.location.x;
    double dY = startY - enes.location.y;
    double dist = sqrt(dX * dX + dY * dY);
    if (enes.location.y <= .3 && (objectDetection == OBJECT_LEFT || objectDetection == OBJECT_RIGHT)) {
      state = TURN_RIGHT;
    } else
    if (enes.location.y >= 1.7 && (objectDetection == OBJECT_LEFT || objectDetection == OBJECT_RIGHT)) {
      state = TURN_LEFT;
    } else
    if (objectDetection == OBJECT_LEFT) {
      state = TURN_RIGHT;
    } else if (objectDetection == OBJECT_RIGHT) {
      state = TURN_LEFT;
    } else if (dist > DRIVE_TO_AVOID_DIST/2) {
      state = TURN_DOWNSTREAM;
    }
  } else if (state == DRIVE_TO_AVOID) {
    motors[0]->setPower(180);
    motors[1]->setPower(200);
    double dX = startX - enes.location.x;
    double dY = startY - enes.location.y;
    double dist = sqrt(dX * dX + dY * dY);

    if (enes.location.y <= .3 && (objectDetection == OBJECT_LEFT || objectDetection == OBJECT_RIGHT)) {
      state = TURN_RIGHT;
    } else
    if (enes.location.y >= 1.7 && (objectDetection == OBJECT_LEFT || objectDetection == OBJECT_RIGHT)) {
      state = TURN_LEFT;
    } else
    if (objectDetection == OBJECT_LEFT) {
      state = TURN_RIGHT;
    } else if (objectDetection == OBJECT_RIGHT) {
      state = TURN_LEFT;
    } else if (objectDetection == OBJECT_CLOSE) {
      startX = enes.location.x;
      startY = enes.location.y;
      state = BACK_UP;
    } else if (dist > DRIVE_TO_AVOID_DIST) {
      state = TURN_DOWNSTREAM;
    }
  } else if (state == TURN_TO_GOAL) {
    double x = enes.destination.x - enes.location.x;
    double y = enes.destination.y - enes.location.y;
    double desiredTheta = atan2(y, x);
    if (turn(desiredTheta))
    {
      motors[0]->setPower(0);
      motors[1]->setPower(0);
      state = DRIVE_TO_GOAL;
    }
  } else if (state == DRIVE_TO_GOAL) {

    motors[0]->setPower(180);
    motors[1]->setPower(200);
    double x = enes.destination.x - enes.location.x;
    double y = enes.destination.y - enes.location.y;
    double desiredTheta = atan2(y, x);
    double thetaError = dabs(desiredTheta - enes.location.theta);
    double distance = sqrt(x * x + y * y);
    if (thetaError >= PI / 16)
    {
      state = TURN_TO_GOAL;
    } else if (distance <= .55) {
      motors[0]->setPower(0);
      motors[1]->setPower(0);
      stateTimer = millis();
      state = ARMDOWN;
      navigatedTime = millis();
      doNavigated = true; //will trigger enes.navigated() in loop()
//      enes.navigated();
//      servos[2]->set(720); //should lower green arm 90 degrees
      armServo.write(65);
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
      doBaseObjective = true;  //will trigger enes.baseObjective(phValue) in loop()
//      enes.baseObjective(phValue); //transmit the inital pH of the pool
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
        doBaseObjective = true;  //will trigger enes.baseObjective(phValue) in loop()
//        enes.baseObjective(phValue);
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
       motors[0]->setPower(-180);
       motors[1]->setPower(-200);
       state = MIX_BACKWARD;
    }
  }
  else if (state == MIX_BACKWARD){
    if (millis() > stateTimer + 250) {
       motors[0]->setPower(180);
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
     doBaseObjective = true;  //will trigger enes.baseObjective(phValue) in loop()
//     enes.baseObjective(phValue);
     state = STOP;
  }
}
