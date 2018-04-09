#include <Enes100.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include <Servo.h>
#include "Hardware.h"
#include "Util.h"

/* Create a new Enes100 object
   Parameters:
    string teamName
    int teamType
    int markerId
    int rxPin
    int txPin
*/
Enes100 enes("pHearless Terps", CHEMICAL, 3, 8, 9);

double dabs(double val) {
  if (val > 0) return val;
  if (val < 0) return -val;
  return 0;
}

#define MAX_NUM_MOTORS 8
#define MAX_NUM_SERVOS 8

Motor* motors[MAX_NUM_MOTORS] = {
  // PORT           IN1|PWM|EN| FB | SF |BRAKE
  /*0*/ new Motor(  2,  3, 4, NONE, NONE, true), //left motor
  /*1*/ new Motor(  7,  5, 12, NONE, NONE, true), //right motor
  /*2*/ new Motor(),
  /*3*/ new Motor(),
  /*4*/ new Motor(),
  /*5*/ new Motor(),
  /*6*/ new Motor(),
  /*7*/ new Motor()
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
  /*3*/ new ServoControl(11                   ), //lidar servo
  /*4*/ new ServoControl(                     ),
  /*5*/ new ServoControl(                     ),
  /*6*/ new ServoControl(                     ),
  /*7*/ new ServoControl(                     )
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

//  // Retrieve the destination
//  while (!enes.retrieveDestination()) {
//    enes.println("Unable to retrieve location");
//  }
//
//  enes.print("My destination is at ");
//  enes.print(enes.destination.x);
//  enes.print(",");
//  enes.println(enes.destination.y);

//  Serial.begin(9600);
//  
//  double targetHeading = PI / 2.0;
//
//  double TURN_GAIN = 0.2;
//  double TURN_DEADZONE = 0.01;
//  double TURN_MIN_SPEED = 0.05;
//  double TURN_MAX_SPEED = 1;
//  Vector2D targetHeadingVector = Vector2D(1, Angle::fromRadians(targetHeading));
//  Vector2D directionVector = Vector2D(1, Angle::fromRadians(enes.location.theta));
//
//  //find the "signed angular separation", the magnitude and direction of the error
//  double angleRadians = Vector2D::signedAngularSeparation(targetHeadingVector, directionVector).getRadians();
//  Serial.print("signed angular separation: ");
//  Serial.println(angleRadians);
//
//  //              This graph shows angle error vs. rotation correction
//  //              ____________________________
//  //              | correction.       ____   |
//  //              |           .      /       |
//  //              |           .   __/        |
//  //              | ........__.__|.......... |
//  //              |      __|  .     error    |
//  //              |     /     .              |
//  //              | ___/      .              |
//  //              |__________________________|
//  //
//  //              The following code creates this graph:
//
//
//  Serial.print("angleRadians: ");
//  Serial.println(angleRadians);
//
//  //scale the signedAngularSeparation by a constant
//  double rotationCorrection = TURN_GAIN * angleRadians;
//  //                rotationCorrection = GYRO_PID.computeCorrection(0, angleRadians);
//
//  Serial.print("rotationCorrection before: ");
//  Serial.println(rotationCorrection);
//  Serial.println(TURN_DEADZONE);
//  
//  if (dabs(rotationCorrection) > TURN_MAX_SPEED) {
//    //cap the rotationCorrection at +/- TURN_MAX_SPEED
//    rotationCorrection = sgn(rotationCorrection) * TURN_MAX_SPEED;
//  } else if (dabs(rotationCorrection) < TURN_DEADZONE) {
//    //set it to 0 if it is in the deadzone
//    rotationCorrection = 0;
//  } else if (dabs(rotationCorrection) < TURN_MIN_SPEED) {
//    //set it to the minimum if it is below
//    rotationCorrection = sgn(rotationCorrection) * TURN_MIN_SPEED;
//  }
//
//  Serial.print("rotationCorrection after: ");
//  Serial.println(rotationCorrection);
//
//  motors[0]->setPower(255 * rotationCorrection);
//  motors[1]->setPower(-255 * rotationCorrection);
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

int task = 0;

enum State {
//  START,
//  MOTOR0F, MOTOR1F, MOTOR0B, MOTOR1B,
  TURN,
  STOP
};

enum State state = START;

uint32_t stateTimer = 0;
int stateLength = 1000;

void loop() {
  updateDevices(loopTimer);

  //every 16 millis (on a different count than the motors) update OSV location
  if (loopTimer % 16 == 8) enes.updateLocation();

  if (state == TURN) {
    double targetHeading = PI / 2.0;

    double TURN_GAIN = 0.2;
    double TURN_DEADZONE = 0.01;
    double TURN_MIN_SPEED = 0.05;
    double TURN_MAX_SPEED = 1;
    Vector2D targetHeadingVector = Vector2D(1, targetHeading);
    Vector2D directionVector = Vector2D(1, Angle::fromRadians(enes.location.theta));

    //find the "signed angular separation", the magnitude and direction of the error
    double angleRadians = Vector2D::signedAngularSeparation(targetHeadingVector, directionVector).getRadians();
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
    } else if (dabs(rotationCorrection) < TURN_MIN_SPEED) {
      //set it to the minimum if it is below
      rotationCorrection = sgn(rotationCorrection) * TURN_MIN_SPEED;
    }

    enes.print("rotationCorrection: ");
    enes.println(rotationCorrection);

    motors[0]->setPower(255 * rotationCorrection);
    motors[1]->setPower(-255 * rotationCorrection);

  } else if (state == STOP) {
    motors[0]->setPower(0);
    motors[1]->setPower(0);
  }
//  if (state == START) {
//    stateTimer = millis();
//    state = MOTOR0F;
//    motors[0]->setPower(255);
//    motors[1]->setPower(0);
//  } else if (state == MOTOR0F) {
//    if (millis() > stateTimer + stateLength) {
//      state = MOTOR1F;
//      stateTimer += stateLength;
//      motors[0]->setPower(0);
//      motors[1]->setPower(255);
//    }
//  } else if (state == MOTOR1F) {
//    if (millis() > stateTimer + stateLength) {
//      state = MOTOR0B;
//      stateTimer += stateLength;
//      motors[0]->setPower(-255);
//      motors[1]->setPower(0);
//    }
//  } else if (state == MOTOR0B) {
//    if (millis() > stateTimer + stateLength) {
//      state = MOTOR1B;
//      stateTimer += stateLength;
//      motors[0]->setPower(0);
//      motors[1]->setPower(-255);
//    }
//  } else if (state == MOTOR1B) {
//    if (millis() > stateTimer + stateLength) {
//      state = STOP;
//      stateTimer += stateLength;
//    }
//  } else if (state == STOP) {
//    motors[0]->setPower(0);
//    motors[1]->setPower(0);
//  }
//
//  return;

  //  } else if (state == BROADCAST) {
  //    enes.navigated();
  //    // Transmit the initial pH of the pool
  //    enes.baseObjective(2.7);
  //    // Transmit the final pH of the pool
  //    enes.baseObjective(7.0);
  //  }

  //  // Update the OSV's current location
  //  if (enes.updateLocation()) {
  //    enes.println("Huzzah! Location updated!");
  //    enes.print("My x coordinate is ");
  //    enes.println(enes.location.x);
  //    enes.print("My y coordinate is ");
  //    enes.println(enes.location.y);
  //    enes.print("My theta is ");
  //    enes.println(enes.location.theta);
  //  } else {
  //    enes.println("Sad trombone... I couldn't update my location");
  //  }

  //
  //  //A. forward locomotive test
  //  if (task == 0) {
  //    while (enes.location.x <= 4) {
  //      motors[0]->setPower(200);
  //      motors[1]->setPower(200);
  //      if (enes.location.x == 4) {
  //        motors[0]->setPower(0);
  //        motors[1]->setPower(0);
  //        delay(1000);
  //        task += 1;
  //      }
  //      enes.updateLocation();
  //    }
  //  }
  //
  //
  //  //B. Turning Test (orientation has to be replaced with theta which is radians -pi to pi)
  //  if (task == 2) {
  //    while (enes.location.theta <= PI / 2) {
  //      motors[0]->setPower(-200);
  //      motors[1]->setPower(200);
  //      if (enes.location.theta == PI / 2) {
  //        motors[0]->setPower(0);
  //        motors[1]->setPower(0);
  //        delay(1000);
  //      }
  //      enes.updateLocation();
  //    }
  //    while (enes.location.theta >= 0) {
  //      motors[0]->setPower(200);
  //      motors[1]->setPower(-200);
  //      if (enes.location.theta == 0) {
  //         motors[0]->setPower(0);
  //         motors[1]->setPower(0);
  //        delay(1000);
  //      }
  //      enes.updateLocation();
  //    }
  //    while (enes.location.theta >= -PI / 2) {
  //      motors[0]->setPower(200);
  //      motors[1]->setPower(-200);
  //      if (enes.location.theta == -PI / 2) {
  //        motors[0]->setPower(0);
  //        motors[1]->setPower(0);
  //        delay(1000);
  //        task += 1;
  //      }
  //      enes.updateLocation();
  //    }
  //  }
  //
  //
  //  //C. RF Communications
  //  if (task == 3) {
  //    if (enes.retrieveDestination() == true) {
  //      enes.print("Receiving destination is at the time of part C. is: ");
  //      enes.print(enes.destination.x);
  //      enes.print(",");
  //      enes.println(enes.destination.y);
  //    }
  //    task += 1;
  //  }
  //
  //  //D.
  //  if (task == 4) {
  //    if (enes.location.theta != 0) {
  //      if (enes.location.theta < 0) {
  //        while (enes.location.theta != 0) {
  //          motors[0]->setPower(-200);
  //          motors[1]->setPower(200);
  //          enes.updateLocation();
  //        }
  //      }
  //      else if (enes.location.theta > 0) {
  //        while (enes.location.theta != 0) {
  //          motors[0]->setPower(200);
  //          motors[1]->setPower(-200);
  //          enes.updateLocation();
  //        }
  //      }
  //    }
  //    if (enes.location.y != enes.destination.y) {
  //      if (enes.location.y > enes.destination.y) {
  //        while (enes.location.theta > -PI / 2) {
  //          motors[0]->setPower(200);
  //          motors[1]->setPower(-200);
  //          enes.updateLocation();
  //        }
  //      }
  //      else if (enes.location.y < enes.destination.y) {
  //        while (enes.location.theta < PI / 2) {
  //          motors[0]->setPower(-200);
  //          motors[1]->setPower(200);
  //          enes.updateLocation();
  //        }
  //      }
  //      while (enes.location.y != enes.destination.y) {
  //        motors[0]->setPower(200);
  //        motors[1]->setPower(200);
  //        if (enes.location.theta == PI / 2) {
  //          while (enes.location.theta != 0) {
  //            motors[0]->setPower(200);
  //            motors[1]->setPower(-200);
  //            enes.updateLocation();
  //          }
  //        }
  //        else if (enes.location.theta == -PI / 2) {
  //          while (enes.location.theta != 0) {
  //            motors[0]->setPower(-200);
  //            motors[1]->setPower(200);
  //            enes.updateLocation();
  //          }
  //        }
  //      }
  //    }
  //    while (enes.location.x < enes.destination.x) {
  //      motors[0]->setPower(200);
  //      motors[1]->setPower(200);
  //    }
  //    task += 1;
  //  }
}
