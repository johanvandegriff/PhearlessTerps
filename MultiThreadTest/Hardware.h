#include "Arduino.h"
#include <Servo.h>

#define NONE -1

#define SERVO_DEFAULT_MIN_MICROS 1000
#define SERVO_DEFAULT_MAX_MICROS 2000
#define SERVO_DEFAULT_POSITION 720
#define SERVO_DEFAULT_MICROS 1500
#define SERVO_INC_PER_MILLI 4
#define SERVO_TICKS_PER_RANGE 1440

/*
   Motor
   contains the motor's pins, state, and feedback
*/
class Motor {
  public:
    /*
       pin connected to IN1 on the motor controller
       controls the direction of the motor
    */
    signed directionPin : 8;
    /*
       pin connected to IN2 on the motor controller
       controls the power sent to the motor
    */
    signed pwmPin : 8;
    /*
       pin connected to EN on the motor controller
       turns the motor on and off
    */
    signed enablePin : 8;
    /*
       pin connected to FB on the motor controller
       reads the current the motor is drawing
    */
    signed analogPin : 8;
    /*
       pin connected to SF on the motor controller
       reads if the motor controller has shut off due to a high temperature or current
    */
    signed statusPin : 8;
    /*
       the motor's power, including direction
       ranges from -255 to 255 (-256 is illegal)
    */
    signed power : 16;
    /*
       true to brake when stopping
       false to coast when stopping
    */
    unsigned brake : 8;
    /*
       the current being drawn by the motor
       a negative value means there is no sensor or it has not been read yet
    */
    signed currentDraw : 16;
    /*
       the status flag of the motor controller
    */
    unsigned isOK : 8;

    Motor(int8_t directionPin1, int8_t pwmPin1, int8_t enablePin1, int8_t analogPin1, int8_t statusPin1, boolean brake1);
    Motor();

    boolean setPower(int16_t power1);

    void update();

  private:
    void init();
};

