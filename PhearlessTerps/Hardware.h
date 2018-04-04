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

/*
   ServoControl
   controls the speed of a servo
*/
class ServoControl {
  public:
    Servo servo;
    /*
       Which pin the servo is connected to
    */
    signed pin : 8;
    /*
       The minimum microseconds to send the servo to set it to its minimum position
       0-2048, default = 1000
    */
    unsigned minMicros : 16;
    /*
       The maximum microseconds to send the servo to set it to its maximum position
       0-4096, default = 2000
    */
    unsigned maxMicros : 16;
    /*
       The last command sent to the servo
    */
    unsigned micros : 16;
    /*
       The current position of the servo in 1/8 degree ticks.
       0-1440, default = 360 (180 degrees with 1/8 degree precision)
    */
    unsigned position : 16;
    /*
       The target position where the servo is headed when using the speed control
       0-1440 (180 degrees with 1/8 degree precision)
    */
    unsigned target : 16;

    /*
       stores the last speed set for this servo
       prevents the moveMask from being recreated every time
    */
    unsigned speed : 8;
    /*
       A bitmask where each bit tells whether or not to increment the servo on that millisecond
       Every millisecond, the mask is checked, and if it is a 1 at that position, the servo increments 4 ticks (0.5 degrees)
    */
    uint64_t moveMask : 64;
    /*
       For speed control, tells whether the servo's position has reached its target
       0 = not done, 1 = done
    */
    unsigned isDone : 8;

    ServoControl(uint8_t pin1, uint16_t minMicros1, uint16_t maxMicros1, uint16_t position1);

    ServoControl(uint8_t pin1, uint16_t minMicros1, uint16_t maxMicros1);

    ServoControl(uint8_t pin1, uint16_t position1);

    ServoControl(uint8_t pin1);

    ServoControl();

    /*
       address   0-7     which servo
       position  0-1440  1/8 of a degree
    */
    boolean set(uint16_t position1);

    /*
       address   0-7     which servo
       position  0-1440  1/8 of a degree
       speed     1-64    ticks per 128 milliseconds

       a value of 0 for the speed will result in no command sent
       a value greater than 64 will call the other setServo function (max speed)
    */
    boolean set(uint16_t position1, uint8_t speed1);

    uint16_t calculateMicros();

    void update(uint64_t servoMask);

  private:
    void init();
};

