#include "Adafruit_VL53L0X.h"
#include <Servo.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

Servo armServo;
Servo lidarServo;
Servo collectServo;

uint32_t loopTimer = 0;

void setup() {
  
  armServo.attach(13);
  lidarServo.attach(11);
  collectServo.attach(10);
  
  Serial.begin(9600);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  // power
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
}

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

void loop() {

  //make sure the loop runs no faster than once every 1 millisecond
  int16_t difference = millis() - loopTimer;
  if (difference >= 100) {
    loopTimer += difference;
  } else {
    loopTimer += 1;
    while (millis() < loopTimer);
  }
  
//  VL53L0X_RangingMeasurementData_t measure;
//  
//  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
//
//  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
//   if (measure.RangeMilliMeter < 300){
//      Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
//   }
//  }

  Serial.println(lidarScan());

//  delay(10);
}

