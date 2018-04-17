//#include "Arduino.h"
#include "Util.h"

double dabs1(double val) {
  if (val > 0) return val;
  if (val < 0) return -val;
  return 0;
}


Angle::Angle(double radians1) {
  radians = radians1;
}
Angle Angle::absAngle() {
  return Angle(dabs1(radians));
}

double Angle::signum() {
  return sgn(radians);
}


/**
   Adds two Angles together

   @param angle1 one Angle
   @param angle2 another Angle
   @return the resulting Angle
*/
static Angle Angle::add(Angle angle1, Angle angle2) {
  return Angle(angle1.radians + angle2.radians);
}

/**
   Subtracts angle2 from angle1

   @param angle1 one Angle
   @param angle2 another Angle
   @return the resulting Angle
*/
static Angle Angle::subtract(Angle angle1, Angle angle2) {
  return Angle(angle1.radians - angle2.radians);
}

/**
   Multiplies an Angle by a number

   @param angle the Angle to be multiplied
   @param number the number to multiply by
   @return the resulting Angle
*/
static Angle Angle::multiply(Angle angle, double number) {
  return Angle(angle.radians * number);
}

/**
   Divides an Angle by a number

   @param angle the Angle to be multiplied
   @param number the number to divided by
   @return the resulting Angle
*/
static Angle Angle::divide(Angle angle, double number) {
  return Angle(angle.radians / number);
}

/**
   Creates an Angle class from a value in radians

   @param v the value in radians
   @return the Angle class
*/
static Angle Angle::fromRadians(double v) {
  return Angle(v);
}

/**
   Creates an Angle class from a value in degrees

   @param v the value in degrees
   @return the Angle class
*/
static Angle Angle::fromDegrees(double v) {
  return Angle(v * PI / 180);
}

/**
   Creates an Angle class from a value in rotations

   @param v the value in rotations
   @return the Angle class
*/
static Angle Angle::fromRotations(double v) {
  return Angle(v * 2 * PI);
}

/**

   @return the value of the Angle in radians
*/
double Angle::getRadians() {
  return radians;
}

/**

   @return the value of the Angle in degrees
*/
double Angle::getDegrees() {
  return radians * 180 / PI;
}

/**

   @return the value of the Angle in rotations
*/
double Angle::getRotations() {
  return radians * 1 / (2 * PI);
}

Vector2D::Vector2D(double magnitude, Angle theta1) {
  double thetaRads = theta1.getRadians();
  x = magnitude * cos(thetaRads);
  y = magnitude * sin(thetaRads);

  l = magnitude;
  theta = &theta1;
}

Vector2D::Vector2D(double x1, double y1) {
  x = x1;
  y = y1;

  //Pythagorean theorem
  l = sqrt(x * x + y * y);
  theta = &Angle::fromRadians(atan2(y, x));
}

static Angle Vector2D::signedAngularSeparation(Vector2D ref, Vector2D vector) {

  //cross product of two 2D vectors in 3D space
  double z = ref.getX() * vector.getY() - ref.getY() * vector.getX();

  // If the vectors are too closely aligned, then return zero for
  // separation.
  if (dabs1(z) < closeToZero) return Angle(0);

  // To get the angle:
  // a dot b = a * b * cos(angle)
  // so angle = acos[ (a dot b) / (a * b) ]
  // Make sure a * b is not too close to zero 0
  double lengths = ref.getLength() * vector.getLength();
  if (lengths < closeToZero) {
    // this is really an error, but to keep the OSV from crashing,
    // just return 0
    return Angle(0);
  }
  double dot = ref.getX() * vector.getX() + ref.getY() * vector.getY();

  return Angle::fromRadians(sgn(z) * acos(dot / lengths));
}

static Vector3D Vector3D::from2D(Vector2D vector2D) {
  return Vector3D(vector2D.getX(), vector2D.getY(), 0);
}
