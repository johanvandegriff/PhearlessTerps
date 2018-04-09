#include "Arduino.h"

//#define PI 3.1415926535897932384626433832795;

static inline double sgn(double val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

class Angle {
    /*
       Constants to relate various units to radians
       The reciprocal of each constant is calculated once to be used multiple
       times later
    */
  private:
    //    static const double RAD_PER_DEG = PI / 180;
    //    static const double RAD_PER_ROT = 2 * PI;
    //
    //    static const double DEG_PER_RAD = 1 / RAD_PER_DEG;
    //    static const double ROT_PER_RAD = 1 / RAD_PER_ROT;

    // holds the angle's value in radians
    double radians;

    //private constructor to make an Angle object from a value in radians

  public:
    Angle(double radians);
    //return the absolute value of the angle
    Angle absAngle();

    //return the sign of the angle
    double signum();

    /**
       Adds two Angles together

       @param angle1 one Angle
       @param angle2 another Angle
       @return the resulting Angle
    */
    static Angle add(Angle angle1, Angle angle2);

    /**
       Subtracts angle2 from angle1

       @param angle1 one Angle
       @param angle2 another Angle
       @return the resulting Angle
    */
    static Angle subtract(Angle angle1, Angle angle2);

    /**
       Multiplies an Angle by a number

       @param angle the Angle to be multiplied
       @param number the number to multiply by
       @return the resulting Angle
    */
    static Angle multiply(Angle angle, double number);

    /**
       Divides an Angle by a number

       @param angle the Angle to be multiplied
       @param number the number to divided by
       @return the resulting Angle
    */
    static Angle divide(Angle angle, double number);

    /**
       Creates an Angle class from a value in radians

       @param v the value in radians
       @return the Angle class
    */
    static Angle fromRadians(double v);

    /**
       Creates an Angle class from a value in degrees

       @param v the value in degrees
       @return the Angle class
    */
    static Angle fromDegrees(double v);

    /**
       Creates an Angle class from a value in rotations

       @param v the value in rotations
       @return the Angle class
    */
    static Angle fromRotations(double v);

    /**

       @return the value of the Angle in radians
    */
    double getRadians();

    /**

       @return the value of the Angle in degrees
    */
    double getDegrees();

    /**

       @return the value of the Angle in rotations
    */
    double getRotations();
};






class Vector2D {
  private:
    /*
       the 2 components of the vector
    */
    double x;
    double y;

    /*
       the polar coordinates
    */
    double l;
    Angle* theta;

    const static double closeToZero = 1.0e-3;
  public:
    /**
       create a vector using polar coordinates

       @param magnitude the magnitude of the 2-D vector
       @param theta the direction of the 2-D vector
       @return the created vector
    */
    Vector2D(double magnitude, Angle theta);

    /**
       create a vector using x and y

       @param x x component
       @param y y component
    */
    Vector2D(double x1, double y1);

    /**
       @return the x component of the vector
    */
    double getX() {
      return x;
    }

    /**
       @return the y component of the vector
    */
    double getY() {
      return y;
    }

    /**
       @return the length or magnitude of the vector
    */
    double getLength() {
      return l;
    }

    /**
       @return a new vector that is normalized (length = 1)
    */
    Vector2D normalized() {
      return Vector2D(x / l, y / l);
    }

    /**
       The order does not matter for the dot product

       @param v1 one vector
       @param v2 another vector
       @return the dot product of the two vectors
    */
    static double dotProduct(Vector2D v1, Vector2D v2) {
      return v1.x * v2.x + v1.y * v2.y;
    }

    /**
       @return the direction of the vector
    */
    Angle* getDirection() {
      return theta;
    }

    /**
       This helps when using a gyro sensor or any type of 360 degree rotation
       mechanism

       @param ref the reference direction, assumed to have no z component
       @param vector the vector to be measured against the reference to find the
                  angular separation, also assumed to have no z component
       @return angular separation between -pi and pi. If vector is to the right
               of reference, then it is positive.
    */
    static Angle signedAngularSeparation(Vector2D ref, Vector2D vector);
};




class Vector3D {
  private:
    /*
       the 3 components of the vector
    */
    double x;
    double y;
    double z;

    /*
       the spherical coordinates
    */
    double l;
    Angle* theta;
    Angle* phi;

  public:
    /**
       create a vector from a Vector2D with z = 0

       @param vector2D the 2D vector to use
       @return the created Vector3D
    */
    static Vector3D from2D(Vector2D vector2D);


    /**
       create a vector using polar coordinates with z = 0

       @param magnitude the magnitude of the Vector2D
       @param theta the direction of the Vector2D
       @return the created Vector3D
    */
    static Vector3D fromPolar2D(double magnitude, Angle theta) {
      return from2D(Vector2D(magnitude, theta));
    }
    /**
       Create a vector using spherical coordinates

       @param magnitude the magnitude of the 3D vector
       @param theta the direction in the x-y plane
       @param phi the z direction
       @return
    */
    Vector3D(double magnitude, Angle theta1, Angle phi1) {
      double thetaRads = theta1.getRadians();
      double phiRads = phi1.getRadians();

      // http://mathinsight.org/spherical_coordinates
      // x = ρ sinϕ cosθ
      // y = ρ sinϕ sinθ
      // z = ρ cosϕ

      x = magnitude * sin(phiRads) * cos(thetaRads);
      y = magnitude * sin(phiRads) * sin(thetaRads);
      z = magnitude * cos(phiRads);

      l = magnitude;
      theta = &theta1;
      phi = &phi1;
    }

    /**
       create a vector using x, y, and z

       @param x x component
       @param y y component
       @param z z component
    */
    Vector3D(double x, double y, double z) {
      x = x;
      y = y;
      z = z;

      //Pythagorean theorem
      l = sqrt(x * x + y * y + z * z);

      //compute spherical coordinates
      phi = &Angle::fromRadians(acos(z / l));
      theta = &Angle::fromRadians(atan2(y, x));
    }


    /**
       @return the x component of the vector
    */
    double getX() {
      return x;
    }

    /**
       @return the y component of the vector
    */
    double getY() {
      return y;
    }

    /**
       @return the z component of the vector
    */
    double getZ() {
      return z;
    }

    void setX(double x1) {x=x1;}
    void setY(double y1) {y=y1;}
    void setZ(double z1) {z=z1;}

    /**
       @return the length or magnitude of the vector
    */
    double getLength() {
      return l;
    }

    /**
       @return the z direction
    */
    Angle* getPhi() {
      return phi;
    }

    /**
       @return the x-y direction
    */
    Angle* getTheta() {
      return theta;
    }

    /**
       @return a new vector that is normalized (length = 1)
    */
    Vector3D normalized() {
      return Vector3D(x / l, y / l, z / l);
    }

    /**
       Order matters for the cross product

       @param v1 the first vector
       @param v2 the second vector
       @return the first vector crossed with the second vector
    */
    static Vector3D crossProduct(Vector3D v1, Vector3D v2) {
      double x = v1.y * v2.z - v1.z * v2.y;
      double y = v1.z * v2.x - v1.x * v2.z;
      double z = v1.x * v2.y - v1.y * v2.x;
      return Vector3D(x, y, z);
    }

    /**
       The order does not matter for the dot product

       @param v1 one vector
       @param v2 another vector
       @return the dot product of the two vectors
    */
    static double dotProduct(Vector3D v1, Vector3D v2) {
      return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }

    /**

       @param v1 one vector
       @param v2 another vector
       @return the angle between the two vectors
    */
    static Angle angularSeparation(Vector3D v1, Vector3D v2) {
      //             a dot b
      //cos theta = ---------
      //            |a| * |b|
      return Angle::fromRadians(acos(dotProduct(v1, v2) / (v1.l * v2.l)));
    }
};
