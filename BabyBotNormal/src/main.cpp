/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\hererjd23                                        */
/*    Created:      Wed Sep 25 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <iostream>
#include <cmath>
#define PI 3.14159265

using namespace vex;
// A global instance of vex::brain used for printing to the V5 brain screen
brain Brain;
// A global instance of vex::competition
competition Competition;

// define your global instances of motors and other devices here

controller controller1;

motor frontRight(PORT2, true);
motor frontLeft(PORT4);
motor rearRight(PORT1, true);
motor rearLeft(PORT3);
motor armMotor(PORT5);
gyro gyro1(Brain.ThreeWirePort.G);


// USE INCHES!!!

const float WHEEL_CIRCUMFERENCE = 3.25 * PI;

const float TRACK_WIDTH = 7.5; // Width between right and left wheels
const float WHEEL_BASE = 4.5; // Length between front and back
const float DIAGONAL = sqrt(pow(TRACK_WIDTH, 2) + pow(WHEEL_BASE, 2)); // Calculate diagonal distance between nonadjacent wheels using pythagorean theorum
const float CIRCUMFERENCE = DIAGONAL*PI; // Calculate circumference of circle in which the wheel.

const static float ARM_MAX = 30;
const static float ARM_MIN = 0;


float inchesToTicks(float inches) {
  float ticks = (inches / WHEEL_CIRCUMFERENCE) * 360;
  return ticks;

}

/*
float angleWrap(float degrees) {
  // Keeps angle inside range -180 to 180 while preserving angle measure
  while (degrees >= 180) degrees -= 360;
  while (degrees < -180) degrees += 360;
  Brain.Screen.printAt(0, 200,"%f", degrees);
  return degrees;

}
*/
float angleWrap(float degrees) {
  // Keeps angle within range 0 to 360 while preserving angle measure
  while (degrees >= 360) degrees -=360;
  while (degrees < 0) degrees -= 360;
  return degrees;

}


float restrictToRange(float number, float bottom, float top) {
  Brain.Screen.printAt(200, 175, "%.3f", number);
  if (number < bottom) number = bottom;
  if (number > top) number = top;
  return number;
}




void setMotorSides(float leftSpeed, float rightSpeed) {
  frontRight.spin(fwd, rightSpeed, velocityUnits::pct);
  frontLeft.spin(fwd, leftSpeed, velocityUnits::pct);
  rearRight.spin(fwd, rightSpeed, velocityUnits::pct);
  rearLeft.spin(fwd, leftSpeed, velocityUnits::pct);

}


void stopBase() {
  frontRight.stop(brakeType::coast);
  frontLeft.stop(brakeType::coast);
  rearRight.stop(brakeType::coast);
  rearLeft.stop(brakeType::coast);

}


void coastBase() {
  frontRight.stop(brakeType::coast);
  frontLeft.stop(brakeType::coast);
  rearRight.stop(brakeType::coast);
  rearLeft.stop(brakeType::coast);

}


void clearEncoders(){
  frontRight.resetRotation();
  frontLeft.resetRotation();
  rearRight.resetRotation();
  rearLeft.resetRotation();


}


void forwardForInches(float inches, int speed, bool wait = true) {
  float ticks = inchesToTicks(inches);

  frontRight.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
  frontLeft.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
  rearRight.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
  rearLeft.rotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
  // Start motor and move on if wait == false, else wait for target to be reached and stop motors
  if (!wait) {
    rearLeft.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);

  } else {
    rearLeft.rotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
    stopBase();
  }
  task::sleep(15);


}


void forwardGradual(float inches, int maxSpeed, int accelTime) {
  float ticks = inchesToTicks(inches);
  float increment = 1;
  float speed = increment;

  clearEncoders();
   //Gradually accelerate for accelTime and reach desired speed.
  for (float i = 0; i <= maxSpeed; i += increment) {
    speed = round(i);
    frontRight.setVelocity(speed, velocityUnits::pct);
    frontLeft.setVelocity(speed, velocityUnits::pct);
    rearRight.setVelocity(speed, velocityUnits::pct);
    rearLeft.setVelocity(speed, velocityUnits::pct);

    frontRight.spin(fwd, speed, velocityUnits::pct);
    frontLeft.spin(fwd, speed, velocityUnits::pct);
    rearRight.spin(fwd, speed, velocityUnits::pct);
    rearLeft.spin(fwd, speed, velocityUnits::pct);
    task::sleep(increment/accelTime);
    controller1.Screen.print(speed);
  }

  float deaccelPoint = ticks - (increment*maxSpeed);

  while ((frontRight.rotation(deg)+frontLeft.rotation(deg)+rearRight.rotation(deg)+rearLeft.rotation(deg))/4 < deaccelPoint) {
    // Wait until the moment when motor rotation is the correct degrees away from desired wheel rotation.
    controller1.Screen.print("%s", frontRight.rotation(deg));//"%f %f", deaccelPoint, ticks);
  }

  for (float i = maxSpeed; i > 0; i -= increment) {
    speed = round(i);
    frontRight.setVelocity(speed, velocityUnits::pct);
    frontLeft.setVelocity(speed, velocityUnits::pct);
    rearRight.setVelocity(speed, velocityUnits::pct);
    rearLeft.setVelocity(speed, velocityUnits::pct);
    
    frontRight.spin(fwd, speed, velocityUnits::pct);
    frontLeft.spin(fwd, speed, velocityUnits::pct);
    rearRight.spin(fwd, speed, velocityUnits::pct);
    rearLeft.spin(fwd, speed, velocityUnits::pct);
    task::sleep(increment/accelTime);
    controller1.Screen.print("%s %d", "deaccelerating", speed);
  }

  controller1.Screen.clearScreen();
  controller1.Screen.print("%s", "done");
  stopBase();

  task::sleep(15);

}


void turnLeftForDegrees(float degrees, int speed, bool wait = true) {
  float ticks = inchesToTicks(CIRCUMFERENCE) * (degrees/360);

  frontRight.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
  frontLeft.startRotateFor(directionType::rev, ticks, rotationUnits::deg, speed, velocityUnits::pct);
  rearRight.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
  rearLeft.rotateFor(directionType::rev, ticks, rotationUnits::deg, speed, velocityUnits::pct);
  if (!wait) {
    rearLeft.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);

  } else {
    rearLeft.rotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
    stopBase();
  }
  task::sleep(15);
}


void turnRightForDegrees(float degrees, int speed, bool wait = true) {
  float ticks = inchesToTicks(CIRCUMFERENCE) * (degrees/360);

  frontRight.startRotateFor(directionType::rev, ticks, rotationUnits::deg, speed, velocityUnits::pct);
  frontLeft.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
  rearRight.startRotateFor(directionType::rev, ticks, rotationUnits::deg, speed, velocityUnits::pct);
  if (!wait) {
    rearLeft.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);

  } else {
    rearLeft.rotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
    stopBase();
  }
  task::sleep(15);

}

/*
void turnToDegrees(float degrees, int speed, bool wait = true) {
  // Calculates fastest direction to angle and number of ticks to turn to angle
  float turnDegrees = degrees - angleWrap(gyro1.value(rotationUnits::deg));
  float ticks = inchesToTicks(CIRCUMFERENCE) * (turnDegrees/360);
  clearEncoders();
  // Positive angle turns right by default because positive angles are to the right (in this program).
  while (fabs(frontRight.rotation(rotationUnits::deg)+frontLeft.rotation(rotationUnits::deg)+rearRight.rotation(rotationUnits::deg)+rearLeft.rotation(rotationUnits::deg))/4 < ticks) {
    frontRight.spin(directionType::rev, speed, velocityUnits::pct);
    frontLeft.spin(fwd, speed, velocityUnits::pct);
    rearRight.spin(directionType::rev, speed, velocityUnits::pct);
    rearLeft.spin(fwd, speed, velocityUnits::pct);

    Brain.Screen.printAt(140,95, "Gyro: %6.2f", gyro1.value(rotationUnits::deg));
  }
  stopBase();
  task::sleep(15);

}

*/


void turnToDegreesConstant(float degrees, int speed, bool wait = true) {
  // Calculates fastest direction to angle and number of ticks to turn to angle
  float turnDegrees = degrees - angleWrap(gyro1.value(rotationUnits::deg));
  float ticks = inchesToTicks(CIRCUMFERENCE) * (turnDegrees/390);

  // Right is forward, left is reverse because positive angles are to the right.
  frontRight.startRotateFor(directionType::rev, ticks, rotationUnits::deg, speed, velocityUnits::pct);
  frontLeft.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
  rearRight.startRotateFor(directionType::rev, ticks, rotationUnits::deg, speed, velocityUnits::pct);
  if (!wait) {
    rearLeft.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);

  } else {
    rearLeft.rotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
    stopBase();
  }
  task::sleep(15);

}


/* boi.
void turnRightToDegreesGradual(float degrees, int maxSpeed) {
  // Calculates fastest direction to angle and number of ticks to turn to angle
  
  float turnDegrees = degrees - angleWrap(gyro1.value(rotationUnits::deg));
  float ticks = inchesToTicks(CIRCUMFERENCE) * (turnDegrees/380); // total revolution degrees adjusted from 360 because it increases accuracy for unknown reasons (probably build reasons)

  clearEncoders();

  maxSpeed = restrictToRange(maxSpeed, 0, 100);
  float peak = ticks/2;
  bool peaked = false;
  float encoderAvg = 1;
  float trueSpeed;
  //while (encoderAvg < fabs(ticks)) {
    Brain.Screen.printAt(1, 30, "%f.3", gyro1.value(rotationUnits::deg) - degrees);
    while (fabs(gyro1.value(rotationUnits::deg) - degrees) > 0.5) {
      encoderAvg = ((fabs(frontRight.rotation(rotationUnits::deg))+fabs(frontLeft.rotation(rotationUnits::deg))+fabs(rearRight.rotation(rotationUnits::deg))+fabs(rearLeft.rotation(rotationUnits::deg)))/4)+1;
      
      float accel = (2*maxSpeed)/ticks;
      if (encoderAvg > ticks/2 && !peaked) {
          Brain.Screen.printAt(250, 200, "Peak %: %f.3", trueSpeed);
          peaked = true;
      }

      // Speed calculated using absolute value function (slower in beginning and end and faster in middle)
      trueSpeed = -(accel*fabs((encoderAvg-peak))) + maxSpeed;
      if (!peaked) { // Make it accelerate faster in the first half, still being capped at maxSpeed
        trueSpeed *= 4;
        trueSpeed = restrictToRange(trueSpeed, 1, maxSpeed); // Minimal speed is one when it starts so it doesn't stay at zero and do nothing
      } else trueSpeed = restrictToRange(trueSpeed, 0, maxSpeed);
      
      // Right is reversed, left is forward because positive angles are to the right and reverse turns in that direction.
      frontRight.spin(directionType::rev, trueSpeed, velocityUnits::pct);
      frontLeft.spin(fwd, trueSpeed, velocityUnits::pct);
      rearRight.spin(directionType::rev, trueSpeed, velocityUnits::pct);
      rearLeft.spin(fwd, trueSpeed, velocityUnits::pct);
      Brain.Screen.printAt(1, 120, "Traveled: %.3f Total: %.3f", encoderAvg, ticks);
      Brain.Screen.printAt(1, 140, "True speed: %.3f, Acceleration: %.3f", trueSpeed, accel);
      Brain.Screen.printAt(1, 160, "Peak ticks: %.3f, Max speed: %.3f", peak, maxSpeed);
      
      if ((peaked && trueSpeed < 1) || (fabs(gyro1.value(rotationUnits::deg) - degrees) < 1)) { // Makes sure it is completely over if robot stops
        if (fabs(gyro1.value(rotationUnits::deg)) > fabs(degrees)) { // Makeshift PID stops robot from oversteering
          float error = fabs(gyro1.value(rotationUnits::deg) - degrees);
          // Left is negative, since to counteract right it needs to turn left
          frontRight.startRotateFor(error, rotationUnits::deg, 5, velocityUnits::pct);
          frontLeft.startRotateFor(error , rotationUnits::deg, -5, velocityUnits::pct);
          rearRight.startRotateFor(error, rotationUnits::deg, 5, velocityUnits::pct);
          rearLeft.rotateFor(error, rotationUnits::deg, -5, velocityUnits::pct);
        }
        break;
      
      }
  }
  Brain.Screen.printAt(200, 180, "Finished: %f.2", encoderAvg/ticks);
  stopBase();
  
  task::sleep(15);

}
*/


void turnLeftToDegrees(float degrees, int maxSpeed) {
  // Calculates fastest direction to angle and number of ticks to turn to angle
  float initialDegrees = angleWrap(gyro1.value(rotationUnits::deg));
  float turnDegrees = -degrees - initialDegrees;
  
  float targetRange = 0.5; // Distance from the desired angle that is allowed
  float error = turnDegrees; // Distance from the desired range
  float minSpeed = 2;
  float trueSpeed;

  clearEncoders();
  maxSpeed = restrictToRange(maxSpeed, 0, 100);

  while (fabs(error) > targetRange) {
    error = gyro1.value(rotationUnits::deg) + degrees;
    trueSpeed = (error/degrees*maxSpeed)+minSpeed; // Speed approaches one as the angle approaches the desired value

    // Left is reversed, right is forward because positive angles are to the right and reverse on that side turns in that direction.
    frontRight.spin(fwd, trueSpeed, velocityUnits::pct);
    frontLeft.spin(directionType::rev, trueSpeed, velocityUnits::pct);
    rearRight.spin(fwd, trueSpeed, velocityUnits::pct);
    rearLeft.spin(directionType::rev, trueSpeed, velocityUnits::pct);
    Brain.Screen.printAt(1, 140, "True speed: %.2f, error: %.2f", trueSpeed, error);
    Brain.Screen.printAt(1, 180, "Error: %f.2", error);
  }

  stopBase();
  Brain.Screen.printAt(1, 200, "Gyro: %.2f", gyro1.value(rotationUnits::deg));
  task::sleep(15);

}



void turnRightToDegrees(float degrees, int maxSpeed) {
  // Calculates fastest direction to angle and number of ticks to turn to angle
  float initialDegrees = angleWrap(gyro1.value(rotationUnits::deg));
  float turnDegrees = -degrees - initialDegrees;
  
  float targetRange = 0.5; // Distance from the desired angle that is allowed. It stops when it gets within that range
  float error = turnDegrees; // Distance from the desired range
  float trueSpeed;
  float minSpeed = 3;

  clearEncoders();
  maxSpeed = restrictToRange(maxSpeed, 0, 100);

  while (fabs(error) > targetRange) {
    error = -gyro1.value(rotationUnits::deg) + degrees;

    trueSpeed = (error/degrees*maxSpeed)+minSpeed; // Speed approaches one as the angle approaches the desired value

    // Right is reversed, left is forward because positive angles are to the right and reverse turns in that direction.
    frontLeft.spin(fwd, trueSpeed, velocityUnits::pct);
    frontRight.spin(directionType::rev, trueSpeed, velocityUnits::pct);
    rearLeft.spin(fwd, trueSpeed, velocityUnits::pct);
    rearRight.spin(directionType::rev, trueSpeed, velocityUnits::pct);
    Brain.Screen.printAt(1, 140, "True speed: %.2f, error: %.2f", trueSpeed, error);
    Brain.Screen.printAt(1, 180, "Error: %f.2", error);
  }

  stopBase();
  Brain.Screen.printAt(1, 200, "Gyro: %.2f", gyro1.value(rotationUnits::deg));
  task::sleep(15);

}



void armToDegrees(float ticks,  int speed, bool wait = true) {
  if (ticks >= ARM_MAX) ticks = ARM_MAX;
  if (ticks <= ARM_MIN) ticks = ARM_MIN;

  if (!wait) {
    armMotor.startRotateTo(ticks, rotationUnits::deg);

  } else {
    armMotor.rotateTo(ticks, rotationUnits::deg);
    stopBase();
  }
  task::sleep(15);

}



void prep() {
  gyro1.startCalibration(5000);
  task::sleep(3000);
}



void auto1() {
  turnLeftToDegrees(180, 50);
  
  Brain.Screen.printAt(0, 220, "1");
  turnRightToDegrees(45, 25);
  Brain.Screen.printAt(0, 220, "2");
  //turnRightToDegrees(45, 50);


}



void loop() {

float sensitivity = 0.65;
        

    /*
    int leftSpeed = controller1.Axis3.value()*sensitivity;
    int rightSpeed = controller1.Axis2.value()*sensitivity;
    */

    int forwardSpeed = controller1.Axis3.value()/2;
    int turnSpeed = controller1.Axis1.value();

    int leftSpeed = forwardSpeed + round(turnSpeed/2);
    int rightSpeed = forwardSpeed - round(turnSpeed/2);
    
    frontRight.spin(fwd, rightSpeed*sensitivity, velocityUnits::pct);
    frontLeft.spin(fwd, leftSpeed*sensitivity, velocityUnits::pct);
    rearRight.spin(fwd, rightSpeed*sensitivity, velocityUnits::pct);
    rearLeft.spin(fwd, leftSpeed*sensitivity, velocityUnits::pct);
    

    float armSensitivity = 0.25;
    if (controller1.ButtonR1.pressing()) armMotor.rotateTo(ARM_MAX, rotationUnits::deg, 100*armSensitivity, velocityUnits::pct);
    else if (controller1.ButtonR2.pressing()) armMotor.rotateTo(ARM_MIN, rotationUnits::deg, 100*armSensitivity, velocityUnits::pct);
    else armMotor.stop(brakeType::hold);

}




int main() {
  prep();
  auto1();
  Brain.Screen.printAt(200, 50, "Callibrating: %s", gyro1.isCalibrating());
  int frames=0;
  while(true) {
    loop();
    Brain.Screen.printAt(10,50, "Time %d", frames++);
    Brain.Screen.printAt(140,95, "Gyro: %6.2f", gyro1.value(rotationUnits::deg));
  }


}
