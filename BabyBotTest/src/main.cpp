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
gyro gyro1(Brain.ThreeWirePort.H);


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


float angleWrap(float degrees) {
  // Keeps angle within range 0 to 360 while preserving angle measure
  while (degrees >= 360) degrees -=360;
  while (degrees < -0) degrees -= 360;
  return degrees;

}


void setMotorSides(float leftSpeed, float rightSpeed) {
  frontRight.spin(fwd, rightSpeed, velocityUnits::pct);
  frontLeft.spin(fwd, leftSpeed, velocityUnits::pct);
  rearRight.spin(fwd, rightSpeed, velocityUnits::pct);
  rearLeft.spin(fwd, leftSpeed, velocityUnits::pct);

}


void stopBase() {
  frontRight.stop(brakeType::hold);
  frontLeft.stop(brakeType::hold);
  rearRight.stop(brakeType::hold);
  rearLeft.stop(brakeType::hold);

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


void turnToDegrees(float degrees, int speed, bool wait = true) {
  // Calculates fastest direction to angle and number of ticks to turn to angle
  float turnDegrees = degrees - angleWrap(gyro1.value(rotationUnits::deg));
  float ticks = inchesToTicks(CIRCUMFERENCE) * (turnDegrees/360);

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



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  armMotor.resetRotation();
  gyro1.startCalibration();
  
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous( void ) {
  // ..........................................................................
  // Insert autonomous user code here.
  // .........................................................................
  
  forwardForInches(24, 30);
  armToDegrees(45, 10);
  armToDegrees(0, 10);
  forwardForInches(24, 30);
  task::sleep(750);
  turnLeftForDegrees(45, 30);
  task::sleep(1000);
  turnLeftForDegrees(90, 20);
  task::sleep(1000);
  turnLeftForDegrees(45, 10);
  task::sleep(1500);
  turnLeftForDegrees(360, 30);
  task::sleep(1000);
  turnLeftForDegrees(360, 20);
  task::sleep(1000);
  turnLeftForDegrees(360, 40);

  forwardGradual(48, 60, 3000);

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol( void ) {

  while (true) {

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

    //controller1.Screen.print(armMotor.rotation(rotationUnits::deg));
    //vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources. 
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    
    //Run the pre-autonomous function. 
    pre_auton();
       
    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }    
       
}
