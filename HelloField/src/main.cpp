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


// USE INCHES!!!
float wheelCircumference = 4 * PI;

float trackWidth = 7.5; // 
float wheelBase = 4.5; // 
float diagonal = sqrt(pow(trackWidth, 2) + pow(wheelBase, 2)); // Calculate diagonal distance between nonadjacent wheels using pythagorean theorum
float circumference = diagonal *PI; // Calculate circumference of circle in which the wheel.
float fullRotationTicks = (circumference/wheelCircumference) * 360; // Calculate encoder ticks needed to rotate robot 360 degrees


float inchesToTicks(float inches) {
  float ticks = (360/wheelCircumference) * inches;
  return ticks;

}


void forwardForInches(float inches, int speed) {
  float ticks = inchesToTicks(inches);
  frontRight.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
  frontLeft.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
  rearRight.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
  rearLeft.rotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);

  frontRight.stop();
  frontLeft.stop();
  rearRight.stop();
  rearLeft.stop();
  task::sleep(25);


}


void turnLeftForDegrees(float degrees, int speed, bool wait = false) {
  float inches = fullRotationTicks * (degrees/360);
  float ticks = inchesToTicks(inches);

  frontRight.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
  frontLeft.startRotateFor(directionType::rev, ticks, rotationUnits::deg, speed, velocityUnits::pct);
  rearRight.startRotateFor(ticks, rotationUnits::deg, speed, velocityUnits::pct);
  rearLeft.rotateFor(directionType::rev, ticks, rotationUnits::deg, speed, velocityUnits::pct);

  if (wait) {
    frontRight.stop();
    frontLeft.stop();
    rearRight.stop();
    rearLeft.stop();
  }
  task::sleep(25);
}


void turnRightForDegrees(float degrees, int speed, bool wait = false){
  turnLeftForDegrees(-degrees, speed, wait);

}



void armToDegrees(float ticks,  int speed) {
  armMotor.rotateTo(ticks, rotationUnits::deg);
  armMotor.stop(brakeType::hold);
  task::sleep(25);

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
  forwardForInches(25, 70);
  armToDegrees(45, 10);
  armToDegrees(0, 10);
  forwardForInches(25, 30);
  task::sleep(750);
  turnLeftForDegrees(45, 50);
  task::sleep(1000);
  turnRightForDegrees(90, 65);
  task::sleep(1000);
  turnLeftForDegrees(360, 35);

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
    
    frontRight.spin(fwd, rightSpeed, velocityUnits::pct);
    frontLeft.spin(fwd, leftSpeed, velocityUnits::pct);
    rearRight.spin(fwd, rightSpeed, velocityUnits::pct);
    rearLeft.spin(fwd, leftSpeed, velocityUnits::pct);
    

    float armSensitivity = 0.25;
    if (controller1.ButtonR1.pressing()) armMotor.rotateTo(30, rotationUnits::deg, 100*armSensitivity, velocityUnits::pct);
    else if (controller1.ButtonR2.pressing()) armMotor.rotateTo(0, rotationUnits::deg, 100*armSensitivity, velocityUnits::pct);
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