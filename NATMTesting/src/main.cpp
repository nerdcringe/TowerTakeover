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

controller controllerPrim(controllerType::primary);

motor RFBASE(PORT2, true);
motor LFBASE(PORT4);
motor RBBASE(PORT1, true);
motor LBBASE(PORT3);
motor ARM(PORT16);
motor RAIL(PORT6);
motor LINTAKE(PORT17);
motor RINTAKE(PORT7);

gyro Gyro1(Brain.ThreeWirePort.G);
pot Pot1(Brain.ThreeWirePort.H);
line cubeDetector(Brain.ThreeWirePort.F); // Line sensor for detecting cube touches.



// USE INCHES!!!

const float WHEEL_CIRCUMFERENCE = 3.25 * PI;

const float TRACK_WIDTH = 7.5; // Width between right and left wheels
const float WHEEL_BASE = 4.5; // Length between front and back
const float CIRCUMDIAMETER = sqrt(pow(TRACK_WIDTH, 2) + pow(WHEEL_BASE, 2)); // Calculate diagonal distance between nonadjacent wheels using pythagorean theorum.
const float BASE_CIRCUMFERENCE = CIRCUMDIAMETER*PI; // Calculate circumference of circle which circumscribes the base.


const float ARM_ENCODER_MAX = 30;


const float POT_ARM_LOW = 246;
const float POT_ARM_MID = 125;
const float POT_ARM_HIGH = 1;

const float RAIL_STACK_POS = 800; // Best angle for stacking cubes (probably perpendicular). Ofc this angle isn't 90 because of gear ratio.
const float RAIL_MAX = 800;
const float RAIL_DEACCEL_POS = 600;





int brightnessThreshold = 65;
int cubeCount = 0;
bool detectingCube = false;





float inchesToTicks(float inches) {
  // Derived from: length/circumference = deg/360
  float ticks = (inches / WHEEL_CIRCUMFERENCE) * 360;
  return ticks;

}


float angleWrap(float degrees) {
  // Keeps angle within range 0 to 360 while preserving angle measure
  while (degrees >= 360) degrees -=360;
  while (degrees < 0) degrees += 360;
  return degrees;

}


float restrictToRange(float number, float bottom, float top) {
  if (number < bottom) number = bottom;
  if (number > top) number = top;
  return number;
}




void armToDeg(float deg, int maxSpeed) {

  const float initialError = deg-Pot1.value(rotationUnits::deg); // Error = desired - actual mesasurement.

  float error = initialError;
  const float acceptableError = 4; // How far off from intended degree value that it's ok to stop at

  const int minSpeed = 5;
  const float deaccelRate = 3;
  Brain.Screen.printAt(0, 60, "%d", fabs(error) > acceptableError);
  Brain.Screen.printAt(25, 60, "%d, %d", error, acceptableError);

  while (fabs(error) > acceptableError) {
    error = deg-Pot1.value(rotationUnits::deg); // Error = desired - actual mesasurement.
    float speed = fabs((error/initialError)*maxSpeed*deaccelRate); // P-controller (proportional to error)
    
    // Limit the speed at maxSpeed because otherwise deaccel rate would increase speed past the max.
    // Keep the speed up to at least minSpeed, because otherwise steady-state error would become impossible to overcome.
    speed = restrictToRange(speed, minSpeed, maxSpeed);
    
    Brain.Screen.printAt(0, 20, "%.2f, %.2f", Pot1.value(rotationUnits::deg), ARM.rotation(rotationUnits::deg));
    Brain.Screen.printAt(0, 40, "Error: %.2f, Speed: %.2f", error, speed);
    
    if (error < 0) {
      ARM.spin(fwd, fabs(speed), velocityUnits::pct);
    } else {
      ARM.spin(directionType::rev, fabs(speed), velocityUnits::pct);
    }

  }

  ARM.stop(brakeType::hold);

}



void armToDegDriver(double deg, int maxSpeed) {
   const float initialError = deg-Pot1.value(rotationUnits::deg); // Error = desired - actual mesasurement.

  float error = initialError;
  const float acceptableError = 4; // How far off from intended degree value that it's ok to stop at

  const int minSpeed = 5;
  const float deaccelRate = 3;
  Brain.Screen.printAt(0, 60, "%d", fabs(error) > acceptableError);
  Brain.Screen.printAt(25, 60, "%d, %d", error, acceptableError);

  while (fabs(error) > acceptableError) {
    error = deg-Pot1.value(rotationUnits::deg); // Error = desired - actual mesasurement.
    float speed = fabs((error/initialError)*maxSpeed*deaccelRate); // P-controller (proportional to error)
    
    // Limit the speed at maxSpeed because otherwise deaccel rate would increase speed past the max.
    // Keep the speed up to at least minSpeed, because otherwise steady-state error would become impossible to overcome.
    speed = restrictToRange(speed, minSpeed, maxSpeed);
    
    Brain.Screen.printAt(0, 20, "%.2f, %.2f", Pot1.value(rotationUnits::deg), ARM.rotation(rotationUnits::deg));
    Brain.Screen.printAt(0, 40, "Error: %.2f, Speed: %.2f", error, speed);

    if (error < 0) {
      ARM.spin(fwd, fabs(speed), velocityUnits::pct);
    } else {
      ARM.spin(directionType::rev, fabs(speed), velocityUnits::pct);
    }

  // Driver control so robot can be controlled inside this loop
  float sensitivity = 0.65;
  float railSpeed = 100;

  int leftSpeed = controllerPrim.Axis3.value() * sensitivity;
  int rightSpeed = controllerPrim.Axis3.value() * sensitivity;
  

  RFBASE.spin(fwd, rightSpeed, velocityUnits::pct);
  LFBASE.spin(fwd, leftSpeed, velocityUnits::pct);
  RBBASE.spin(fwd, rightSpeed, velocityUnits::pct);
  LBBASE.spin(fwd, leftSpeed, velocityUnits::pct);
  



  // Move arm only if ButtonUp or ButtonUp is pressed.
  if (controllerPrim.ButtonUp.pressing()) {
    if (RAIL.rotation(rotationUnits::deg) > RAIL_DEACCEL_POS) {

      // Speed of rail is proportional to error (error/greatest possible error)*speed scale)
      railSpeed = (RAIL_MAX-RAIL.rotation(rotationUnits::deg)) / ((RAIL_MAX-RAIL_DEACCEL_POS) * 100);
    }
    else
    {
      railSpeed = 100;
    }

    RAIL.rotateTo(ARM_ENCODER_MAX, rotationUnits::deg, railSpeed, velocityUnits::pct);

  } else if (controllerPrim.ButtonLeft.pressing()) {
    RAIL.rotateTo(0, rotationUnits::deg, 100, velocityUnits::pct);

  } else {
    RAIL.stop(brakeType::hold);
  }

  // Reset arm if ButtonA is pressed once
  if (controllerPrim.ButtonA.pressing()) {
    RAIL.spin(directionType::rev, 80, velocityUnits::pct);
  }

  controllerPrim.Screen.print("%.2f, %.2f", Pot1.value(rotationUnits::deg), ARM.rotation(rotationUnits::deg));
  }

  ARM.stop(brakeType::hold);
}



// Parameterless functions to put in callback to enable preset arm positions.
void armToDegLowDriver() { armToDegDriver(POT_ARM_LOW, 70); }

void armToDegMidDriver() { armToDegDriver(POT_ARM_MID, 70); }

void armToDegHighDriver() {  armToDegDriver(POT_ARM_HIGH, 70); }



int detectCubes()
{
  while( true )
	{

     // If sensor reading is as dark or darker than brightness threshold, meaning a cube is ~1 cm in front of sensor and blocking light.
    if (cubeDetector.value(percentUnits::pct) < brightnessThreshold)
    {
      // Only increase cube touch counter once when robot starts touching cube (when detectingCube hasn't been set to true yet).
      if (!detectingCube)
      {
        cubeCount += 1;
        detectingCube = true;
      }
    }
    else
    {
      // Reset touching to false if no cube is detected anymore
      detectingCube = false; 
    }

    task::sleep(20);
    Brain.Screen.printAt(0, 120, "Gyro: %.2f", Gyro1.value(rotationUnits::deg));
	}
    return(0);
}




void pre_auton() {

  Gyro1.startCalibration(5000);
  task::sleep(3000);


  // Set up callbacks for moving arm to preset positions 
  controllerPrim.ButtonY.pressed(armToDegLowDriver);
  controllerPrim.ButtonR2.pressed(armToDegMidDriver);
  controllerPrim.ButtonR1.pressed(armToDegHighDriver);
}



void auton() {


}




void userControl() {

  float sensitivity = 0.65;
  float railSpeed = 100;

  int leftSpeed = controllerPrim.Axis3.value() * sensitivity;
  int rightSpeed = controllerPrim.Axis3.value() * sensitivity;
  
  RFBASE.spin(fwd, rightSpeed, velocityUnits::pct);
  LFBASE.spin(fwd, leftSpeed, velocityUnits::pct);
  RBBASE.spin(fwd, rightSpeed, velocityUnits::pct);
  LBBASE.spin(fwd, leftSpeed, velocityUnits::pct);
  


  // Intake controls - intake/outtake for duration of button press.
  if (controllerPrim.ButtonDown.pressing())
  {
    LINTAKE.spin(fwd, 100, velocityUnits::pct);
    RINTAKE.spin(fwd, 100, velocityUnits::pct);
  }
  else if (controllerPrim.ButtonDown.pressing())
  {
    LINTAKE.spin(directionType::rev, 100, velocityUnits::pct);
    RINTAKE.spin(directionType::rev, 100, velocityUnits::pct);
  }
  else
  {
    LINTAKE.stop();
    RINTAKE.stop();
  }



  // Manual arm control (Scuf) - move arm for duration of button press.
  if (controllerPrim.ButtonDown.pressing())
  {
    ARM.spin(fwd, 100, velocityUnits::pct);
  }
  else if (controllerPrim.ButtonB.pressing())
  {
    ARM.spin(directionType::rev, 100, velocityUnits::pct);
  }
  

  // Autostack rail when scuf left trigger is pressed once.
  if (controllerPrim.ButtonRight.pressing()) {
    RAIL.startRotateTo(RAIL_STACK_POS, rotationUnits::deg, 100, velocityUnits::pct);
  }
  // Reset rail pos if ButtonA is pressed once.
  if (controllerPrim.ButtonA.pressing()) {
    RAIL.startRotateTo(0, rotationUnits::deg, 80, velocityUnits::pct);
  }


  // Move rail for duration that ButtonLeft or ButtonUp is pressed.
  if (controllerPrim.ButtonUp.pressing())
  {
    // move rail up slower if it is rotated further than specified position to increase precision.
    if (RAIL.rotation(rotationUnits::deg) < RAIL_DEACCEL_POS) {
      RAIL.spin(fwd, 80, velocityUnits::pct);
    } else {
      RAIL.spin(fwd, 40, velocityUnits::pct);
    }

  }
  else if (controllerPrim.ButtonLeft.pressing())
  {
    RAIL.startRotateTo(0, rotationUnits::deg, 100, velocityUnits::pct);
  }
  else
  {
    RAIL.stop(brakeType::hold);
  }


  controllerPrim.Screen.print("%.2f, %.2f", Pot1.value(rotationUnits::deg), ARM.rotation(rotationUnits::deg));

}




int main() {
  pre_auton();

  // Start running background tasks
  task cubeDetection(detectCubes);



  auton();
  cubeDetection.stop();


  Brain.Screen.printAt(200, 50, "Callibrating: %s", Gyro1.isCalibrating());
  int frames=0;

  while(true) {
    userControl();
    Brain.Screen.printAt(10,50, "Time %d", frames++);
    
    Brain.Screen.printAt(0,80, "Pot: %6.2f, Arm: %6.2f", Pot1.value(rotationUnits::deg), ARM.rotation(rotationUnits::deg));
    Brain.Screen.printAt(140,95, "Gyro: %6.2f", Gyro1.value(rotationUnits::deg));
  }


}
