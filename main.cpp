// To complete the VEXcode V5 Text project upgrade process, please follow the
// steps below.
// 
// 1. You can use the Robot Configuration window to recreate your V5 devices
//   - including any motors, sensors, 3-wire devices, and controllers.
// 
// 2. All previous code located in main.cpp has now been commented out. You
//   will need to migrate this code to the new "int main" structure createds
//   below and keep in mind any new device names you may have set from the
//   Robot Configuration window. 
// 
// If you would like to go back to your original project, a complete backup
// of your original (pre-upgraded) project was created in a backup folder
// inside of this project's folder.


#include "vex.h"

using namespace vex;



/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\hererjd23                                        */
/*    Created:      Wed Sep 25 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#define PI 3.14159265


// USE INCHES!!!

const float WHEEL_CIRCUMFERENCE = 3.25 * PI;

const float TRACK_WIDTH = 7.5; // Width between right and left wheels
const float WHEEL_BASE = 4.5; // Length between front and back
const float CIRCUMDIAMETER = sqrt(pow(TRACK_WIDTH, 2) + pow(WHEEL_BASE, 2)); // Calculate diagonal distance between nonadjacent wheels using pythagorean theorum
const float CIRCUMFERENCE = CIRCUMDIAMETER*PI; // Calculate circumference of circle which circumscribes rect of wheel contact points to ground.
 
const static float ARM_MAX = 30;
const static float ARM_MIN = 0;


float inchesToTicks(float inches) {
  float ticks = (inches / WHEEL_CIRCUMFERENCE) * 360;
  return ticks;

}

float ticksToInches(float ticks) {
  float inches = (ticks * WHEEL_CIRCUMFERENCE) / 360;
  return inches;

}

/*
float angleWrap(float degrees) {
  // Keeps angle within range 0 to 360 while preserving angle measure
  while (degrees >= 360) degrees -=360;
  while (degrees < 0) degrees += 360;
  return degrees;

}*/


float restrictToRange(float number, float bottom, float top) {
  Brain.Screen.printAt(200, 175, "%.3f", number);
  if (number < bottom) number = bottom;
  if (number > top) number = top;
   return number;
 }


// Movement functions

void setMotors(float leftSpeed, float rightSpeed) {
  frontRight.spin(fwd, rightSpeed, pct);
  frontLeft.spin(fwd, leftSpeed, pct);
  rearRight.spin(fwd, rightSpeed, pct);
  rearLeft.spin(fwd, leftSpeed, pct);

}


void stopBase(float PID = true) {
  if (PID) {
    frontRight.stop(brakeType::hold);
    frontLeft.stop(brakeType::hold);
    rearRight.stop(brakeType::hold);
    rearLeft.stop(brakeType::hold);
  } else {
    frontRight.stop(brakeType::coast);
    frontLeft.stop(brakeType::coast);
    rearRight.stop(brakeType::coast);
    rearLeft.stop(brakeType::coast);

  }
}


void clearEncoders(){
  frontRight.resetRotation();
  frontLeft.resetRotation();
  rearRight.resetRotation();
  rearLeft.resetRotation();

}




void gyroTurn(float degrees, int maxSpeed) {

  float initialAngle = gyro1.value(deg);
  // positive degrees == right; negative degrees == left
  float turnDegrees = degrees-gyro1.value(deg);

  float targetRange = 0.5; // Distance from the desired angle that is allowed
  float error = turnDegrees; // Distance from the desired range
  float progress; // Degrees the robot has turned already
  float minSpeed = 2; // Lowest speed the motors will go; Turning is generally more precise and accurate when lower.
  float speed; // Actual speed value of the motors

  maxSpeed = restrictToRange(maxSpeed, 0, 100);
  
  while (fabs(error) > targetRange) {
    progress = gyro1.value(deg)-initialAngle;

    /*if (turnDegrees < 0) error = -turnDegrees+progress;
    else */error = progress-turnDegrees;

    speed = ((1-(progress/turnDegrees))*maxSpeed)+minSpeed; // Speed starts at maximum and approaches minimum as the gyro value approaches the desired angle. It deccelerates for precision and accuray.
    
    speed = restrictToRange(speed, 0, maxSpeed);

    if (turnDegrees < 0) {
      frontRight.spin(forward, speed, pct);
      frontLeft.spin(reverse, speed, pct);
      rearRight.spin(forward, speed, pct);
      rearLeft.spin(reverse, speed, pct);
    } else {
      frontLeft.spin(forward, speed, pct);
      frontRight.spin(reverse, speed, pct);
      rearLeft.spin(forward, speed, pct);
      rearRight.spin(reverse, speed, pct);
    }

    Brain.Screen.printAt(1, 120, "Desired angle: %.2f, Turn degrees: %.2f", degrees, turnDegrees);
    Brain.Screen.printAt(1, 140, "Speed: %.2f, Error: %.2f", speed, error);
    Brain.Screen.printAt(1, 180, "Gyro: %.2f", gyro1.value(deg));

  }

  stopBase();
  task::sleep(15);

}



void forwardInches(float inches, int maxSpeed) {
  float ticks = inchesToTicks(inches);
  float initialAngle = gyro1.value(deg);

  float targetRange = 0.5; // Distance from the desired distance the robot has to be to stop.
  float error = ticks; // Distance from the desired range
  float progress; // Ticks the motors has turned already
  float minSpeed = 2; // Lowest speed the motors will go; Turning is generally more precise and accurate when lower.
  float speed; // Actual speed value of the motors

  maxSpeed = restrictToRange(maxSpeed, 0, 100);
  clearEncoders();

  while (error > targetRange) {

    progress = (frontLeft.rotation(deg)+frontRight.rotation(deg)+rearLeft.rotation(deg)+rearRight.rotation(deg))/4; // Average value of all motors
    error = ticks-progress;
    
    // First half: accelerate to max; Second half, deccelerate to min
    if (error >= ticks/2) speed = (((progress/ticks))*maxSpeed*6)+minSpeed;
    else speed = ((1-(progress/ticks))*maxSpeed*3)+minSpeed;
    speed = restrictToRange(speed, minSpeed, maxSpeed);

    frontRight.spin(forward, speed, pct);
    frontLeft.spin(forward, speed, pct);
    rearRight.spin(forward, speed, pct);
    rearLeft.spin(forward, speed, pct);

    Brain.Screen.printAt(1, 120, "Desired distance: %.2f, Progress: %.2f", ticksToInches(ticks), ticksToInches(progress));
    Brain.Screen.printAt(1, 140, "Speed: %.2f, Error: %.2f", speed, error);

  }

  stopBase();
  task::sleep(15);

}



void backwardInches(float inches, int maxSpeed) {
  float ticks = inchesToTicks(inches);
  float initialAngle = gyro1.value(deg);

  float targetRange = 0.5; // Distance from the desired distance the robot has to be to stop.
  float error = ticks; // Distance from the desired range
  float progress; // Ticks the motors has turned already
  float minSpeed = 2; // Lowest speed the motors will go; Turning is generally more precise and accurate when lower.
  float speed; // Actual speed value of the motors

  maxSpeed = restrictToRange(maxSpeed, 0, 100);
  clearEncoders();

  while (error > targetRange) {

    progress = - (frontLeft.rotation(deg));//(frontLeft.rotation(deg)+frontRight.rotation(deg)+rearLeft.rotation(deg)+rearRight.rotation(deg))/4; // Average value of all motors
    error = ticks-progress;

    // First half: accelerate to max; Second half, deccelerate to min
    if (error >= ticks/2) speed = (((progress/ticks))*maxSpeed*5)+minSpeed;
    else speed = ((1-(progress/ticks))*maxSpeed*2)+minSpeed;
    speed = restrictToRange(speed, minSpeed, maxSpeed);

    frontRight.spin(reverse, speed, pct);
    frontLeft.spin(reverse, speed, pct);
    rearRight.spin(reverse, speed, pct);
    rearLeft.spin(reverse, speed, pct);

    Brain.Screen.printAt(1, 120, "Desired distance: %.2f, Progress: %.2f", ticksToInches(ticks), ticksToInches(progress));
    Brain.Screen.printAt(1, 140, "Speed: %.2f, Error: %.2f", speed, error);

  }

  /*task::sleep(1000);
  Brain.Screen.printAt(1, 160, "Turning from to: %.2f", gyro1.value(deg));
  gyroTurn(initialAngle, maxSpeed);
  
  task::sleep(1500);*/
  stopBase();
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
  task::sleep(2000);

}
 


void auto1() {
  
  //forwardInches(24, 90);
  //backwardInches(24, 50);
  gyroTurn(90, 75);
  gyroTurn(-90, 75);
  gyroTurn(0, 75);


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
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  prep();
  auto1();
  //task::sleep(500);
  //stopBase(false);

  Brain.Screen.printAt(200, 50, "Callibrating: %s", gyro1.isCalibrating());
  int frames=0;
  while(true) {

    loop();
    Brain.Screen.printAt(10,50, "Time %d", frames++);
    Brain.Screen.printAt(30,95, "Gyro: %6.2f", gyro1.value(rotationUnits::deg));

   }
}
