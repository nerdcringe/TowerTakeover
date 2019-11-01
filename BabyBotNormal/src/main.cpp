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

const float TILE_LENGTH = 24;

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


float angleWrap(float degrees) {
  // Keeps angle within range 0 to 360 while preserving angle measure
  while (degrees >= 360) degrees -=360;
  while (degrees < 0) degrees += 360;
  return degrees;

}


float restrictToRange(float number, float bottom, float top) {
  Brain.Screen.printAt(200, 175, "%.3f", number);
  if (number < bottom) number = bottom;
  if (number > top) number = top;
   return number;
 }


// Movement functions

void spinMotors(float leftSpeed, float rightSpeed) {
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



void alignPerpendicular(float speed) {
  // Lines robot up to nearest 90 degree angle relative to starting angle.
  // Make sure robot angle when program starts is perpendicular or parallel to the wall
  float unalignedAngle = angleWrap(gyro1.value(deg));
  int multiple = 90;
  float nearestAngleMultiple = round(unalignedAngle/multiple)*multiple;
  gyroTurn(nearestAngleMultiple, speed);

}



void forwardInches(float inches, int maxSpeed) {
  float ticks = inchesToTicks(inches);
  //float initialAngle = gyro1.value(deg);

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
  //float initialAngle = gyro1.value(deg);

  float targetRange = 0.5; // Distance from the desired distance the robot has to be to stop.
  float error = ticks; // Distance from the desired range
  float progress; // Ticks the motors has turned already
  float minSpeed = 2; // Lowest speed the motors will go; Turning is generally more precise and accurate when lower.
  float speed; // Actual speed value of the motors

  maxSpeed = restrictToRange(maxSpeed, 0, 100);
  clearEncoders();

  while (error > targetRange) {

    progress = -(frontLeft.rotation(deg));//(frontLeft.rotation(deg)+frontRight.rotation(deg)+rearLeft.rotation(deg)+rearRight.rotation(deg))/4; // Average value of all motors
    error = ticks-progress;

    // First half: accelerate to max; Second half, deccelerate to min
    if (error >= ticks/2)
      speed = (((progress/ticks))*maxSpeed*5)+minSpeed;
    else
      speed = ((1-(progress/ticks))*maxSpeed*2)+minSpeed;

    speed = restrictToRange(speed, minSpeed, maxSpeed);

    frontRight.spin(reverse, speed, pct);
    frontLeft.spin(reverse, speed, pct);
    rearRight.spin(reverse, speed, pct);
    rearLeft.spin(reverse, speed, pct);

    Brain.Screen.printAt(1, 120, "Desired distance: %.2f, Progress: %.2f", ticksToInches(ticks), ticksToInches(progress));
    Brain.Screen.printAt(1, 140, "Speed: %.2f, Error: %.2f", speed, error);

  }

  stopBase();
  task::sleep(15);

}



void driftTurnForward(float inches, double degrees, int maxSpeed) {
  float desiredDistance = inchesToTicks(inches);
  float distanceTargetRange = 6; // Distance from the desired distance the robot has to be to stop.
  float distanceProgress; // Ticks the motors has turned already
  float distanceError = desiredDistance; // Distance of value from the desired range
  bool drivingForward = true;

  // positive degrees == right; negative degrees == left
  float initialAngle = gyro1.value(deg);
  float turnDegrees = degrees-gyro1.value(deg);
  float turningTargetRange = 1; // Distance from the desired angle that is allowed
  float turnProgress; // Degrees the robot has turned already
  float turnError = desiredDistance; // Degrees that the robot needs to turn
  bool turning = true;

  float minSpeed = 2; // Lowest speed the motors will go; Turning is generally more precise and accurate when lower. It takes exponentially longer the smaller it is.
  float forwardSpeed; // Base speed for motors (before turning speed added/subtracted)
  float turnSpeed; // Value added or subtracted from motor speeds to make robot turn

  directionType leftDirection;
  directionType rightDirection;
  float leftSpeed; // Actual speed value of motors
  float rightSpeed;

  maxSpeed = restrictToRange(maxSpeed, minSpeed, 100);
  clearEncoders();

  while (drivingForward || turning) {
    if ((fabs(distanceError)) < distanceTargetRange)
      drivingForward = false;
    distanceProgress = (rearLeft.rotation(deg)+rearRight.rotation(deg)+rearRight.rotation(deg)+frontRight.rotation(deg))/4;// Average value of all motors on that side
    distanceError = desiredDistance-distanceProgress;

    // Accelerate until at midway point, then deccelerate
    if (distanceError >= desiredDistance/2)
      forwardSpeed = (((distanceProgress/desiredDistance))*maxSpeed*5)+minSpeed;
    else
      forwardSpeed = ((1-(distanceProgress/desiredDistance))*maxSpeed*1)+minSpeed;


    if ((fabs(turnError)) < turningTargetRange)
      turning = false;
    else turning = true;

    turnProgress = gyro1.value(deg)-initialAngle;
    turnError = turnProgress-turnDegrees;
    turnSpeed = ((1-(turnProgress/turnDegrees))*maxSpeed)+minSpeed; // Speed starts at maximum and approaches minimum as the gyro value approaches the desired angle. It deccelerates for precision and accuray.
    leftSpeed = 0;
    rightSpeed = 0;

    // Makes motors move (forward and turning) if not within target range for that type of motion. If within, it stops changing speed for that motion
    if (drivingForward) {
      // Tells motors what speed to move at by adding/subtracting, depending on if the robot must go forwards or backwards (to correct error)
      forwardSpeed = restrictToRange(forwardSpeed, minSpeed, maxSpeed);
      if (distanceError > 0) {
        leftSpeed += forwardSpeed;
        rightSpeed += forwardSpeed;
      } else {
        leftSpeed -= forwardSpeed;
        rightSpeed -= forwardSpeed;
      }
    }
  
    if (turning) {
      // Tells motors what speed to turn at by adding/subtracting, depending on which side the robot needs to turn
      turnSpeed = restrictToRange(turnSpeed, minSpeed, maxSpeed);
      if (turnError <= 0) {
        leftSpeed += turnSpeed;
        rightSpeed -= turnSpeed;
      } else {
        leftSpeed -= turnSpeed;
        rightSpeed += turnSpeed;
      }

    } else
      turnSpeed = 0;

    if (leftSpeed >= 0) {
      leftDirection = forward;
    } else {
      leftDirection = reverse;
    }

    if (rightSpeed >= 0) {
      rightDirection = forward;
    } else {
      rightDirection = reverse;
    }

    // If robot is either done driving forward or turning, let the robot stop completely when the other requirement is met.
    if (!drivingForward || !turning) {
      leftSpeed = restrictToRange(leftSpeed, 0, maxSpeed);
      rightSpeed = restrictToRange(rightSpeed, 0, maxSpeed);
    } else {
      leftSpeed = restrictToRange(leftSpeed, minSpeed, maxSpeed);
      rightSpeed = restrictToRange(rightSpeed, minSpeed, maxSpeed);
    }

    frontLeft.spin(leftDirection, leftSpeed, pct);
    rearLeft.spin(leftDirection, leftSpeed, pct);
    rearRight.spin(rightDirection, rightSpeed, pct);
    frontRight.spin(rightDirection, rightSpeed, pct);

    if (!drivingForward && !turning) 
        break;

    Brain.Screen.printAt(1, 80, "Turning: %.2f, drivingForward: %f.2", turning, drivingForward);
    Brain.Screen.printAt(1, 100, "Desired distance: %.2f, distanceProgress: %.2f", ticksToInches(desiredDistance), ticksToInches(distanceProgress));
    Brain.Screen.printAt(1, 120, "Desired angle: %.2f, angleProgress: %.2f", ticksToInches(turnDegrees), ticksToInches(turnProgress));
    Brain.Screen.printAt(1, 140, "leftSpeed: %.2f, rightSpeed: %.2f", leftSpeed, rightSpeed);
    Brain.Screen.printAt(1, 160, "distanceError: %.2f, turnError: %.2f", distanceError, turnError);
    Brain.Screen.printAt(1, 180, " Gyro: %.2f", gyro1.value(deg));
  }
  
  stopBase();
  Brain.Screen.printAt(1, 180, "Done.");
  task::sleep(15);

}



void driftTurnBackward(float inches, double degrees, int maxSpeed) {
  float desiredDistance = inchesToTicks(inches);
  float distanceTargetRange = 6; // Distance from the desired distance the robot has to be to stop.
  float distanceProgress; // Ticks the motors has turned already
  float distanceError = desiredDistance; // Distance of value from the desired range
  bool drivingBackward = true;

  // positive degrees == right; negative degrees == left
  float initialAngle = gyro1.value(deg);
  float turnDegrees = degrees-gyro1.value(deg);
  float turningTargetRange = 1; // Distance from the desired angle that is allowed
  float turnProgress; // Degrees the robot has turned already
  float turnError = desiredDistance; // Degrees that the robot needs to turn
  bool turning = true;

  float minSpeed = 2; // Lowest speed the motors will go; Turning is generally more precise and accurate when lower. It takes exponentially longer the smaller it is.
  float backwardSpeed; // Base speed for motors (before turning speed added/subtracted)
  float turnSpeed; // Value added or subtracted from motor speeds to make robot turn

  directionType leftDirection;
  directionType rightDirection;
  float leftSpeed; // Actual speed value of motors
  float rightSpeed;

  maxSpeed = restrictToRange(maxSpeed, minSpeed, 100);
  clearEncoders();

  while (drivingBackward || turning) {
    if ((fabs(distanceError)) < distanceTargetRange)
      drivingBackward = false;
    distanceProgress = -(rearLeft.rotation(deg)+rearRight.rotation(deg)+rearRight.rotation(deg)+frontRight.rotation(deg))/4;// Average value of all motors on that side
    distanceError = desiredDistance-distanceProgress;

    // Accelerate until at midway point, then deccelerate
    if (distanceError >= desiredDistance/2)
      backwardSpeed = ((-(distanceProgress/desiredDistance))*maxSpeed*4)-minSpeed;
    else
      backwardSpeed = (-(1-(distanceProgress/desiredDistance))*maxSpeed)-minSpeed;


    if ((fabs(turnError)) < turningTargetRange)
      turning = false;
    else turning = true;

    turnProgress = gyro1.value(deg)-initialAngle;
    turnError = turnProgress-turnDegrees;
    turnSpeed = ((1-(turnProgress/turnDegrees))*maxSpeed)+minSpeed; // Speed starts at maximum and approaches minimum as the gyro value approaches the desired angle. It deccelerates for precision and accuray.
    leftSpeed = 0;
    rightSpeed = 0;

    // Makes motors move (forward and turning) if not within target range for that type of motion. If within, it stops changing speed for that motion
    if (drivingBackward) {
      // Tells motors what speed to move at by adding/subtracting, depending on if the robot must go backwards or forwards (to correct error)
      backwardSpeed = restrictToRange(backwardSpeed, -maxSpeed, -minSpeed);
      if (distanceError < 0) {
        leftSpeed += backwardSpeed;
        rightSpeed += backwardSpeed;
      } else {
        leftSpeed -= backwardSpeed;
        rightSpeed -= backwardSpeed;
      }
    }
  
    if (turning) {
      // Tells motors what speed to turn at by adding/subtracting, depending on which side the robot needs to turn
      turnSpeed = restrictToRange(turnSpeed, minSpeed, maxSpeed);
      if (turnError >= 0) {
        leftSpeed += turnSpeed;
        rightSpeed -= turnSpeed;
      } else {
        leftSpeed -= turnSpeed;
        rightSpeed += turnSpeed;
      }

    } else
      turnSpeed = 0;

    if (leftSpeed >= 0) {
      leftDirection = reverse;
    } else {
      leftDirection = forward;
    }

    if (rightSpeed >= 0) {
      rightDirection = reverse;
    } else {
      rightDirection = forward;
    }

    // If robot is either done driving forward or turning, let the robot stop completely when the other requirement is met.
    if (!drivingBackward || !turning) {
      leftSpeed = restrictToRange(leftSpeed, 0, maxSpeed);
      rightSpeed = restrictToRange(rightSpeed, 0, maxSpeed);
    } else {
      leftSpeed = restrictToRange(leftSpeed, minSpeed, maxSpeed);
      rightSpeed = restrictToRange(rightSpeed, minSpeed, maxSpeed);
    }

    frontLeft.spin(leftDirection, leftSpeed, pct);
    rearLeft.spin(leftDirection, leftSpeed, pct);
    rearRight.spin(rightDirection, rightSpeed, pct);
    frontRight.spin(rightDirection, rightSpeed, pct);

    if (!drivingBackward && !turning) 
        break;

    Brain.Screen.printAt(1, 100, "Turning: %.2f, drivingBackward: %f.2", turning, drivingBackward);
    Brain.Screen.printAt(1, 120, "backwardSpeed: %.2f, turnSpeed: %.2f", backwardSpeed, turnSpeed);
    Brain.Screen.printAt(1, 140, "leftSpeed: %.2f, rightSpeed: %.2f", leftSpeed, rightSpeed);
    Brain.Screen.printAt(1, 160, "distanceError: %.2f, turnError: %.2f", distanceError, turnError);
    Brain.Screen.printAt(1, 180, " Gyro: %.2f", gyro1.value(deg));
  }
  
  stopBase();
  Brain.Screen.printAt(1, 180, "Done.");
  task::sleep(15);

}



void armToDegrees(float degrees,  int speed, bool wait = true) {
  if (degrees >= ARM_MAX) degrees = ARM_MAX;
  if (degrees <= ARM_MIN) degrees = ARM_MIN;

  if (!wait) {
    armMotor.startRotateTo(degrees, rotationUnits::deg);

  } else {
    armMotor.rotateTo(degrees, rotationUnits::deg);
    stopBase();
  }
  task::sleep(15);
}


void armToMax(float speed, bool wait = true) {
  armToDegrees(ARM_MAX, speed, wait);
}


void armToMin(float speed, bool wait = true) {
  armToDegrees(ARM_MIN, speed, wait);
}



void prep() {

  gyro1.startCalibration(5000);
  task::sleep(2000);

}



void auto1(){
  //gyroTurn(90, 60);
  driftTurnBackward(24, 90, 85);

}


void skillsAuto() {
  // Blue back side

  armToMax(1, false);
  forwardInches(TILE_LENGTH*0.75, 80);

  // pick up first line of 4 cubes
  task::sleep(1000);

  // lift cube carrier
  // start spinning intake
  forwardInches(1.25*TILE_LENGTH, 80);

  forwardInches(1.5*TILE_LENGTH, 80);
  // pick up next line of 4 cubes
  forwardInches(1.25*TILE_LENGTH, 80);
  task::sleep(1000);

  backwardInches(3*TILE_LENGTH, 90);

  // turn around and go forward to approach goal
  gyroTurn(-157.5, 65);

  forwardInches(18, 80);
  task::sleep(1000);
  // angle cube holder straight up to deposit cube stack
  backwardInches(0.93*TILE_LENGTH, 35);
  gyroTurn(0, 70);



}


void blueBackAuto() {
  // Blue back side

  // Lift cube carrier and start spinning intake to prepare to succ cube.
  armToMax(10);

  // Pick up first line of 4 cubes
  forwardInches(TILE_LENGTH*1.8, 90);

  //Drift back to second row of cubes, then go forwards, making sure to pick up all 3 cubes in row.
  driftTurnBackward(1.3*TILE_LENGTH, -90, 85);
  gyroTurn(0, 80); 
  forwardInches(1.35*TILE_LENGTH, 90);

  //Turn towards the goal, move towards it, then deposit cube stack.
  driftTurnBackward(100, 95, 85);
  gyroTurn(180, 80);
  armToMin(10);


}


void blueFrontAuto() {
// Blue front side

}


void redFrontAuto() {
// Red front side

}



void usercontrol() {
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
  //auto1();
  blueBackAuto();
  task::sleep(500);
  stopBase(false);

  Brain.Screen.printAt(200, 50, "Callibrating: %s", gyro1.isCalibrating());
  int frames=0;
  while(true) {

    usercontrol();
    Brain.Screen.printAt(10, 40, "Time %d", frames++);
    Brain.Screen.printAt(30, 60, "Gyro: %6.2f", gyro1.value(rotationUnits::deg));

   }
}
