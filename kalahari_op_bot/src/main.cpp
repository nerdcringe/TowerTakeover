/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

#define PI 3.14159265

const double WHEEL_CIRCUMFERENCE = 3.25 * PI;

const double TRACK_WIDTH = 7.5; // Width between right and left wheels
const double WHEEL_BASE = 4.5;  // Length between front and back
const double CIRCUMDIAMETER =
    sqrt(pow(TRACK_WIDTH, 2) +
         pow(WHEEL_BASE, 2)); // Calculate diagonal distance between nonadjacent
                              // wheels using pythagorean theorum.
const double BASE_CIRCUMFERENCE =
    CIRCUMDIAMETER *
    PI; // Calculate circumference of circle which circumscribes the base.

// Specify desired potentiometer degree to slow down rail at.
const double RAIL_POT_DEACCEL = 180;

// Specify desired potentiometer positions for preset rail and arm positions for
// driver.
const double POT_RAIL_STACK = 96;
const double POT_RAIL_RESET = 220;

/*
const double POT_ARM_LOW = 200;
const double POT_ARM_MID = 150;
const double POT_ARM_HIGH = 100;
*/

// Specify encoder preset positions for arm in driver.
const double ARM_LOW = 0;
const double ARM_MID = 150;
const double ARM_HIGH = 300;

// Declare gyro self correct PID variable
double kp = .39; // the proportional term
double desiredDegree = 0.0;
double KPC = 50.0;
double KDC = 0.12;
double correctionL = 0;
double correctionR = 0;
double cSpeed = 0;
double desiredX =
    0; // Desired value for horizontal encoder to be. Robot will turn to make
       // xTracker go back to this position if it deviates.
double rotationStart = 0; // change for doing autonomous in chunks

int lineThreshold =
    45; // REPLACE REPLACE REPLACE REPLACE REPLACE REPLACE REPLACE REPLACE
        // REPLACE REPLACE REPLACE REPLACE REPLACE REPLACE

// Cube detection
/*
bool cubeDetectEnabled = true;
int brightnessThreshold = 65;
int cubeCount = 0;
bool detectingCube = false;

double sensitivity = 1;
double railSpeed = 100;
*/

// A global instance of brain used for printing to the V5 brain screen
// brain Brain;
controller controllerPrim(controllerType::primary);
// brain  Brain;
/*// baby bot ports
motor RFBASE(PORT6, true);
motor LFBASE(PORT12);
motor RBBASE(PORT20, true);
motor LBBASE(PORT17);
*/

motor RFBASE(PORT12);       // true for old bot
motor LFBASE(PORT11, true); // for new bot port 15 && for old bot port 11
motor RBBASE(PORT15);       // true for old bot
motor LBBASE(PORT14, true);

// motor ARM(PORT2);
// motor RAIL(PORT6, true);
motor LINTAKE(PORT2);
motor RINTAKE(PORT18, true);
// Top & bottom outtake roller motors
motor TROLLER = motor(PORT7, true); // for new bot port 2 && for old bot port 1
motor BROLLER = motor(PORT10, false);

/*
///// new robot motor config
motor RFBASE(PORT12);
motor LFBASE(PORT11,true); //for new bot port 15 && for old bot port 11
motor RBBASE(PORT13);
motor LBBASE(PORT14,true);

//motor ARM(PORT2);
//motor RAIL(PORT6, true);
motor LINTAKE(PORT2);
motor RINTAKE(PORT18, true);
// Top & bottom outtake roller motors
motor TROLLER = motor(PORT7, true); // for new bot port 2 && for old bot port 1
motor BROLLER = motor(PORT10, false);
*/

inertial Inertial(PORT20);
/*
encoder xTracker(Brain.ThreeWirePort.C); // Tracks horizontal shift of robot.
Used to keep robot in straight line.

pot RailPot(Brain.ThreeWirePort.E);
pot ArmPot(Brain.ThreeWirePort.F);
line cubeDetector(Brain.ThreeWirePort.D); // Line sensor for detecting cube
touches.
*/
triport threeWirePort(PORT22);

line TINDEX1(threeWirePort.A);
line TINDEX2(threeWirePort.D);
line BINDEX(threeWirePort.B);
line PINDEX(threeWirePort.C);

// Ball detected is a lower line sensor value
bool tBallDetected() {
  int val = 67;
  return TINDEX1.value(pct) <= val || TINDEX2.value(pct) <= val;
}

bool bBallDetected() { return BINDEX.value(pct) <= 66; }
bool poopBallDetected() { return PINDEX.value(pct) <= 66; }

double inchesToTicks(double inches) {
  // Derived from: length/circumference = deg/360
  double ticks = (inches / WHEEL_CIRCUMFERENCE) * 360;
  return ticks;
}

double ticksToInches(double ticks) {
  double inches = (ticks * WHEEL_CIRCUMFERENCE) / 360;
  return inches;
}

double keepInRange(double number, double bottom, double top) {
  if (number < bottom)
    number = bottom;
  if (number > top)
    number = top;
  return number;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                           Base Movement Functions                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void stopBase() {
  RFBASE.stop(brakeType::coast);
  LFBASE.stop(brakeType::coast);
  RBBASE.stop(brakeType::coast);
  LBBASE.stop(brakeType::coast);
}

void clearEncoders() {
  RFBASE.resetRotation();
  LFBASE.resetRotation();
  RBBASE.resetRotation();
  LBBASE.resetRotation();
}

double getBaseAvg() {
  return (LFBASE.rotation(deg) + LBBASE.rotation(deg) + RFBASE.rotation(deg) +
          RBBASE.rotation(deg)) /
         2;
}

void moveRightBase(int power) {
  RFBASE.spin(directionType::fwd, power, velocityUnits::pct);
  RBBASE.spin(directionType::fwd, power, velocityUnits::pct);
}

void moveLeftBase(int power) {
  LFBASE.spin(directionType::fwd, power, velocityUnits::pct);
  LBBASE.spin(directionType::fwd, power, velocityUnits::pct);
}

void turnBase(int power) // positive is clockwise, negative is counterclockwise
{
  LFBASE.spin(directionType::fwd, -power, velocityUnits::pct);
  LBBASE.spin(directionType::fwd, -power, velocityUnits::pct);
  RFBASE.spin(directionType::fwd, power, velocityUnits::pct);
  RBBASE.spin(directionType::fwd, power, velocityUnits::pct);
}

double getRotation() {
  return -Inertial.rotation(deg) +
         rotationStart; // IMPORTANT that this is NEGATIVE
}

double getHeading() { return Inertial.value() + rotationStart; }

void forwardInches(double inches, int maxSpeed) {
  double ticks = inchesToTicks(inches);
  // double initialAngle = Gyro1.value(deg);

  double targetRange =
      2; // Distance from the desired distance the robot has to be to stop.
  double error = ticks; // Distance from the desired range
  double progress;      // Ticks the motors has turned already
  double minSpeed =
      2; // Lowest speed the motors will go; Turning is more precise when lower.
  double accelRate = 30;
  double deaccelRate = 12;

  double speed; // Actual speed value of the motors

  maxSpeed = keepInRange(maxSpeed, 0, 100);
  clearEncoders();

  while (error > targetRange) {

    progress = (LFBASE.rotation(deg) + RFBASE.rotation(deg) +
                LBBASE.rotation(deg) + RBBASE.rotation(deg)) /
               4; // Average value of all motors
    error = ticks - progress;

    // First half: accelerate to max; Second half, deccelerate to min
    if (error >= ticks / 2) {
      speed = (((progress / ticks)) * maxSpeed * accelRate) + minSpeed;
    } else {
      speed = ((1 - (progress / ticks)) * maxSpeed * deaccelRate) + minSpeed;
    }
    speed = keepInRange(speed, minSpeed, maxSpeed);

    RFBASE.spin(fwd, speed /*+ correctionR*/, pct);
    LFBASE.spin(fwd, speed /*+ correctionL*/, pct);
    RBBASE.spin(fwd, speed /*+ correctionR*/, pct);
    LBBASE.spin(fwd, speed /* + *correctionL */, pct);

    // Brain.Screen.printAt(1, 120, "Desired distance: %.2f, Progress: %.2f",
    // ticksToInches(ticks), ticksToInches(progress));
    // Brain.Screen.printAt(1, 140, "Speed: %.2f, Error: %.2f", speed, error);
  }

  stopBase();
  task::sleep(15);
}

// If time (ms) in function is > maxTime, exit the loop
void forwardInchesTimed(double inches, int maxSpeed, int maxTimeMs) {
  double ticks = inchesToTicks(inches);
  // double initialAngle = Gyro1.value(deg);

  double targetRange =
      2; // Distance from the desired distance the robot has to be to stop.
  double error = ticks; // Distance from the desired range
  double progress;      // Ticks the motors has turned already
  double minSpeed =
      2; // Lowest speed the motors will go; Turning is more precise when lower.
  double accelRate = 30;
  double deaccelRate = 12;

  double speed; // Actual speed value of the motors

  maxSpeed = keepInRange(maxSpeed, 0, 100);
  clearEncoders();
  timer Timer;
  Timer.clear();

  while (error > targetRange && Timer.time(msec) <= maxTimeMs) {

    progress = (LFBASE.rotation(deg) + RFBASE.rotation(deg) +
                LBBASE.rotation(deg) + RBBASE.rotation(deg)) /
               4; // Average value of all motors
    error = ticks - progress;

    // First half: accelerate to max; Second half, deccelerate to min
    if (error >= ticks / 2) {
      speed = (((progress / ticks)) * maxSpeed * accelRate) + minSpeed;
    } else {
      speed = ((1 - (progress / ticks)) * maxSpeed * deaccelRate) + minSpeed;
    }
    speed = keepInRange(speed, minSpeed, maxSpeed);

    RFBASE.spin(fwd, speed /*+ correctionR*/, pct);
    LFBASE.spin(fwd, speed /*+ correctionL*/, pct);
    RBBASE.spin(fwd, speed /*+ correctionR*/, pct);
    LBBASE.spin(fwd, speed /* + *correctionL */, pct);

    // Brain.Screen.printAt(1, 120, "Desired distance: %.2f, Progress: %.2f",
    // ticksToInches(ticks), ticksToInches(progress));
    // Brain.Screen.printAt(1, 140, "Speed: %.2f, Error: %.2f", speed, error);
  }

  stopBase();
  task::sleep(15);
}

// Positive angles go left (make inches negative to go right)
void encoderTurn(double inches, int speed) {
  double turnDeg = inchesToTicks(inches);

  LFBASE.setVelocity(speed, pct);
  LBBASE.setVelocity(speed, pct);
  LFBASE.startRotateFor(directionType::rev, turnDeg, deg);
  LBBASE.startRotateFor(directionType::rev, turnDeg, deg);

  RFBASE.setVelocity(speed, pct);
  RBBASE.setVelocity(speed, pct);
  RFBASE.startRotateFor(fwd, turnDeg, deg);
  RBBASE.rotateFor(fwd, turnDeg, deg);
  stopBase();
}

void backwardInches(double inches, int maxSpeed) {
  double ticks = -inchesToTicks(inches);
  // double initialAngle = Gyro1.value(deg);

  double targetRange =
      2; // Distance from the desired distance the robot has to be to stop.
  double error = ticks; // Distance from the desired range
  double progress = 0;  // Ticks the motors has turned already
  double minSpeed = 2;  // Lowest speed the motors will go; Turning is generally
                        // more precise and accurate when lower.
  double accelRate = 30;
  double deaccelRate = 9;

  double speed = 0; // Actual speed value of the motors

  maxSpeed = keepInRange(maxSpeed, 0, 100);
  clearEncoders();

  // Brain.Screen.printAt(1, 120, "Desired distance: %.2f, Progress: %.2f",
  // ticksToInches(ticks), ticksToInches(progress)); Brain.Screen.printAt(1, 140,
  // "Speed: %.2f, Error: %.2f", speed, error);

  while (error < targetRange) {

    progress = (LFBASE.rotation(deg) + RFBASE.rotation(deg) +
                LBBASE.rotation(deg) + RBBASE.rotation(deg)) /
               4; // Average value of all motors
    error = ticks - progress;

    // First half: accelerate to max; Second half, deccelerate to min
    if (error <= ticks / 2) {
      speed = (((progress / ticks)) * maxSpeed * accelRate) + minSpeed;
    } else {
      speed = ((1 - (progress / ticks)) * maxSpeed * deaccelRate) + minSpeed;
    }
    speed = keepInRange(speed, minSpeed, maxSpeed);

    RFBASE.spin(directionType::rev, speed /*- correctionR*/, pct);
    LFBASE.spin(directionType::rev, speed /*- correctionL*/, pct);
    RBBASE.spin(directionType::rev, speed /*- correctionR*/, pct);
    LBBASE.spin(directionType::rev, speed /*- correctionL*/, pct);

    // Brain.Screen.printAt(1, 120, "Desired distance: %.2f, Progress: %.2f",
    // ticksToInches(ticks), ticksToInches(progress)); Brain.Screen.printAt(1,
    // 140, "Speed: %.2f, Error: %.2f", speed, error);
  }
  stopBase();
  task::sleep(15);
}

// If time (ms) in function is > maxTime, exit the loop
void backwardInchesTimed(double inches, int maxSpeed, int maxTimeMs) {
  double ticks = -inchesToTicks(inches);
  // double initialAngle = Gyro1.value(deg);

  double targetRange =
      2; // Distance from the desired distance the robot has to be to stop.
  double error = ticks; // Distance from the desired range
  double progress;      // Ticks the motors has turned already
  double minSpeed =
      2; // Lowest speed the motors will go; Turning is more precise when lower.
  double accelRate = 30;
  double deaccelRate = 12;

  double speed; // Actual speed value of the motors

  maxSpeed = keepInRange(maxSpeed, 0, 100);
  clearEncoders();
  timer Timer;
  Timer.clear();

  while (error < targetRange && Timer.time(msec) <= maxTimeMs) {

    progress = (LFBASE.rotation(deg) + RFBASE.rotation(deg) +
                LBBASE.rotation(deg) + RBBASE.rotation(deg)) /
               4; // Average value of all motors
    error = ticks - progress;

    // First half: accelerate to max; Second half, deccelerate to min
    if (error <= ticks / 2) {
      speed = (((progress / ticks)) * maxSpeed * accelRate) + minSpeed;
    } else {
      speed = ((1 - (progress / ticks)) * maxSpeed * deaccelRate) + minSpeed;
    }
    speed = keepInRange(speed, minSpeed, maxSpeed);

    RFBASE.spin(reverse, speed /*+ correctionR*/, pct);
    LFBASE.spin(reverse, speed /*+ correctionL*/, pct);
    RBBASE.spin(reverse, speed /*+ correctionR*/, pct);
    LBBASE.spin(reverse, speed /* + *correctionL */, pct);

    // Brain.Screen.printAt(1, 120, "Desired distance: %.2f, Progress: %.2f",
    // ticksToInches(ticks), ticksToInches(progress));
    // Brain.Screen.printAt(1, 140, "Speed: %.2f, Error: %.2f", speed, error);
  }

  stopBase();
  task::sleep(15);
}

// Absolute turn in degrees
void gyroTurn(double targetAngle, int maxSpeed) {

  double initialAngle = getHeading();
  // positive degrees == right; negative degrees == left // actually this is
  // wrong cuz counter-clockwise (left) is positive but go off
  double relativeAngle = targetAngle - getHeading();

  double targetRange = 2; // Distance from the desired angle that is allowed
  double error = relativeAngle; // Distance from the desired range
  double progress;              // Degrees the robot has turned already
  double minSpeed = 3; // Lowest speed the motors will go; Turning is generally
                       // more precise when lower, but slower.
  double deaccelRate = 3; // 2.4

  double speed; // Actual speed value of the motors

  maxSpeed = keepInRange(maxSpeed, 0, 100);

  while (fabs(error) > targetRange) {
    progress = getHeading() - initialAngle;

    if (relativeAngle < 0)
      error = -relativeAngle + progress;
    else
      error = progress - relativeAngle;

    // Speed starts at maximum and approaches minimum as the gyro value
    // approaches the desired angle. It deccelerates for precision.
    speed = /*fabs*/ (
        ((1 - (progress / relativeAngle)) * maxSpeed * deaccelRate) + minSpeed);
    // Speed is absolute value so that it can be kept in range properly
    // Direction is based on negative angle's sign instead
    speed = keepInRange(speed, 0, maxSpeed);
    if (relativeAngle < 0) {
      RFBASE.spin(fwd, speed, pct);
      LFBASE.spin(directionType::rev, speed, pct);
      RBBASE.spin(fwd, speed, pct);
      LBBASE.spin(directionType::rev, speed, pct);
    } else {
      LFBASE.spin(fwd, speed, pct);
      RFBASE.spin(directionType::rev, speed, pct);
      LBBASE.spin(fwd, speed, pct);
      RBBASE.spin(directionType::rev, speed, pct);
    }

    controllerPrim.Screen.clearScreen();
    task::sleep(75);
    controllerPrim.Screen.setCursor(1, 1);
    task::sleep(75);
    controllerPrim.Screen.print("%f", getHeading());

    // Brain.Screen.printAt(1, 120, "Desired angle: %.2f, Turn degrees: %.2f",
    // degrees, turnDegrees);
    // Brain.Screen.printAt(1, 140, "Speed: %.2f, Error: %.2f", speed, error);
    // Brain.Screen.printAt(1, 180, "Gyro: %.2f", Gyro1.value(deg));
  }

  RFBASE.stop(brakeType::hold);
  LFBASE.stop(brakeType::hold);
  RBBASE.stop(brakeType::hold);
  LBBASE.stop(brakeType::hold);
  task::sleep(15);
}

void forwardPID(float targetInches, float maxPower, float msTimeout) {
  float change = 1.34;
  float Kp = 0.3031; // you need to tune these value manually. 0.305
  float Ki = 0.008;// //.03; // they are just arbitrary constants so you need to
                     //test     0.0145
  float Kd = 0.53231; //.18922175;    //until they work. 0.529

  float error =
      inchesToTicks(targetInches * change) - getBaseAvg(); // desired - actual
  float lastError;
  float integral;
  float derivative;

  float integralPowerLimit =
      40 / Ki; // little less than half power in pct (percent)
  float integralActiveZone = 15;
  // explaining integral active zone
  // area where proportion is effective is represented with >>/<<
  // area where proportion is not strong enough to move base rep with ----
  // 0 is the target where error is equal to 0
  // only want integral during ineffective zone to prevent windup of power
  //>>>>>>>>>>>>-----------0---------------<<<<<<<<<<<<<<<<

  float exitThreshold = 0.5; // Exit loop when error is less than this
  float finalPower;

  clearEncoders();
  Brain.resetTimer();

  while (fabs(error) > exitThreshold &&
         Brain.timer(vex::timeUnits::msec) < msTimeout) {
    // Brain.Screen.printAt(140, 85,"ROTATION: %.3f deg", getRotation());
    error = inchesToTicks(targetInches * change) - getBaseAvg();

    if (fabs(error) < integralActiveZone && error != 0) {
      integral = integral + error;
    } else {
      integral = 0;
    }
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;
    /*
     if (error == 0)
     {
       derivative = 0;
     }*/

    finalPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    finalPower = keepInRange(finalPower, -maxPower, maxPower);

    moveLeftBase(finalPower);
    moveRightBase(finalPower);
    Brain.Screen.printAt(140, 25, "P: %.2f, I: %.2f, D: %.2f", (Kp * error),
                         (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(
        140, 50, "Dist: %.2f, Error: %.2f", ticksToInches(getBaseAvg()),
        error - getBaseAvg());
    vex::task::sleep(40);
  }
  stopBase();
}

void backwardPID(float targetInches, float maxPower, float msTimeout) {
  float change = 1.2;
  float Kp = 0.303; // you need to tune these value manually. 0.305
  float Ki = 0.008;// //.03; // they are just arbitrary constants so you need to
                     //test     0.0145
  float Kd = 0.53232; //.18922175;    //until they work. 0.529

  float error =
      inchesToTicks(-targetInches * change) - getBaseAvg(); // desired - actual
  float lastError;
  float integral;
  float derivative;

  float integralPowerLimit =
      40 / Ki; // little less than half power in pct (percent)
  float integralActiveZone = 15;
  // explaining integral active zone
  // area where proportion is effective is represented with >>/<<
  // area where proportion is not strong enough to move base rep with ----
  // 0 is the target where error is equal to 0
  // only want integral during ineffective zone to prevent windup of power
  //>>>>>>>>>>>>-----------0---------------<<<<<<<<<<<<<<<<

  float exitThreshold = 0.5; // Exit loop when error is less than this
  float finalPower;

  clearEncoders();
  Brain.resetTimer();

  while (fabs(error) > exitThreshold &&
         Brain.timer(vex::timeUnits::msec) < msTimeout) {
    // Brain.Screen.printAt(140, 85,"ROTATION: %.3f deg", getRotation());
    error = inchesToTicks(-targetInches * change) - getBaseAvg();

    if (fabs(error) < integralActiveZone && error != 0) {
      integral = integral + error;
    } else {
      integral = 0;
    }
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;
    /*
     if (error == 0)
     {
       derivative = 0;
     }*/

    finalPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    finalPower = keepInRange(finalPower, -maxPower, maxPower);

    moveLeftBase(finalPower);
    moveRightBase(finalPower);
    Brain.Screen.printAt(140, 25, "P: %.2f, I: %.2f, D: %.2f", (Kp * error),
                         (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(140, 50, "Dist: %.2f, Error: %.2f",
                         ticksToInches(getBaseAvg()), ticksToInches(error));
    vex::task::sleep(40);
  }
  stopBase();
}

void turnPID(float target, float maxPower, float msTimeout) {
  float change = 1.0;
  /*
    float Kp = 0.5105;   //getting to target
    float Ki = 0.00825; // increases speed (builds up over time) before: 0.008
    float Kd = 0.0111;*/    //slow down

  /*float Kp = 0.6;    // 0.508497;
  float Ki = 0.0001; // 0.007;
  float Kd = 0.0504; // 051;//.09;*/

  // new constants for Kalahari
  float Kp = 0.48;       // 0.508497;
  float Ki = 0.005;//75;//19;      // 11; //0.007;
  float Kd = 0.499;//3; // 0.0504;//051;//.09;

  float error = (target * change) - getRotation();
  float lastError;
  float integral;
  float derivative;

  float integralPowerLimit =
      40 / Ki;                   // little less than half power in pct (percent)
  float integralActiveZone = 15; // degrees b/c its a gyro turn doesnt use ticks
  // explaining integral active zone
  // area where proportion is effective is represented with >>/<<
  // area where proportion is not strong enough to move base rep with ----
  // 0 is the target where error is equal to 0
  // only want integral during ineffective zone to prevent windup of power
  //>>>>>>>>>>>>-----------0---------------<<<<<<<<<<<<<<<<

  float exitThreshold = 0.75; // Exit loop when error is less than this
  float finalPower;

  clearEncoders();
  Brain.resetTimer();

  while (fabs(error) > exitThreshold &&
         Brain.timer(vex::timeUnits::msec) < msTimeout) {
    Brain.Screen.printAt(140, 95, "ROTATION: %.3f deg", getRotation());
    error = (target * change) - getRotation();

    if (fabs(error) < integralActiveZone && error != 0) {
      integral = integral + error;
    } else {
      integral = 0;
    }
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;
    /*
     if (error == 0)
     {
       derivative = 0;
     }*/

    finalPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    finalPower = keepInRange(finalPower, -maxPower, maxPower);

    turnBase(finalPower);
    // Brain.Screen.printAt(140, 25,"P: %.2f, I: %.2f, D: %.2f", (Kp * error),
    // (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(140, 65, "P: %.2f, I: %.2f, D: %.2f", (Kp * error),
                         (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(140, 45, "INERTIAL: %.2f", Inertial.value());
    Brain.Screen.printAt(140, 25, "INERTIAL2: %.2f", getRotation());

    vex::task::sleep(40);
  }
  stopBase();
}

/*
// Stay moving in the same direction.
int driveStraight()
{
    while( true )
{
double Error = fabs( desiredDegree - Gyro1.value(rotationUnits::deg));
double Derivative = KPC * exp(-KDC * Error); //Sets the Derivative equal to
exponential of the error double  Proportion = 200.0 / (1 + Derivative)-4; //sets
Proportion to the desired speed over the derivative cSpeed = (Proportion*1.1);
// Make turning speed a little faster than proportion if (Error != 0) { // If
error is not completely eliminated, make sure speed is at least 1 (or -1) to
ensure speed is enough to overcome friction/weight of robot. cSpeed +=1 ;
    }
        //Brain.Screen.printAt(140, 55, "Gyro: %.3f",
Gyro1.value(rotationUnits::deg)); if( Gyro1.value(rotationUnits::deg) <
desiredDegree )
    {
correctionL = cSpeed;
correctionR = -cSpeed;
}
else if ( Gyro1.value(rotationUnits::deg) > desiredDegree )
    {
correctionL = -cSpeed;
correctionR = cSpeed;
}
else
    {
correctionL = 0;
correctionR = 0;
}

        task::sleep(20);
        //Brain.Screen.printAt(0, 120, "Gyro: %.2f",
Gyro1.value(rotationUnits::deg));
}
    return(0);
}
*/

/*
// Stay moving in the same direction and on the line.
int driveStraightLine()
{
  while( true )
{
double turnError = fabs( desiredDegree - Gyro1.value(rotationUnits::deg) );
    double xError = fabs ( desiredX - xTracker.rotation(rotationUnits::deg) );

    // Not sure if this is the way to do it, but the error is the average of the
turn error and x error to account for both angle and horizontal position. double
Error = (turnError + xError)/2;

double Derivative = KPC * exp(-KDC * Error); //Sets the Derivative equal to
exponential of the error double  Proportion = 200.0 / (1 + Derivative)-4; //sets
Proportion to the desired speed over the derivative cSpeed = Proportion*1.34;
        //Brain.Screen.printAt(140, 55, "Gyro: %.3f",
Gyro1.value(rotationUnits::deg)); if( Gyro1.value(rotationUnits::deg) <
desiredDegree )
    {
correctionL = cSpeed;
correctionR = -cSpeed;
}
else if ( Gyro1.value(rotationUnits::deg) > desiredDegree )
    {
correctionL = -cSpeed;
correctionR = cSpeed;
}
else
    {
correctionL = 0;
correctionR = 0;
}
        task::sleep(20);
        //Brain.Screen.printAt(0, 120, "Gyro: %.2f",
Gyro1.value(rotationUnits::deg));
}
    return(0);
}
*/

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                           Misc Movement Functions                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

/*
void armToDeg(double deg, int maxSpeed) {

  const double initialError = deg-ArmPot.value(rotationUnits::deg); // Error =
desired - actual mesasurement.

  double error = initialError;
  const double acceptableError = 4; // How far off from intended degree value
that it's ok to stop at

  const int minSpeed = 5;
  const double deaccelRate = 3;
  Brain.Screen.printAt(0, 60, "%d", fabs(error) > acceptableError);
  Brain.Screen.printAt(25, 60, "%d, %d", error, acceptableError);

  while (fabs(error) > acceptableError) {
    error = deg-ArmPot.value(rotationUnits::deg); // Error = desired - actual
mesasurement. double speed = fabs((error/initialError)*maxSpeed*deaccelRate); //
P-controller (proportional to error)

    // Limit the speed at maxSpeed because otherwise deaccel rate would increase
speed past the max.
    // Keep the speed up to at least minSpeed, because otherwise steady-state
error would become impossible to overcome. speed = keepInRange(speed, minSpeed,
maxSpeed);

    Brain.Screen.printAt(0, 20, "%.2f, %.2f", ArmPot.value(rotationUnits::deg),
ARM.rotation(rotationUnits::deg)); Brain.Screen.printAt(0, 40, "Error: %.2f,
Speed: %.2f", error, speed);

    if (error < 0) {
      ARM.spin(fwd, fabs(speed), velocityUnits::pct);
    } else {
      ARM.spin(directionType::rev, fabs(speed), velocityUnits::pct);
    }

  }

  ARM.stop(brakeType::hold);

}

*/
/*

void armToDegEncoder(double deg, int maxSpeed) {

  const double initialError = deg-ARM.rotation(rotationUnits::deg); // Error =
desired - actual mesasurement.

  double error = initialError;
  const double acceptableError = 4; // How far off from intended degree value
that it's ok to stop at

  const int minSpeed = 5;
  const double deaccelRate = 3;
  Brain.Screen.printAt(0, 60, "%d", fabs(error) > acceptableError);
  Brain.Screen.printAt(25, 60, "%d, %d", error, acceptableError);

  while (fabs(error) > acceptableError) {
    error = deg-ARM.rotation(rotationUnits::deg); // Error = desired - actual
mesasurement. double speed = fabs((error/initialError)*maxSpeed*deaccelRate); //
P-controller (proportional to error)

    // Limit the speed at maxSpeed because otherwise deaccel rate would increase
speed past the max.
    // Keep the speed up to at least minSpeed, because otherwise steady-state
error would become impossible to overcome. speed = keepInRange(speed, minSpeed,
maxSpeed);

    Brain.Screen.printAt(0, 20, "%.2f", ARM.rotation(rotationUnits::deg));
    Brain.Screen.printAt(0, 40, "Error: %.2f, Speed: %.2f", error, speed);

    if (error < 0) {
      ARM.spin(fwd, fabs(speed), velocityUnits::pct);
    } else {
      ARM.spin(directionType::rev, fabs(speed), velocityUnits::pct);
    }

  }

  ARM.stop(brakeType::hold);

}
*/

void intake(double speed) {
  LINTAKE.spin(directionType::fwd, speed, velocityUnits::pct);
  RINTAKE.spin(directionType::fwd, speed, velocityUnits::pct);
}

void outtake(double speed) {
  LINTAKE.spin(directionType::rev, speed, velocityUnits::pct);
  RINTAKE.spin(directionType::rev, speed, velocityUnits::pct);
}

void intakeStop() {
  LINTAKE.stop(brakeType::coast);
  RINTAKE.stop(brakeType::coast);
}

void rollersIn(double speed) {
  BROLLER.spin(directionType::fwd, speed, velocityUnits::pct);
  TROLLER.spin(directionType::fwd, speed, velocityUnits::pct);
}

void rollersOutBottom(double speed) {
  BROLLER.spin(directionType::rev, speed, velocityUnits::pct);
}

void rollersOut(double speed) {
  BROLLER.spin(directionType::rev, speed, velocityUnits::pct);
  TROLLER.spin(directionType::rev, speed, velocityUnits::pct);
}

// why.
void poopIn(double speed) {
  BROLLER.spin(directionType::fwd, speed, velocityUnits::pct);
  TROLLER.spin(directionType::rev, speed, velocityUnits::pct);
}

void poopOut(double speed) {
  BROLLER.spin(directionType::rev, speed, velocityUnits::pct);
  TROLLER.spin(directionType::fwd, speed, velocityUnits::pct);
}

void rollersStop() {
  BROLLER.stop(brakeType::coast);
  TROLLER.stop(brakeType::coast);
}

/* Switches to picking up ball after certain percent of distance has been
 * reached */
void forwardPIDIntake(float targetInches, float maxPower, float msTimeout,
                      float powers) {
  float change = 1.34;
  float Kp = 0.305;  // you need to tune these value manually. 0.305
  float Ki = 0.0144; //.03; // they are just arbitrary constants so you need to
                     //test     0.0145
  float Kd = 0.5292; //.18922175;    //until they work. 0.529

  float error =
      inchesToTicks(targetInches * change) - getBaseAvg(); // desired - actual
  float initialError = error;
  float lastError;
  float integral;
  float derivative;

  bool intakeStarted = false;
  float integralPowerLimit =
      40 / Ki; // little less than half power in pct (percent)
  float integralActiveZone = 15;
  // explaining integral active zone
  // area where proportion is effective is represented with >>/<<
  // area where proportion is not strong enough to move base rep with ----
  // 0 is the target where error is equal to 0
  // only want integral during ineffective zone to prevent windup of power
  //>>>>>>>>>>>>-----------0---------------<<<<<<<<<<<<<<<<

  float exitThreshold = 0.5; // Exit loop when error is less than this
  float finalPower;

  clearEncoders();
  Brain.resetTimer();

  while (fabs(error) > exitThreshold &&
         Brain.timer(vex::timeUnits::msec) < msTimeout) {
    // Brain.Screen.printAt(140, 85,"ROTATION: %.3f deg", getRotation());
    error = inchesToTicks(targetInches * change) - getBaseAvg();

    if (fabs(error) < integralActiveZone && error != 0) {
      integral = integral + error;
    } else {
      integral = 0;
    }
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;
    /*
     if (error == 0)
     {
       derivative = 0;
     }*/

    finalPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    finalPower = keepInRange(finalPower, -maxPower, maxPower);

    moveLeftBase(finalPower);
    moveRightBase(finalPower);

    if (fabs(error) < fabs(initialError) * 0.666666 && !intakeStarted) {
      rollersOut(powers);
      intakeStarted = true;
    }

    Brain.Screen.printAt(140, 25, "P: %.2f, I: %.2f, D: %.2f", (Kp * error),
                         (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(
        140, 50, "Dist: %.2f, Error: %.2f", ticksToInches(getBaseAvg()),
        ticksToInches(inchesToTicks(targetInches) - getBaseAvg()));
    vex::task::sleep(40);
  }
  stopBase();
}

/*
void encoderTurnLeft(double inches, int speed) {
  double turnDeg = inchesToTicks(inches);

  LFBASE.setVelocity(speed, pct);
  LBBASE.setVelocity(speed, pct);
  LFBASE.startRotateFor(directionType::rev, turnDeg, deg);
  LBBASE.startRotateFor(directionType::rev, turnDeg, deg);
  RFBASE.setVelocity(-speed, pct);
  RBBASE.setVelocity(-speed, pct);
  RFBASE.startRotateFor(fwd, turnDeg, deg);
  RBBASE.startRotateFor(fwd, turnDeg, deg);
  stopBase();
}*/

/// INDEX TESTING
/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Stop top roller after ball is shot from loaded position after some amount of
// time.
int stopScoring() {
  task::sleep(190);
  TROLLER.stop();

  return 0;
}

// Stop top roller after ball is shot from loaded position after some amount of
// time.
int stopDescoring() {
  intakeStop();
  task::sleep(100);
  BROLLER.stop();
  // task::sleep(300);
  // outtake(400);
  // task::sleep(100);
  return 0;
}

// TESTING
int load1() {
  rollersOut(65);

  // Wait until top indexer sensor detects ball in loaded position or until it
  // times out. This makes the ball always stop in the same place.

  while (!tBallDetected()) // && Timer.time(msec) < timeOut)
  {
  }
  rollersStop();

  return 0;
}

int load2() {
  rollersOut(60);

  // Wait until top indexer sensor detects ball in loaded position or until it
  // times out. This makes the ball always stop in the same place.
  int bPasses = 0;
  bool bLast = bBallDetected();
  bool tLast = tBallDetected();
  bool loaded1 = false;

  while (bPasses < 2) // && Timer.time(msec) < timeOut)
  {
    bool bDetected = bBallDetected();
    bool tDetected = tBallDetected();

    if (bDetected && !bLast) {
      bPasses++;
    }

    if (tBallDetected() && !loaded1) {
      loaded1 = true;
      TROLLER.stop();
    }

    bLast = bDetected;
    tLast = tDetected;
  }
  task::sleep(150);
  BROLLER.stop();
  intakeStop();

  return 0;
}

void load1Async() { task l1(load1); }

void load2Async() { task l2(load2); }

// Score our balls and pick up the ones in the tower.
// Make sure ball is in loaded position before calling.
void cycle(int ourBalls, int towerBalls, int timeOutMs) {
  // How many times a ball has passed over the sensor
  int tPassed = 0; // Counts after ball leaves
  int bPasses = 0; // Counts when ball first appears
  bool tLast =
      tBallDetected(); // If line sensor detected ball last iteration of loop.
  bool bLast = bBallDetected();

  bool scoring = true;
  bool descoring = false;

  // int rollerSpeed = 90;
  TROLLER.spin(directionType::rev, 65, pct);
  BROLLER.spin(directionType::rev, 55, pct);
  intakeStop();

  timer Timer;
  Timer.clear();
  Brain.Screen.clearScreen();

  // Loop until we finish intaking.
  while ((scoring || descoring) && Timer.time(msec) <= timeOutMs) {
    bool tDetected = tBallDetected();
    bool bDetected = bBallDetected();

    // Top is counted when ball exits, bottom is counted when ball enters
    if (!tDetected && tLast) {
      tPassed++;
    }
    if (bDetected && !bLast) {
      bPasses++;
    }

    // endScoringCallback will stop top roller some time after our balls are
    // scored to avoid scoring opposing color
    if (tPassed >= ourBalls && scoring) {
      scoring = false;
      task endScoring(stopScoring);
    }

    // If not enough tower balls have been descored, then continue descoring
    if (bPasses < towerBalls) {
      // Intake opposing balls after scoring so we don't hold too many at once.
      if (tPassed >= 1 && !descoring) {
        intake(100);
        descoring = true;
      }
    } else // if bPasses >= towerBalls
    {
      descoring = false;
      task endDescoring(stopDescoring);
    }

    tLast = tDetected;
    bLast = bDetected;

    Brain.Screen.printAt(
        0, 20, "T BALL: %d, TROLLER: %d, tDetected: %d, tPassed: %d",
        tBallDetected(), TROLLER.isSpinning(), tDetected, tPassed);
    Brain.Screen.printAt(
        0, 40, "B BALL: %d, BROLLER: %d, bDetected: %d, bPasses: %d",
        bBallDetected, BROLLER.isSpinning(), bDetected, bPasses);
    Brain.Screen.printAt(0, 60, "INTAKE: %d, Scoring: %d, Descoring: %d",
                         LINTAKE.isSpinning(), scoring, descoring);
  }

  // task endScoring(stopScoring);
  stopScoring();
  BROLLER.stop();
  intakeStop();

  Brain.Screen.printAt(0, 100, "EXITED");
  task::sleep(15);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////// /////////////////////////////
/////////////////////                      KALAHARI VOIDS
////////////////////////////////
///////////////////// /////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Score our balls and pick up the ones in the tower.
 * Make sure ball is in loaded position before calling.
 */
/* Score our balls and pick up the ones in the tower.
 * Make sure ball is in loaded position before calling.
 */
void cyclePoop(float timeoutMs) {
  timer Timer;
  Timer.clear();

  // only pick up first ball
  intake(100);
  poopOut(100);
  // poop first ball
  while (!bBallDetected() && Timer.time(msec) < timeoutMs) {
  }
  intakeStop();
  while (!poopBallDetected() && Timer.time(msec) < timeoutMs) {
  }

  // only intake second ball
  intake(100);
  rollersOut(100);
  while (!bBallDetected() && Timer.time(msec) < timeoutMs) {
  }
  intakeStop();
  //backwardInchesTimed(2, 14, 500);
  //forwardInchesTimed(3, 14, 500);
  // score second ball
  while (!tBallDetected() && Timer.time(msec) < timeoutMs) {
  }
  task::sleep(170);

  // intake third ball
  intake(30);
  poopOut(30);
  while (!bBallDetected() && Timer.time(msec) < timeoutMs) {
  }
  task::sleep(40);
  outtake(65);
  //backwardInchesTimed(2, 14, 500);
  //forwardInchesTimed(4.5, 14, 500);

  // poop third ball
  while (!poopBallDetected() && Timer.time(msec) < timeoutMs) {
  }
  task::sleep(240);
  intakeStop();
  rollersStop();
}

void cycleCorner(float timeoutMs) // up to date as of 5-25-21
{ 
  timer Timer;
  Timer.clear();
  // first ball already intaken
  TROLLER.spin(directionType::rev, 85, pct);
  BROLLER.spin(directionType::rev, 85, pct);
  //rollersOut(75);

  Brain.Screen.printAt(20, 20, "1");
  while (!tBallDetected() && Timer.time(msec) < timeoutMs) {
  } // wait until 1st ball scored
  task::sleep(190);

  intake(100); // intake 2nd ball
  Brain.Screen.printAt(20, 20, "2");
  while (!bBallDetected() && Timer.time(msec) < timeoutMs) {
  } // wait until 2nd ball intaken
  intakeStop();
  Brain.Screen.printAt(20, 20, "3");
  while (!tBallDetected() && Timer.time(msec) < timeoutMs) {
  } // wait until 2nd ball scored
  task::sleep(160);

  intake(90); // intake 3rd ball
  task::sleep(30);
  rollersStop(); // stop rollers so last ball is not scored
  Brain.Screen.printAt(20, 20, "4");
  while (!bBallDetected() && Timer.time(msec) < timeoutMs) {
    } // wait until 3rd ball intaken

  Brain.Screen.printAt(20, 20, "5");
  //task::sleep(20);
  outtake(60); // outtake so robot doesn't intake our scored balls
  task::sleep(400);
  rollersStop();
  intakeStop();
}

/*
int descore1(int timeOutMs)
{
  timer Timer;
  intake(100);
  Timer.clear();
  while(!bBallDetected())
  {

  }
  return 0;
}

int descore(int numDescoring, int timeOutMs)
{
  int numPasses = 0;
  bool lastDetected = bBallDetected();
  timer Timer;

  intake(100);
  Timer.clear();

  // Loop until we finish descoring all balls or function times out
  while (numPasses < numDescoring && Timer.time(msec) <= timeOutMs)
  {
    bool detected = bBallDetected();
    // When ball is first detected, increment passes
    if (detected && !lastDetected)
    {
      numPasses++;
    }
    lastDetected = detected;
  }

  intakeStop();

  return 0;
}*/

/* Descore without blocking execution in the scope the function was called in */
/*void descoreAsync()
{
  task descore;
}*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Gyro1.startCalibration(3000);

  // RAIL.resetRotation();
  //task::sleep(3000);
  // selectionMenu();
  // Inertial.setHeading(0, deg);
  /*Brain.Screen.printAt(140, 225,"Callibrated");
  Brain.Screen.printAt(140, 45,"INERTIAL: %.2f", Inertial.value());
  Brain.Screen.printAt(140, 25,"INERTIAL2: %.2f", getRotation());*/

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  ///////  DEMO AUTON  ///////////////////////////////
  /*
  forwardInches(12, 85);
  backwardInches(12, 85);
  intake(75);
  rollersIn(75);
  task::sleep(500);
  intakeStop();
  rollersStop();
*/

  //////  FLIP OUT (could put into method) /////////////////////////////////////
  /*BROLLER.spin(directionType::rev, 100, pct); // flip out hood
  task::sleep(250);
  BROLLER.stop();
  /////////
  intake(100);
  forwardInches(10, 75);
  intakeStop();
  //gyroTurn(45, 65);
  //gyroTurn(105, 40);
  encoderTurn(-10, 40);
  forwardInches(18, 75);
  rollersOut(65);
  intake(25);
  task::sleep(4000);
  rollersStop();
  intakeStop();*/

  //////  FLIP OUT (could put into method) /////////////////////////////////////
  BROLLER.spin(directionType::rev, 100, pct); // flip out hood
  task::sleep(250);
  BROLLER.stop();
  /////////
  intake(100);
  forwardInches(10, 45);
  intakeStop();
  // gyroTurn(45, 65);
  // gyroTurn(105, 40);
  encoderTurn(-9.6, 20); // score 1st tower
  forwardInches(18, 75);
  rollersOut(75);
  // intake(25);
  task::sleep(850);
  rollersStop();
  // intakeStop();
  task::sleep(800);

  backwardInches(3.5, 50);
  encoderTurn(10, 40);

  backwardInchesTimed(7.5, 25, 1000); // crash into wall first time
  intake(50);
  rollersIn(30);
  task::sleep(900);
  intakeStop();
  rollersStop();

  forwardInches(30, 60); // get third ball
  intake(80);
  forwardInches(11, 20);
  rollersOut(15);
  task::sleep(1000);
  rollersStop();
  intakeStop();
  //
  // task::sleep(300);

  encoderTurn(-6.5, 25); // score second tower
  forwardInches(4, 30);
  rollersOut(100);
  // intake(80);
  task::sleep(750);
  rollersStop();
  task::sleep(800);
  // intakeStop();

  rollersIn(30);
  task::sleep(500);
  rollersStop();
  backwardInches(6, 40); // back up and bring ball down a little

  encoderTurn(5.5, 40); // score third tower
  forwardInchesTimed(40, 60, 1000);
  rollersOut(80);
  // intake(30);
  task::sleep(1300);
  rollersStop();
  // intakeStop();

  backwardInches(4, 10); // descore corner tower
  /*
  intake(80);
  poopOut(80);
  forwardInchesTimed(4.5, 20, 1000);
  backwardInches(4, 10);
  forwardInchesTimed(4.5, 20, 1000);
  backwardInches(4, 10);
  intakeStop();
  rollersStop();
  */
  // forwardInchesTimed(4.5, 20, 1000);
  // backwardInches(4, 10);
  // forwardInchesTimed(4.5, 20, 1000);

  /*
  backwardInches(4, 25);
  //forwardInches(30,70);    //get third ball
  encoderTurn(11, 25);
  backwardInchesTimed(6, 35, 2000);
  intake(80);
  forwardInchesTimed(11, 20, 3000);
  intakeStop();
  forwardInches(20, 50);


  encoderTurn(-6.5, 25);   //score fourth tower
  forwardInches(4, 30);
  rollersOut(80);
  intake(80);
  task::sleep(750);
  rollersStop();
  task::sleep(800);
  intakeStop();

  backwardInches(3, 35);
  encoderTurn(-6.5, 25);
  forwardInches(12, 35);
  intake(80);
  forwardInches(21, 20);
  intakeStop();
  forwardInches(12, 50);

  rollersOut(100);
  task::sleep(2000);
  rollersStop();

*/
}

/*
void compAuto()
{
  //////  FLIP OUT (could put into method) /////////////////////////////////////
  BROLLER.spin(directionType::rev, 100, pct); // flip out hood
  task::sleep(250);
  BROLLER.stop();
  /////////
  intake(100);
  forwardInches(10, 75);
  intakeStop();
  //gyroTurn(45, 65);
  //gyroTurn(105, 40);
  encoderTurn(-10, 40);
  forwardInches(18, 75);
  rollersOut(65);
  intake(25);
  task::sleep(2000);
  rollersStop();
  intakeStop();
  encoderTurn(-50, 40);
  forwardInches(-5,40);
}
*/

void triangleAuton() {
  //////  FLIP OUT (could put into method) /////////////////////////////////////
  BROLLER.spin(directionType::rev, 100, pct); // flip out hood
  task::sleep(250);
  BROLLER.stop();
  /////////
  intake(100);
  forwardInches(10, 45);
  intakeStop();

  encoderTurn(-9.6, 20); // score 1st tower
  forwardInches(18, 75);
  rollersOut(75);
  task::sleep(700);
  rollersStop();
  task::sleep(800);

  backwardInches(4, 50);
  encoderTurn(10, 40);

  backwardInchesTimed(6.5, 25, 1000); // crash into wall first time
  intake(50);
  rollersIn(30);
  task::sleep(900);
  intakeStop();
  rollersStop();

  forwardInches(30, 60); // get second ball
  intake(80);
  forwardInches(11, 20);
  rollersOut(15);
  task::sleep(1000);
  rollersStop();
  intakeStop();

  encoderTurn(-6.5, 25); // score second tower
  forwardInches(4.25, 30);
  rollersOut(100);
  task::sleep(800);
  rollersStop();

  backwardInches(6.25, 40); // back up  and intake ball
  intake(50);
  task::sleep(750);
  intakeStop();

  encoderTurn(5, 40); // turn towards third tower
  forwardInchesTimed(40, 60, 1000);
  rollersOut(80);
  task::sleep(1300);
  rollersStop();

  ///////////////  ADDED AFTER FIRST COMP (GAINESVILLE)
  //////////////////////////////////////////////////////////////////

  backwardInches(7, 50); // back up from third tower
  encoderTurn(10, 20);
  backwardInchesTimed(15, 25, 1000); // crash into wall 2nd time

  // get fourth ball
  forwardInches(8, 65);
  intake(80);
  forwardInches(6, 25);
  rollersOut(15);
  task::sleep(1000);
  rollersStop();
  intakeStop();
  forwardInches(25.5, 65);

  // score 2nd tower on middle right (blue)
  encoderTurn(-6.5, 25);
  forwardInches(4, 30);
  rollersOut(100);
  task::sleep(1000);

  rollersIn(30);
  task::sleep(500);
  rollersStop();
  backwardInches(6, 40); // back up and bring ball down a little

  // get fifth ball
  encoderTurn(5.5, 25);
  forwardInches(15, 65);
  intake(80);
  forwardInches(8, 25);
  rollersOut(15);
  task::sleep(1000);
  rollersStop();
  intakeStop();
  forwardInches(13, 65);

  // score 5th goal
  encoderTurn(-4, 25);
  forwardInches(8, 30);
  rollersOut(100);
  task::sleep(1000);
  rollersStop();
}

void pidTester() {
  //forwardPID(12, 35, 10000);
  //forwardPID(20, 50, 10000);
   //forwardPID(30, 40, 10000);
   
   //forwardPID(20, 40, 10000);
  //turnPID(64, 18, 11000); ////
  //task::sleep(50);
  //turnPID(193.5, 18, 14000); //
  //forwardPID(20, 50, 10000);
  //task::sleep(50);
  //turnPID(148, 11, 1400);    ///  

   //forwardPID(60, 65, 10000);
  turnPID(-90, 20, 10000);
  //turnPID(-45, 12, 1300);
  // turnPID(-90, 18, 10000);
  // turnPID(45, 11, 10000);
  // turnPID(-90, 18, 10000);
   //turnPID(15, 8, 10000);
  //forwardPID(20, 50, 10000);
}


void indexTester() {
  task::sleep(750);
  intake(100);
rollersOut(100);

  /*
  intake(100);
  forwardInchesTimed(3, 25, 2000);
  //loadBallTop();
  //loadBallTopAsync();

  forwardInchesTimed(8, 25, 1500);
  intakeStop();*/

  // cycle(1, 1, 40000);
  /* task::sleep(150);
   backwardInchesTimed(5, 25, 1500);
 rollersOut(100);
 descoreAsync2();*/
  // descore(2, 5000);
  // task::sleep(600);
  // rollersStop();

  cycleCorner(3000000);
  // cyclePoop();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void kalahariHomeRow() {

  // tower 6 

  // flip out
  outtake(100);
  task::sleep(400);
  intakeStop();

//////////////
// tower 9  

  // go to tower
  forwardPID(27, 50, 1400); // 26, 50, 1400
  
  // task::sleep(1000);
  turnPID(66, 17, 1250); ////

  // get first ball loaded
  intake(100);
  rollersOutBottom(100); // only moves bottom roller
  //rollersOut(40);
  forwardPID(9, 32, 700);
  intakeStop();
  forwardPID(5, 30, 500);
  // task::sleep(1000);
  //outtake(60);

  // cycle tower
  cycleCorner(1700);
  outtake(30);
  rollersIn(60);

  // back away
  backwardPID(29, 30, 1500); // back up is 29 for home row
  intakeStop();
  rollersStop();
  /*
//         UNCOMMENT
//////////////

// tower 3  
  // get to tower
  
  turnPID(197, 18, 1400); //
  
  forwardPID(60, 60, 2300); //
  turnPID(154, 12, 1300);    ///                               here

  // load ball
  intake(100);
  rollersOutBottom(100);     // only moves bottom roller
  forwardPID(10, 40, 1000); //
  intakeStop();


  // cycle tower
  forwardPID(6, 40, 600);
  
  cycleCorner(1800);//(2, 2, 2500);
  outtake(30);
  rollersIn(60);

  // back away
  backwardPID(17, 100, 1000);
  rollersStop();
  intakeStop();
  //
  */
  
}

void IFT(){


  // flip out
  outtake(100);
  task::sleep(500);
  intakeStop();

// go forward before turning to tower
  forwardPID(25.5, 59, 1200);
  // task::sleep(1000);
  turnPID(-75, 18, 1500);//turnPID(-75, 17, 1500); ////


  // get first ball loaded
  intake(90);
  rollersOutBottom(100); // only moves bottom roller
  forwardPID(31.25, 32, 1150);
  intakeStop();
  rollersStop();

  // task::sleep(1000);

  // cycle tower
  //forwardPID(5, 30, 500); //
  cycleCorner(2000);//(2, 2, 2500);
  
  //backwardPID(8, 10, 100); // back up a little bit
  outtake(30);
  rollersIn(60);

  // back away from corner
  backwardPID(37.5, 38, 2400); 

  intakeStop();
  rollersStop();

turnPID(9.25,20.5,2000);

intake(100);
forwardPID(29.5, 34, 2000);
//backwardInchesTimed(2,11,500);
cyclePoop(2000);

backwardInchesTimed(2,30,500);

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void worldsAuto() {
  //////  FLIP OUT (could put into method) /////////////////////////////////////
  /*BROLLER.spin(directionType::rev, 100, pct); // flip out hood
    task::sleep(250);
    BROLLER.stop();*/

  //////  FLIP OUT (could put into method) /////////////////////////////////////
  // UNCOMMENT
  intake(100);
  task::sleep(300);
  BROLLER.startRotateFor(directionType::rev, 100, deg);
  task::sleep(800);
  /////////
  load2Async();
  // turnPID(9.6, 60, 2000);
  forwardPID(69.7, 65, 3500); // pick up both balls along the way
  task::sleep(300);
  // UNCOMMENT
  backwardInchesTimed(9, 40, 2800);
  intakeStop();
  turnPID(71.5, 40, 2000);
  // intake(100);
  forwardPID(29.5, 50, 2200); // forward into the tower
  cycle(2, 2, 3750);

  rollersIn(35); // spit out balls
  outtake(35);
  backwardPID(18, 40, 2500);

  turnPID(225, 50, 3000);
  rollersStop();
  intakeStop();
  intake(100);
  // loadBallTopAsync();
  load1Async();
  forwardPID(75, 65, 7000); // go forward towards ball next to center tower
  turnPID(111, 40, 2500);

  forwardPID(41, 60, 2600); // approach tower 6
                            //  UNCOMMENT

  // Testing ONLY
  rotationStart =
      111; // Start with gyro value offset by a value because robot is
           // callibrated at a different angle when doing auton in sections

  cycle(1, 1, 3500);
  backwardInchesTimed(3.75, 20, 1250);
  turnPID(204, 50, 2000);
  intake(100); // UNCOMMENT
  poopOut(100);
  load2Async();
  forwardPID(45, 55, 5000);
  turnPID(220, 50, 2200);
  forwardPID(36.675, 60, 5500); // dont crash into wall
  backwardInchesTimed(9.5, 40, 1500);
  turnPID(159, 55, 2500);
  forwardInchesTimed(21, 40, 1500);
  cycle(2, 2, 3250); // cycle goal 3
  backwardInchesTimed(4, 20, 1000);
  turnPID(-25, 55, 4000);
  load2Async();
  forwardPID(50, 65, 3000);
  // turn towards ball in front of goal 2
  forwardPID(30, 50, 2000);

  /*
    load2Async(); // approach balls to score in goal 7
    forwardPID(30, 55, 3000);
    intakeStop();
    turnPID(111, 50, 1750);
    intake(100);
    forwardPID(48, 60, 4500);
    task::sleep(225);
    backwardInchesTimed(3.5, 35, 1250);*/
}

void Auton108() {

  // positive number is for a left turn
  // negative number is for a right turn
  // 86 is from back wall to tower
  // 118 is a 90 degree turn to the left

  // flip out hood
  TROLLER.spin(directionType::rev, 100, pct);
  intake(100);
  task::sleep(400);
  TROLLER.stop();

  ///////// collect the first ball and turn towards tower 4
  intake(100);
  rollersOut(50);
  forwardPID(28.5, 37, 1100);
  intakeStop();
  rollersStop();
  turnPID(30, 40, 900);

  // move towards tower 4 and turn to shoot
  forwardPID(36, 60, 2000); // used to be 36
  turnPID(118, 50, 1000);
  forwardInchesTimed(6, 30, 750);
  rollersOut(100);
  task::sleep(600);
  rollersStop();

  // back up and turn towards second ball
  backwardInchesTimed(6, 30, 1000);
  turnPID(28.5, 50, 1000);

  // drive towards second ball
  intake(100);
  rollersOut(25);
  forwardPID(47, 57, 2000);
  intakeStop();
  rollersStop();

  // turn towards tower one and shoot
  turnPID(70, 50, 900);
  forwardPID(28, 45, 1000);
  rollersOut(100);
  task::sleep(600);
  rollersStop();

  // back up from tower one and turn towards third ball on the way tower 2
  backwardPID(18, 35, 1000);
  turnPID(-64, 50, 1000);

  // move towards third ball on way to tower 2
  intake(100);
  rollersOut(45);
  forwardPID(48, 57, 2000);
  task::sleep(50);
  intakeStop();
  rollersStop();

  // turn towards tower 2 and score
  turnPID(29, 40, 1000);
  forwardInchesTimed(6, 30, 900);
  rollersOut(100);
  task::sleep(850);
  rollersStop();

  // back up from tower 2 and go for ball by tower three
  backwardInchesTimed(4.3, 30, 1000);
  turnPID(-63, 50, 1000);
  forwardPID(35, 55, 2000);

  // collect the ball by wall of tower three
  turnPID(29, 40, 1000);
  intake(100);
  rollersOut(40);
  forwardPID(12, 45, 1000); // dist changed from  18
  task::sleep(200);

  intakeStop();
  rollersStop();

  // back up from ball by wall at tower three turn and score
  backwardInchesTimed(5, 30, 1000);
  turnPID(-36, 50, 1000);
  forwardPID(26.7, 45, 1300); // dist changed from 26
  rollersOut(100);
  task::sleep(700);
  rollersStop();

  // back up from tower 3 and turn to face ball by tower 6
  backwardInchesTimed(7, 35, 1000);
  turnPID(-151, 45,
          1200); // used to be -153 but it was turning too much to the right

  // get ball towards tower 6
  intake(100);
  rollersOut(13);
  forwardPID(56, 60, 2400);
  intakeStop();
  rollersStop();

  // turn and score at tower 6
  turnPID(-55, 50, 1000);
  forwardInchesTimed(7, 30, 1000);
  rollersOut(100);
  task::sleep(800);
  rollersStop();

  backwardInchesTimed(5.5, 30, 1000); // back up from tower 6
  turnPID(-147, 50, 1000);            // face ball on the way to tower 9

  // drive towards tower 9 ball
  intake(100);
  rollersOut(30);
  forwardPID(50, 60, 2000);
  intakeStop();
  rollersStop();

  // turn towards tower 9 and shoot
  turnPID(-107, 50, 1000);
  forwardPID(27.5, 45, 1000);
  rollersOut(100);
  task::sleep(700);
  rollersStop();

  // back up and turn towards tower 8
  backwardInchesTimed(7.75, 35, 1000);
  turnPID(-244, 50, 1100);

  // go towards ball at tower 8
  intake(100);
  rollersOut(50);
  forwardPID(51, 60, 1800);
  task::sleep(200);
  intakeStop();
  rollersStop();

  // turn and shoot at tower 8
  turnPID(-150, 50, 1000);
  forwardInchesTimed(3, 30, 750);
  rollersOut(100);
  task::sleep(800);
  rollersStop();

  // back up and turn towards middle tower
  backwardInchesTimed(6.5, 30, 1000);
  turnPID(-335, 50, 1900); // needed to turn more right when it was at -330

  // pick up ball on the way to middle tower
  // pick up ball on the way to middle tower
  intake(100);
  rollersOut(50);
  forwardPID(20, 52, 1000);
  rollersStop();
  intakeStop();

  // descore middle tower and score one red
  outtake(100);
  forwardPID(28, 95, 1000);
  rollersOut(100);
  task::sleep(900);

  backwardInches(2.2, 50);
  forwardPID(5, 100, 600);
  task::sleep(100);
  backwardInches(2.2, 50);
  forwardInchesTimed(5, 100, 600);
  task::sleep(100);
  backwardInches(3, 50);
  intakeStop();
  rollersStop();
}

void Auton108Cycle() {

  // positive number is for a left turn
  // negative number is for a right turn
  // 86 is from back wall to tower
  // 118 is a 90 degree turn to the left

  // flip out hood
  TROLLER.spin(directionType::rev, 100, pct);
  intake(100);
  task::sleep(400);
  TROLLER.stop();

  ///////// collect the first ball and turn towards tower 4
  intake(100);
  rollersOut(50);
  forwardPID(28.5, 37, 1100);
  intakeStop();
  rollersStop();
  turnPID(30, 40, 900);

  // move towards tower 4 and turn to shoot
  forwardPID(36, 60, 2000); // used to be 36
  turnPID(118, 50, 1000);
  forwardInchesTimed(6, 30, 750);
  rollersOut(100);
  intake(100);
  task::sleep(600);
  rollersStop();
  intakeStop();

  // back up and turn towards second ball
  backwardInchesTimed(6, 30, 1000);
  turnPID(28.5, 50, 1000);

  // pooping ball
  intake(100);
  rollersIn(100);
  task::sleep(100);
  poopOut(100);

  // drive towards second ball
  forwardPIDIntake(47, 60, 2000, 25);
  intakeStop();
  rollersStop();

  // turn towards tower one and shoot
  turnPID(70, 50, 900);
  forwardPID(27, 45, 1000);
  intake(100);
  rollersOut(100);
  task::sleep(750);
  rollersStop();
  intakeStop();

  // back up from tower one and turn towards third ball on the way tower 2
  backwardPID(18, 35, 1000);
  turnPID(-64, 50, 1000);

  // move towards third ball on way to tower 2
  intake(100);
  poopOut(100);
  forwardPIDIntake(48, 57, 2000, 45);
  task::sleep(50);
  intakeStop();
  rollersStop();

  // turn towards tower 2 and score
  turnPID(29, 40, 1000);
  forwardInchesTimed(6, 30, 900);
  rollersOut(100);
  intake(100);
  task::sleep(625);
  rollersStop();
  intakeStop();

  // back up from tower 2 and go for ball by tower three
  backwardInchesTimed(4.3, 30, 1000);
  turnPID(-63, 50, 1000);

  // poop
  intake(100);
  rollersIn(100);
  task::sleep(100);
  poopOut(100);

  forwardPID(37, 55, 2000);

  // collect the ball by wall of tower three
  turnPID(29, 40, 1000);
  intake(100);
  rollersOut(40);
  forwardPID(12, 45, 1000); // dist changed from  18
  task::sleep(200);

  intakeStop();
  rollersStop();

  // back up from ball by wall at tower three turn and score
  backwardInchesTimed(5, 30, 1000);
  turnPID(-36, 50, 1000);

  forwardPID(26.7, 45, 1300); // dist changed from 26
  rollersOut(100);
  intake(100);
  task::sleep(600);
  rollersStop();

  // back up from tower 3 and turn to face ball by tower 6
  backwardInchesTimed(7, 35, 1000);
  turnPID(-151, 45,
          1200); // used to be -153 but it was turning too much to the right

  // poop
  intake(100);
  rollersIn(100);
  task::sleep(100);
  poopOut(100);

  // get ball towards tower 6
  intake(100);
  forwardPIDIntake(56, 60, 2400, 10);
  intakeStop();
  rollersStop();

  // turn and score at tower 6
  turnPID(-55, 50, 1000);
  forwardInchesTimed(7, 30, 1000);
  rollersOut(100);
  intake(100);
  task::sleep(700);
  rollersStop();
  intakeStop();

  backwardInchesTimed(5.5, 30, 1000); // back up from tower 6
  turnPID(-147, 50, 1000);            // face ball on the way to tower 9

  // poop
  intake(100);
  rollersIn(100);
  task::sleep(100);
  poopOut(100);

  // drive towards tower 9 ball
  intake(100);
  forwardPIDIntake(50, 60, 2000, 30);
  intakeStop();
  rollersStop();

  // turn towards tower 9 and shoot
  turnPID(-107, 50, 1000);
  forwardPID(27.5, 45, 1000);
  rollersOut(100);
  intake(100);
  task::sleep(700);
  intakeStop();
  rollersStop();

  // back up and turn towards tower 8
  backwardInchesTimed(8, 35, 1000);
  turnPID(-244, 50, 1100);

  // poop
  intake(100);
  rollersIn(100);
  task::sleep(100);
  poopOut(100);

  // go towards ball at tower 8
  intake(100);
  forwardPIDIntake(50.5, 60, 1800, 50);
  task::sleep(200);
  intakeStop();
  rollersStop();

  // turn and shoot at tower 8
  turnPID(-150, 50, 1000);
  forwardInchesTimed(2.5, 30, 750);
  rollersOut(100);
  // intake(100);
  task::sleep(800);
  rollersStop();
  // intakeStop();

  // back up and turn towards middle tower
  backwardInchesTimed(6.5, 30, 1000);
  rollersOut(60);
  turnPID(-335, 50, 1900); // needed to turn more right when it was at -330

  /*
      // poop
  intake(100);
      rollersIn(100);
      task::sleep(100);
      poopOut(80);
      */

  // pick up ball on the way to middle tower
  // pick up ball on the way to middle tower
  intake(100);
  rollersOut(50);
  forwardPID(23, 52, 1000);
  rollersStop();
  intakeStop();

  // descore middle tower and score one red
  outtake(100);
  forwardPID(25, 95, 1000);
  rollersOut(100);
  task::sleep(900);

  backwardInches(2.2, 50);
  forwardPID(5, 100, 600);
  task::sleep(100);
  backwardInches(2.2, 50);
  forwardInchesTimed(5, 100, 600);
  task::sleep(100);
  backwardInches(3, 50);
  intakeStop();
  rollersStop();
}

void Kali108(){
 // flip out
  outtake(100);
  task::sleep(400);
  intakeStop();

  ///////// collect the first ball and turn towards tower 4
  intake(100);
  rollersOutBottom(50);
  forwardPID(17, 50, 1100);
  intakeStop();
  rollersStop();
  turnPID(25, 16, 1500);
 // task::sleep(1000);

  
  // move towards tower 4 and turn to shoot
  forwardPID(24, 50, 1500); // used to be 36
  turnPID(118, 18, 1000);
  forwardInchesTimed(4, 25, 750);
  rollersOut(100);
  task::sleep(600);
  rollersStop();

  
  // back up and turn towards second ball
  backwardInchesTimed(4, 25, 700);
  
  turnPID(25, 16, 1500);
  

//task::sleep(1000);


  // drive towards second ball
  intake(100);
  rollersOutBottom(35);
  forwardPID(35, 50, 1900);
  intakeStop();
  rollersStop();

  // turn towards tower one and shoot

  
  turnPID(69, 11, 1500);
  forwardPID(28, 30, 1000);
  rollersOut(100);
  task::sleep(600);
  rollersStop();
  
  //back up from tower one and turn to tower 2
  backwardInchesTimed(8, 30, 2000);
  turnPID(-59,18,1500);


//go to tower two and turn to score
  intake(100);
  rollersOutBottom(35);
  forwardPID(36, 50, 2000);
  intakeStop();
  rollersStop();
  turnPID(21, 16, 1500);

  //score at tower 2
 forwardInchesTimed(4, 25, 750);
 rollersOut(100);
  task::sleep(600);
  rollersStop();

  //back up from tower 2 and turn to tower 3
 backwardInchesTimed(6, 25, 1000);
turnPID(-59,18,1500);

// turn towards ball by tower three
 forwardPID(26, 50, 2000);
  turnPID(22, 16, 1500);

//get ball by tower three
intake(100);
rollersOutBottom(30);
forwardInchesTimed(7, 30, 2000);
backwardInchesTimed(8,30,2000);
intakeStop();
rollersStop();

//turn to tower three
turnPID(-30,16,2000);


// go foward to tower three and score
forwardPID(24, 50, 1000);
rollersOut(100);
  task::sleep(600);
  rollersStop();


//back up from tower three and turn to go to tower 6
backwardPID(9.5,30,1000);
turnPID(-148,18,2000);

// go towards tower 6 and turn
intake(100);
rollersOutBottom(40);
forwardPID(40.5, 50, 2000);
intakeStop();
rollersStop();
turnPID(-59,18,1500);

// score at tower 6
 forwardInchesTimed(3.5, 23, 750);
rollersOut(100);
  task::sleep(600);
  rollersStop();

  //back up from tower 6 and turn to tower 9
   backwardInchesTimed(6, 25, 1000);
   turnPID(-148,18,2000);

// go forward to tower 9
intake(100);
rollersOutBottom(40);
forwardPID(32,50,2000);
intakeStop();
rollersStop();

// turn and score at tower 9
turnPID(-102,18,2000);
forwardPID(29, 30, 1000);
rollersOut(100);
  task::sleep(600);
  rollersStop();

  //back up from tower 9 and turn to tower 8
backwardInchesTimed(9.5, 30, 2000);
turnPID(-239,18,2000);

//go towards tower 8
intake(100);
rollersOutBottom(45);
forwardPID(34, 50, 2000);
intakeStop();
rollersStop();

//turn towards tower 8
turnPID(-150,18,2000);
 forwardInchesTimed(3.5, 20, 750);
 rollersOut(100);
  task::sleep(600);
  rollersStop();

// backup from tower 8 and turn towards middle tower
  backwardInchesTimed(6, 25, 1000);
  turnPID(24, 18, 1500);

// get ball towards middle
  intake(100);
  rollersOutBottom(50);
  forwardPID(18, 30, 1200);
  rollersStop();
  intakeStop();


//descore middle and score
outtake(100);
  forwardPID(20, 95, 1000);
  rollersOut(100);
  task::sleep(600);

//descore middle
  backwardInchesTimed(3, 100,500);
  forwardPID(6, 100, 600);
  task::sleep(100);
  backwardInchesTimed(3, 100,500);
  forwardInchesTimed(6, 100, 600);
  backwardInches(3, 100);
  intakeStop();
  rollersStop();
}





void LateNightKali108(){
  
 // flip out
  outtake(100);
  task::sleep(400);
  intakeStop();

  ///////// collect the first ball and turn towards tower 4
  intake(100);
  rollersOutBottom(50);
  forwardPID(17, 50, 1100);
  intakeStop();
  rollersStop();
  turnPID(25, 16, 1500);
 // task::sleep(1000);

  
  // move towards tower 4 and turn to shoot
  forwardPID(24, 50, 1500); // used to be 36
  turnPID(118, 18, 1000);
  forwardInchesTimed(4, 25, 750);
  rollersOut(100);
  task::sleep(500);
  rollersStop();

  
  // back up and turn towards second ball
  backwardInchesTimed(4, 25, 700);
  
  turnPID(25, 16, 1500);
  

//task::sleep(1000);


  // drive towards second ball
  intake(100);
  rollersOutBottom(20);
  forwardPID(35, 50, 1900);
  intakeStop();
  rollersStop();

  // turn towards tower one and shoot

  
  turnPID(69, 11, 1500);
  forwardPID(28, 30, 1000);
  rollersOut(100);
  task::sleep(600);
  rollersStop();
  
  //back up from tower one and turn to tower 2
  backwardInchesTimed(8, 30, 2000);
  turnPID(-59,18,1500);


//go to tower two and turn to score
  intake(100);
  rollersOutBottom(30);
  forwardPID(36, 50, 2000);
  intakeStop();
  rollersStop();
  turnPID(21, 16, 1500);

  //score at tower 2
 forwardInchesTimed(4, 25, 750);
 rollersOut(100);
  task::sleep(700);
  rollersStop();

  //back up from tower 2 and turn to tower 3
 backwardInchesTimed(6, 25, 1000);
turnPID(-59,18,1500);

// turn towards ball by tower three
 forwardPID(26, 50, 2000);
  turnPID(22, 16, 1500);

//get ball by tower three
intake(100);
rollersOutBottom(30);
forwardInchesTimed(7, 30, 2000);
backwardInchesTimed(8,30,2000);
intakeStop();
rollersStop();

//turn to tower three
turnPID(-30,16,2000);


// go foward to tower three and score
forwardPID(24, 50, 1000);
rollersOut(100);
  task::sleep(600);
  rollersStop();


//back up from tower three and turn to go to tower 6
backwardPID(9.5,30,1000);
turnPID(-148,18,2000);

// go towards tower 6 and turn
intake(100);
rollersOutBottom(40);
forwardPID(40.5, 50, 2000);
intakeStop();
rollersStop();
turnPID(-59,18,1500);

// score at tower 6
 forwardInchesTimed(3.5, 23, 750);
rollersOut(100);
  task::sleep(600);
  rollersStop();

  //back up from tower 6 and turn to tower 9
   backwardInchesTimed(6, 25, 1000);
   turnPID(-148,18,2000);

// go forward to tower 9
intake(100);
rollersOutBottom(40);
forwardPID(32,50,2000);
intakeStop();
rollersStop();

// turn and score at tower 9
turnPID(-103,18,2000);
forwardPID(29, 30, 1000);
rollersOut(100);
  task::sleep(600);
  rollersStop();

  //back up from tower 9 and turn to tower 8
backwardInchesTimed(9, 30, 2000);
turnPID(-239,18,2000);

//go towards tower 8
intake(100);
rollersOutBottom(45);
forwardPID(34, 50, 2000);
intakeStop();
rollersStop();

//turn towards tower 8
turnPID(-149,18,2000);
 forwardInchesTimed(4, 23, 750);
 rollersOut(100);
  task::sleep(600);
  rollersStop();

// backup from tower 8 and turn towards middle tower
  backwardInchesTimed(6, 25, 1000);
  turnPID(24, 18, 1500);

// get ball towards middle
  intake(100);
  rollersOutBottom(50);
  forwardPID(18, 30, 1200);
  rollersStop();
  intakeStop();


//descore middle and score

outtake(100);
  forwardPID(20, 95, 1500);
  rollersOut(100);
  task::sleep(600);
  

//descore middle
  backwardInchesTimed(3, 50,500);
  forwardPID(10, 20, 500);
  task::sleep(100);
  backwardInchesTimed(3, 50,500);
  forwardInchesTimed(15, 22, 800);
  backwardInches(3, 100);
  intakeStop();
  rollersStop();
}


/*
void autoStackDriver() { railToDegDriver(POT_RAIL_STACK); }

void resetRailDriver() {//railToDegDriver(POT_RAIL_RESET);
RAIL.startRotateTo(0, rotationUnits::deg, 100, velocityUnits::pct); }


void armLowDriver() { armToDegDriverEncoder(ARM_LOW, 70); }

void armMidDriver() { armToDegDriverEncoder(ARM_MID, 70); }

void armHighDriver() { armToDegDriverEncoder(ARM_HIGH, 70); }*/

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

int lowestVal = 100;
int i = 0;
void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    LFBASE.spin(directionType::fwd,
                controllerPrim.Axis3.value() * (100.0 / 127.0),
                velocityUnits::pct);
    LBBASE.spin(directionType::fwd,
                controllerPrim.Axis3.value() * (100.0 / 127.0),
                velocityUnits::pct);
    RFBASE.spin(directionType::fwd,
                controllerPrim.Axis2.value() * (100.0 / 127.0),
                velocityUnits::pct);
    RBBASE.spin(directionType::fwd,
                controllerPrim.Axis2.value() * (100.0 / 127.0),
                velocityUnits::pct);

    /*if (controllerPrim.ButtonL1.pressing())
    {
      LINTAKE.spin(directionType::fwd, 100, velocityUnits::pct);
      RINTAKE.spin(directionType::rev, 100, velocityUnits::pct);
    }
    else if (controllerPrim.ButtonL2.pressing())
    {
      LINTAKE.spin(directionType::rev, 100, velocityUnits::pct);
      RINTAKE.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    else
    {
      LINTAKE.stop(brakeType::hold);
      RINTAKE.stop(brakeType::hold);
    }*/

    // Intake controls
    double intakeSpeed = 100;
    if (controllerPrim.ButtonL2.pressing()) {
      LINTAKE.spin(directionType::fwd, intakeSpeed, velocityUnits::pct);
      RINTAKE.spin(directionType::fwd, intakeSpeed, velocityUnits::pct);
    } else if (controllerPrim.ButtonL1.pressing()) {
      LINTAKE.spin(directionType::fwd, -intakeSpeed, velocityUnits::pct);
      RINTAKE.spin(directionType::fwd, -intakeSpeed, velocityUnits::pct);
    } else {
      LINTAKE.stop();
      RINTAKE.stop();
    }

    // Outtake controls
    double outtakeSpeed = 100;
    // Spin outtake rollers in the same direction
    if (controllerPrim.ButtonR1.pressing()) {
      BROLLER.spin(directionType::fwd, outtakeSpeed, velocityUnits::pct);
      TROLLER.spin(directionType::fwd, outtakeSpeed, velocityUnits::pct);
    } else if (controllerPrim.ButtonR2.pressing()) {
      BROLLER.spin(directionType::fwd, -outtakeSpeed, velocityUnits::pct);
      TROLLER.spin(directionType::fwd, -outtakeSpeed, velocityUnits::pct);
    }
    // Normal poop (rotate top & bottom opposite directions)
    else if (controllerPrim.ButtonB.pressing()) {
      BROLLER.spin(directionType::fwd, -outtakeSpeed, velocityUnits::pct);
      TROLLER.spin(directionType::fwd, outtakeSpeed, velocityUnits::pct);
    }
    // Reverse poop
    else if (controllerPrim.ButtonX.pressing()) {
      BROLLER.spin(directionType::fwd, -outtakeSpeed, velocityUnits::pct);
      TROLLER.spin(directionType::fwd, outtakeSpeed, velocityUnits::pct);
    }
    // Only spin the bottom outtake rollers.
    else if (controllerPrim.ButtonDown.pressing()) {
      TROLLER.spin(directionType::rev, 100, velocityUnits::pct);
      BROLLER.stop();
    }
    // Only spin top rollers
    /*
    else if (controllerPrim.ButtonX.pressing())
    {
      TROLLER.spin(directionType::fwd, -40, velocityUnits::pct);
      BROLLER.stop();
    }*/
    else {
      BROLLER.stop();
      TROLLER.stop();
    }
    /*
    controllerPrim.Screen.clearScreen();
    task::sleep(75);
    controllerPrim.Screen.setCursor(1, 1);
    task::sleep(75);
    controllerPrim.Screen.print("%d %d %d %d %d", Inertial.pitch(deg),
    Inertial.yaw(deg), Inertial.roll(deg), Inertial.heading(deg),
    Inertial.rotation(deg));
    */

    Brain.Screen.printAt(140, 45, "INERTIAL: %.2f", Inertial.value());
    Brain.Screen.printAt(140, 25, "INERTIAL2: %.2f", getRotation());
    Brain.Screen.printAt(5, 260, "Counting: %.2f", i++);
    Brain.Screen.printAt(140, 65, "TINDEX: %d", tBallDetected());
    Brain.Screen.printAt(140, 85, "BINDEX: %d", bBallDetected());

    Brain.Screen.printAt(140, 125, "TINDEX value 1: %d, 2: %d", TINDEX1.value(pct), TINDEX2.value(pct));
    Brain.Screen.printAt(140, 145, "BINDEX value: %d", BINDEX.value(pct));
    Brain.Screen.printAt(140, 165, "PIDNEX value: %d", PINDEX.value(pct));

    lowestVal = fmin(lowestVal, BINDEX.value(pct));
    Brain.Screen.printAt(140, 185, "lowest BINDEX: %d", lowestVal);

    // Brain.Screen.printAt(150, 60, "RailPot: %.2f",
    // RailPot.value(rotationUnits::deg)); Brain.Screen.printAt(150, 100,
    // "RailEncoder: %.2f", RAIL.rotation(rotationUnits::deg)); //2400 stacking
    // position Brain.Screen.printAt(150, 80, "i: %d", i++);
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

void jasonUserControl(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    LFBASE.spin(directionType::fwd,
                controllerPrim.Axis3.value() * (127.0 / 127.0),
                velocityUnits::pct);
    LBBASE.spin(directionType::fwd,
                controllerPrim.Axis3.value() * (127.0 / 127.0),
                velocityUnits::pct);
    RFBASE.spin(directionType::fwd,
                controllerPrim.Axis2.value() * (127.0 / 127.0),
                velocityUnits::pct);
    RBBASE.spin(directionType::fwd,
                controllerPrim.Axis2.value() * (127.0 / 127.0),
                velocityUnits::pct);

    /*if (controllerPrim.ButtonL1.pressing())
    {
      LINTAKE.spin(directionType::fwd, 100, velocityUnits::pct);
      RINTAKE.spin(directionType::rev, 100, velocityUnits::pct);
    }
    else if (controllerPrim.ButtonL2.pressing())
    { 
      LINTAKE.spin(directionType::rev, 100, velocityUnits::pct);
      RINTAKE.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    else
    {
      LINTAKE.stop(brakeType::hold);
      RINTAKE.stop(brakeType::hold);
    }*/

    // Intake controls
    double intakeSpeed = 100;
    if (controllerPrim.ButtonL1.pressing()) {
      LINTAKE.spin(directionType::fwd, intakeSpeed, velocityUnits::pct);
      RINTAKE.spin(directionType::fwd, intakeSpeed, velocityUnits::pct);
    } else if (controllerPrim.ButtonL2.pressing()) {
      LINTAKE.spin(directionType::fwd, -intakeSpeed, velocityUnits::pct);
      RINTAKE.spin(directionType::fwd, -intakeSpeed, velocityUnits::pct);
    } else {
      LINTAKE.stop();
      RINTAKE.stop();
    }

    // Outtake controls
    double outtakeSpeed = 100;
    // Spin outtake rollers in the same direction
    if (controllerPrim.ButtonR1.pressing()) {
      BROLLER.spin(directionType::fwd, outtakeSpeed, velocityUnits::pct);
      TROLLER.spin(directionType::fwd, outtakeSpeed, velocityUnits::pct);
    } else if (controllerPrim.ButtonR2.pressing()) {
      BROLLER.spin(directionType::fwd, -outtakeSpeed, velocityUnits::pct);
      TROLLER.spin(directionType::fwd, -outtakeSpeed, velocityUnits::pct);
    }
    // Normal poop (rotate top & bottom opposite directions)
    else if (controllerPrim.ButtonB.pressing()) {
      BROLLER.spin(directionType::fwd, -outtakeSpeed, velocityUnits::pct);
      TROLLER.spin(directionType::fwd, outtakeSpeed, velocityUnits::pct);
    }
    // Reverse poop
    else if (controllerPrim.ButtonX.pressing()) {
      BROLLER.spin(directionType::fwd, -outtakeSpeed, velocityUnits::pct);
      TROLLER.spin(directionType::fwd, outtakeSpeed, velocityUnits::pct);
    }
    // Only spin the bottom outtake rollers.
    else if (controllerPrim.ButtonDown.pressing()) {
      BROLLER.spin(directionType::rev, 100, velocityUnits::pct);
      TROLLER.stop();
    }
    // Only spin top rollers
    /*
    else if (controllerPrim.ButtonX.pressing())
    {
      TROLLER.spin(directionType::fwd, -40, velocityUnits::pct);
      BROLLER.stop();
    }*/
    else {
      BROLLER.stop();
      TROLLER.stop();
    }
    /*
    controllerPrim.Screen.clearScreen();
    task::sleep(75);
    controllerPrim.Screen.setCursor(1, 1);
    task::sleep(75);
    controllerPrim.Screen.print("%d %d %d %d %d", Inertial.pitch(deg),
    Inertial.yaw(deg), Inertial.roll(deg), Inertial.heading(deg),
    Inertial.rotation(deg));
    */

    Brain.Screen.printAt(140, 45, "INERTIAL: %.2f", Inertial.value());
    Brain.Screen.printAt(140, 25, "INERTIAL2: %.2f", getRotation());
    Brain.Screen.printAt(5, 260, "Counting: %.2f", i++);
    Brain.Screen.printAt(140, 65, "TINDEX: %d", tBallDetected());
    Brain.Screen.printAt(140, 85, "BINDEX: %d", bBallDetected());

    // Brain.Screen.printAt(150, 60, "RailPot: %.2f",
    // RailPot.value(rotationUnits::deg)); Brain.Screen.printAt(150, 100,
    // "RailEncoder: %.2f", RAIL.rotation(rotationUnits::deg)); //2400 stacking
    // position Brain.Screen.printAt(150, 80, "i: %d", i++);
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  LFBASE.setBrake(brakeType::coast);
  LBBASE.setBrake(brakeType::coast);
  RFBASE.setBrake(brakeType::coast);
  RBBASE.setBrake(brakeType::coast);

  RINTAKE.setBrake(brakeType::coast);
  LINTAKE.setBrake(brakeType::coast);

  TROLLER.setBrake(brakeType::coast);
  BROLLER.setBrake(brakeType::coast);

  // Set up callbacks for autonomous and driver control periods.
  //Competition.autonomous(IFT);//IFT);
 //Competition.autonomous(pidTester);
 //Competition.autonomous(Kali108);
//Competition.autonomous(LateNightKali108);

Competition.autonomous(kalahariHomeRow);
  //Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
