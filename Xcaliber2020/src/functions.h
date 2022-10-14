#include "vex.h"

using namespace vex;

#define PI 3.14159265



const float WHEEL_CIRCUMFERENCE = 3.25 * PI;

const float TRACK_WIDTH = 7.5; // Width between right and left wheels
const float WHEEL_BASE = 4.5; // Length between front and back
const float CIRCUMDIAMETER = sqrt(pow(TRACK_WIDTH, 2) + pow(WHEEL_BASE, 2)); // Calculate diagonal distance between nonadjacent wheels using pythagorean theorum.
const float BASE_CIRCUMFERENCE = CIRCUMDIAMETER*PI; // Calculate circumference of circle which circumscribes the base.



// Specify desired potentiometer degree to slow down rail at.
const float RAIL_POT_DEACCEL = 150;


// Specify desired positions for preset rail and arm positions for driver.
const float POT_RAIL_STACK = 246;
const float POT_RAIL_RESET = 0;

const float POT_ARM_LOW = 246;
const float POT_ARM_MID = 125;
const float POT_ARM_HIGH = 1;


// Declare gyro self correct PID variable
float kp = .39;//the proportional term
double desiredDegree = 0.0;
double KPC = 50.0;
double KDC = 0.12;
double correctionL = 0;
double correctionR = 0;
double cSpeed = 0;

double desiredX = 0; // Desired value for horizontal encoder to be. Robot will turn to this angle.


// Cube detection
bool cubeDetectEnabled = true;
int brightnessThreshold = 65;
int cubeCount = 0;
bool detectingCube = false;




// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller controllerPrim(controllerType::primary);

motor RFBASE(PORT5,true);
motor LFBASE(PORT20);
motor RBBASE(PORT1,true);
motor LBBASE(PORT19);
motor ARM(PORT16);
motor RAIL(PORT6);
motor LINTAKE(PORT17);
motor RINTAKE(PORT7);


gyro Gyro1(Brain.ThreeWirePort.G);
encoder xTracker(Brain.ThreeWirePort.E); // Tracks horizontal shift of robot. Used to keep robot in straight line.

pot RailPot(Brain.ThreeWirePort.H);
pot ArmPot(Brain.ThreeWirePort.F);
line cubeDetector(Brain.ThreeWirePort.D); // Line sensor for detecting cube touches.





float inchesToTicks(float inches) {
  // Derived from: length/circumference = deg/360
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
  if (number < bottom) number = bottom;
  if (number > top) number = top;
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


void clearEncoders(){
  RFBASE.resetRotation();
  LFBASE.resetRotation();
  RBBASE.resetRotation();
  LBBASE.resetRotation();

}





void forwardInches(float inches, int maxSpeed) {
  float ticks = inchesToTicks(inches);
  //float initialAngle = Gyro1.value(deg);

  float targetRange = 2; // Distance from the desired distance the robot has to be to stop.
  float error = ticks; // Distance from the desired range
  float progress; // Ticks the motors has turned already
  float minSpeed = 2; // Lowest speed the motors will go; Turning is generally more precise and accurate when lower.
  float accelRate = 30;
  float deaccelRate = 12;

  float speed; // Actual speed value of the motors

  maxSpeed = restrictToRange(maxSpeed, 0, 100);
  clearEncoders();

  while (error > targetRange) {

    progress = (LFBASE.rotation(deg)+RFBASE.rotation(deg)+LBBASE.rotation(deg)+RBBASE.rotation(deg))/4; // Average value of all motors
    error = ticks-progress;
    
    // First half: accelerate to max; Second half, deccelerate to min
    if (error >= ticks/2) {
     speed = (((progress/ticks))*maxSpeed*accelRate)+minSpeed;
    } else {
      speed = ((1-(progress/ticks))*maxSpeed*deaccelRate)+minSpeed;
    }
    speed = restrictToRange(speed, minSpeed, maxSpeed);

    RFBASE.spin(fwd, speed + correctionR, pct);
    LFBASE.spin(fwd, speed + correctionL, pct);
    RBBASE.spin(fwd, speed + correctionR, pct);
    LBBASE.spin(fwd, speed + correctionL, pct);

    Brain.Screen.printAt(1, 120, "Desired distance: %.2f, Progress: %.2f", ticksToInches(ticks), ticksToInches(progress));
    Brain.Screen.printAt(1, 140, "Speed: %.2f, Error: %.2f", speed, error);

  }

  stopBase();
  task::sleep(15);

}




void backwardInches(float inches, int maxSpeed) {
  float ticks = -inchesToTicks(inches);
  //float initialAngle = Gyro1.value(deg);

  float targetRange = 2; // Distance from the desired distance the robot has to be to stop.
  float error = ticks; // Distance from the desired range
  float progress = 0; // Ticks the motors has turned already
  float minSpeed = 2; // Lowest speed the motors will go; Turning is generally more precise and accurate when lower.
  float accelRate = 30;
  float deaccelRate = 9;

  float speed = 0; // Actual speed value of the motors

  maxSpeed = restrictToRange(maxSpeed, 0, 100);
  clearEncoders();


    Brain.Screen.printAt(1, 120, "Desired distance: %.2f, Progress: %.2f", ticksToInches(ticks), ticksToInches(progress));
    Brain.Screen.printAt(1, 140, "Speed: %.2f, Error: %.2f", speed, error);

  while (error < targetRange) {

    progress = (LFBASE.rotation(deg)+RFBASE.rotation(deg)+LBBASE.rotation(deg)+RBBASE.rotation(deg))/4; // Average value of all motors
    error = ticks-progress;
    
    // First half: accelerate to max; Second half, deccelerate to min
    if (error <= ticks/2) {
     speed = (((progress/ticks))*maxSpeed*accelRate)+minSpeed;
    } else {
      speed = ((1-(progress/ticks))*maxSpeed*deaccelRate)+minSpeed;
    }
    speed = restrictToRange(speed, minSpeed, maxSpeed);

    RFBASE.spin(directionType::rev, speed - correctionR, pct);
    LFBASE.spin(directionType::rev, speed - correctionL, pct);
    RBBASE.spin(directionType::rev, speed - correctionR, pct);
    LBBASE.spin(directionType::rev, speed - correctionL, pct);

    Brain.Screen.printAt(1, 120, "Desired distance: %.2f, Progress: %.2f", ticksToInches(ticks), ticksToInches(progress));
    Brain.Screen.printAt(1, 140, "Speed: %.2f, Error: %.2f", speed, error);

  }

  stopBase();
  task::sleep(15);

}




// Stay moving in the same direction.
int driveStraight()
{
    while( true )
	{
		double Error = fabs( desiredDegree - Gyro1.value(rotationUnits::deg));
		double Derivative = KPC * exp(-KDC * Error); //Sets the Derivative equal to exponential of the error
		double  Proportion = 200.0 / (1 + Derivative)-4; //sets Proportion to the desired speed over the derivative
    cSpeed = (Proportion*1.1); // Make turning speed a little faster than proportion
    if (Error != 0) { // If error is not completely eliminated, make sure speed is at least 1 (or -1) to ensure speed is enough to overcome friction/weight of robot.
      cSpeed +=1 ;
    }
        //Brain.Screen.printAt(140, 55, "Gyro: %.3f", Gyro1.value(rotationUnits::deg));
		if( Gyro1.value(rotationUnits::deg) < desiredDegree ) 
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
        Brain.Screen.printAt(0, 120, "Gyro: %.2f", Gyro1.value(rotationUnits::deg));
	}
    return(0);
}



// Stay moving in the same direction and on the line.
int driveStraightLine()
{
  while( true )
	{
		double turnError = fabs( desiredDegree - Gyro1.value(rotationUnits::deg) );
    double xError = fabs ( desiredX - xTracker.rotation(rotationUnits::deg) );

    // Not sure if this is the way to do it, but the error is the average of the turn error and x error to account for both angle and horizontal position.
    double Error = (turnError + xError)/2;

		double Derivative = KPC * exp(-KDC * Error); //Sets the Derivative equal to exponential of the error
		double  Proportion = 200.0 / (1 + Derivative)-4; //sets Proportion to the desired speed over the derivative
    cSpeed = Proportion*1.34;
        //Brain.Screen.printAt(140, 55, "Gyro: %.3f", Gyro1.value(rotationUnits::deg));
		if( Gyro1.value(rotationUnits::deg) < desiredDegree ) 
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
        Brain.Screen.printAt(0, 120, "Gyro: %.2f", Gyro1.value(rotationUnits::deg));
	}
    return(0);
}




/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                           Misc Movement Functions                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/


void railToDeg(float deg, int maxSpeed) {

  const float initialError = deg-RailPot.value(rotationUnits::deg); // Error = desired - actual mesasurement.

  float error = initialError;
  const float acceptableError = 4; // How far off from intended degree value that it's ok to stop at

  const int minSpeed = 5;
  const float deaccelRate = 3;
  Brain.Screen.printAt(0, 60, "%d", fabs(error) > acceptableError);
  Brain.Screen.printAt(25, 60, "%d, %d", error, acceptableError);

  while (fabs(error) > acceptableError) {
    error = deg-RailPot.value(rotationUnits::deg); // Error = desired - actual mesasurement.
    float speed = fabs((error/initialError)*maxSpeed*deaccelRate); // P-controller (proportional to error)
    
    // Limit the speed at maxSpeed because otherwise deaccel rate would increase speed past the max.
    // Keep the speed up to at least minSpeed, because otherwise steady-state error would become impossible to overcome.
    speed = restrictToRange(speed, minSpeed, maxSpeed);
    
    Brain.Screen.printAt(0, 20, "%.2f, %.2f", RailPot.value(rotationUnits::deg), RAIL.rotation(rotationUnits::deg));
    Brain.Screen.printAt(0, 40, "Error: %.2f, Speed: %.2f", error, speed);
    
    if (error < 0) {
      RAIL.spin(fwd, fabs(speed), velocityUnits::pct);
    } else {
      RAIL.spin(directionType::rev, fabs(speed), velocityUnits::pct);
    }

  }

  RAIL.stop(brakeType::hold);

}




void armToDeg(float deg, int maxSpeed) {

  const float initialError = deg-ArmPot.value(rotationUnits::deg); // Error = desired - actual mesasurement.

  float error = initialError;
  const float acceptableError = 4; // How far off from intended degree value that it's ok to stop at

  const int minSpeed = 5;
  const float deaccelRate = 3;
  Brain.Screen.printAt(0, 60, "%d", fabs(error) > acceptableError);
  Brain.Screen.printAt(25, 60, "%d, %d", error, acceptableError);

  while (fabs(error) > acceptableError) {
    error = deg-ArmPot.value(rotationUnits::deg); // Error = desired - actual mesasurement.
    float speed = fabs((error/initialError)*maxSpeed*deaccelRate); // P-controller (proportional to error)
    
    // Limit the speed at maxSpeed because otherwise deaccel rate would increase speed past the max.
    // Keep the speed up to at least minSpeed, because otherwise steady-state error would become impossible to overcome.
    speed = restrictToRange(speed, minSpeed, maxSpeed);
    
    Brain.Screen.printAt(0, 20, "%.2f, %.2f", ArmPot.value(rotationUnits::deg), ARM.rotation(rotationUnits::deg));
    Brain.Screen.printAt(0, 40, "Error: %.2f, Speed: %.2f", error, speed);
    
    if (error < 0) {
      ARM.spin(fwd, fabs(speed), velocityUnits::pct);
    } else {
      ARM.spin(directionType::rev, fabs(speed), velocityUnits::pct);
    }

  }

  ARM.stop(brakeType::hold);

}




int detectCubes()
{
  while( cubeDetectEnabled )
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
