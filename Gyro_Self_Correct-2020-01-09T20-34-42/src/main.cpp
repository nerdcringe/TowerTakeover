
#include "vex.h"
using namespace vex;
vex::brain       Brain;

motor RFBASE(PORT5,true);
motor LFBASE(PORT20);
motor RBBASE(PORT1,true);
motor LBBASE(PORT19);
motor ARM(PORT16);
motor RAIL(PORT6);


encoder xTracker(Brain.ThreeWirePort.E); // Tracks horizontal shift of robot. Used to keep robot in straight line.

gyro Gyro1(Brain.ThreeWirePort.G);
pot POT1(Brain.ThreeWirePort.H);

controller ControllerPrim = controller(controllerType:: primary);



#define PI 3.14159265
const float TILE_LENGTH = 24;
const float WHEEL_CIRCUMFERENCE = 4 * PI;
const static float ARM_MAX = 30;
const static float ARM_MIN = 0;


float kp = .39;//the proportional term
double degree = 0.0;
double KPC = 50.0;
double KDC = 0.12;
double correctionL = 0;
double correctionR = 0;
double cSpeed = 0;

double desiredX = 0;



int toggle = 1;
bool ContinueC = true;
int   screen_origin_x = 150;
int   screen_origin_y = 20;
int   screen_width    = 316;
int   screen_height   = 212;
int maxHeight = 110;
int maxWidth = 80;
int bLineX = 150;
int bLineY = 95;
int rLineX = 150;
int rLineY = 95;
int tolerance = 5;
bool nullAuton = true;
bool redSide = false;






void stopBase() {
  RFBASE.stop(brakeType::coast);
  LFBASE.stop(brakeType::coast); 
  RBBASE.stop(brakeType::coast);
  LBBASE.stop(brakeType::coast);
  
}

float inchesToTicks(float inches) {
  float ticks = (inches / WHEEL_CIRCUMFERENCE) * 360;
  return ticks;

}

float ticksToInches(float ticks) {
  float inches = (ticks * WHEEL_CIRCUMFERENCE) / 360;
  return inches;

}


float restrictToRange(float number, float bottom, float top) {
  Brain.Screen.printAt(200, 175, "%.3f", number);
  if (number < bottom) number = bottom;
  if (number > top) number = top;
   return number;
 }

void clearEncoders(){
  RFBASE.resetRotation();
  LFBASE.resetRotation();
  RBBASE.resetRotation();
  LBBASE.resetRotation();

}

void railToDegrees(float degrees,  int speed, bool wait = false) {
  //if (degrees >= ARM_MAX) degrees = ARM_MAX;
  //if (degrees <= ARM_MIN) degrees = ARM_MIN;

  if (!wait) {
    RAIL.startRotateTo(-degrees, rotationUnits::deg, speed, velocityUnits::pct);

  } else {
    RAIL.rotateTo(-degrees, rotationUnits::deg, speed, velocityUnits::pct);
    //stopBase();
  }
  task::sleep(15);
}


int driveStraight()
{
    while( true )
	{
		double Error = fabs( degree - Gyro1.value(rotationUnits::deg));
		double Derivative = KPC * exp(-KDC * Error); //Sets the Derivative equal to exponential of the error
		double  Proportion = 200.0 / (1 + Derivative)-4; //sets Proportion to the desired speed over the derivative
    cSpeed = (Proportion*1.1); // Make turning speed a little faster than proportion
    if (Error != 0) { // If error is not completely eliminated, make sure speed is at least 1 (or -1) to ensure speed is enough to overcome friction/weight of robot.
      cSpeed +=1 ;
    }
        //Brain.Screen.printAt(140, 55, "Gyro: %.3f", Gyro1.value(rotationUnits::deg));
		if( Gyro1.value(rotationUnits::deg) < degree ) 
    {
			correctionL = cSpeed;
			correctionR = -cSpeed;
		}
		else if ( Gyro1.value(rotationUnits::deg) > degree ) 
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



int driveStraight2()
{
  while( true )
	{
		double turnError = fabs( degree - Gyro1.value(rotationUnits::deg) );
    double xError = fabs ( desiredX - xTracker.rotation(rotationUnits::deg) );

    // Not sure if this is the way to do it, but the error is the average of the turn error and x error to account for both angle and horizontal position.
    double Error = (turnError + xError)/2;

		double Derivative = KPC * exp(-KDC * Error); //Sets the Derivative equal to exponential of the error
		double  Proportion = 200.0 / (1 + Derivative)-4; //sets Proportion to the desired speed over the derivative
    cSpeed = Proportion*1.34;
        //Brain.Screen.printAt(140, 55, "Gyro: %.3f", Gyro1.value(rotationUnits::deg));
		if( Gyro1.value(rotationUnits::deg) < degree ) 
    {
			correctionL = cSpeed;
			correctionR = -cSpeed;
		}
		else if ( Gyro1.value(rotationUnits::deg) > degree ) 
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
  float railSet = false;

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



void pre_auton() {
    Gyro1.calibrate(2000);
    task::sleep(1000);
  
    task gyroCorrect(driveStraight);
    //task gyroCorrect(driveStraight2); // horizontal correction
    xTracker.resetRotation(); // Makes current x 0, the desired position for the tracker to be in. If the robot shifts sideways, it will turn to go back to that x position.
}



int main() {
    int count = 0;

    pre_auton();
    backwardInches(80, 85);
    
    stopBase();

    while(1) {
    
      RFBASE.spin(directionType::rev,  - correctionR, pct);
      LFBASE.spin(directionType::rev,  - correctionL, pct);
      RBBASE.spin(directionType::rev,  - correctionR, pct);
      LBBASE.spin(directionType::rev,  - correctionL, pct);

        Brain.Screen.printAt( 10, 50, "Hello V5 %d", count++ );
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
