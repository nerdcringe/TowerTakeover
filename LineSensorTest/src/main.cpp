
#include "vex.h"
using namespace vex;
vex::brain       Brain;

motor RFBASE(PORT5,true);
motor LFBASE(PORT20);
motor RBBASE(PORT1,true);
motor LBBASE(PORT19);
motor ARM(PORT16);
motor RAIL(PORT6);

gyro Gyro1(Brain.ThreeWirePort.G);
pot POT1(Brain.ThreeWirePort.H);
line cubeDetector(Brain.ThreeWirePort.F);


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

// Line sensor for detecting cube touches.
int brightnessThreshold = 65;
int cubeCounter = 0;
bool detectingCube = false;



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
        cubeCounter += 1;
        detectingCube = true;
        ControllerPrim.Screen.clearScreen();
        ControllerPrim.Screen.print(cubeCounter);
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



void pre_auton() {
    Gyro1.calibrate(2000);
    task::sleep(1000);
  
    task cubeDetection(detectCubes); 
}




int main() {
    int count = 0;
    pre_auton();
    

    stopBase();

    while(1) {
    
      if (cubeCounter == 7)
      {
        ARM.spin(fwd, 40, velocityUnits::pct); // Test spin to make sure sensor/logic works
        cubeCounter = 0; // Reset count to prevent conditional evaluating to true before 7 cubes are detected again.
      }


      Brain.Screen.printAt( 10, 50, "Hello V5 %d", count++ );
      
      Brain.Screen.printAt( 10, 70, "LightSensor Value %d", cubeDetector.value(percentUnits::pct));
      // Allow other tasks to run
      this_thread::sleep_for(10);
    }
}
