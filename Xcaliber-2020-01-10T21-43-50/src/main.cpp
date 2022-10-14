
#include "vex.h"
using namespace vex;
vex::brain       Brain;
competition Competition;

motor RFBASE(PORT5,true);
motor LFBASE(PORT20);
motor RBBASE(PORT1,true);
motor LBBASE(PORT19);
motor ARM(PORT16);
motor RAIL(PORT6);

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
		double Error = fabs( degree - Gyro1.value(rotationUnits::deg) );
		double Derivative = KPC * exp(-KDC * Error); //Sets the Derivative equal to exponential of the error
		double  Proportion = 200.0 / (1 + Derivative)-4; //sets Proportion to the desired speed over the derivative
    cSpeed = (Proportion*1.1); // Make turning speed a little faster than proportion
    if (Error != 0) { // If error is not completely eliminated, make sure speed is at least 1 (or -1) to ensure speed is still enough to overcome friction/weight of robot.
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
		double Error = fabs( degree - Gyro1.value(rotationUnits::deg) );
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


#define LEFT_ARROW  247
#define RIGHT_ARROW 246
static  char l_arr_str[4] = { LEFT_ARROW,  LEFT_ARROW,  LEFT_ARROW,  0};
static  char r_arr_str[4] = { RIGHT_ARROW, RIGHT_ARROW, RIGHT_ARROW, 0};


static int MyAutonomous = 0;
#define MAX_CHOICE  3

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











 void Picture()
 {
    color yellow1;
    yellow1.web("#f0e637");


   Brain.Screen.setFillColor("#7A016B");
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.setCursor(1,2);
     Brain.Screen.setFont(mono30);
     Brain.Screen.print("1575Xcaliber");

Brain.Screen.setFillColor(yellow1);
 Brain.Screen.setPenColor(color::black);
Brain.Screen.drawCircle(120,135, 100);


 Brain.Screen.setPenColor(color::black);
 Brain.Screen.setPenWidth(6);     
Brain.Screen.drawLine(135 , 70 , 185 , 120);
Brain.Screen.drawLine(135 , 120 , 185 , 70);


Brain.Screen.setPenColor(color::black);
Brain.Screen.setPenWidth(6);     
Brain.Screen.drawLine(55 , 70 , 105 , 120);
Brain.Screen.drawLine(55 , 120 , 105 , 70);

// Mouth
Brain.Screen.setPenWidth(7);  
Brain.Screen.drawLine(60 , 175 , 180 , 175);

  color pink;
    pink.web("#FD008A");

    vex::color tan;
    tan.web("#D2B48C");

      color grey1;
    grey1.web("#494949");

    color white1;
    white1.web("#0af069");

// Color for text and box    
Brain.Screen.setPenColor("#7A016B");

 }
void Values() {
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(black);
  Brain.Screen.setFont(mono15);
Brain.Screen.printAt(250, 20, "Battery Capacity: %d%%", Brain.Battery.capacity());
}


int autonToRun = 0;

class Button
{
  public:
    int x, y, width, height;
    std::string text;
    vex::color buttonColor, textColor;
    
    Button(int x, int y, int width, int height, std::string text, vex::color buttonColor, vex::color textColor)
    : x(x), y(y), width(width), height(height), text(text), buttonColor(buttonColor), textColor(textColor){}
   
    void render()
    {  
      Brain.Screen.drawRectangle(x, y, width, height, buttonColor);
      Brain.Screen.setFont(mono15);
      Brain.Screen.printAt(x + 10, y + 20, false, text.c_str());
    }

    bool isClicked()
    {
      if(Brain.Screen.pressing() && Brain.Screen.xPosition() >= x && Brain.Screen.xPosition() <= x + width &&
      Brain.Screen.yPosition() >= y && Brain.Screen.yPosition() <= y + width) return true;
      return false;
    }
};

Button autonButtons[] = {
  Button(0, 210, 20, 20, "R", green, yellow),// i = 0
  Button(230, 180, 100, 30, "Blue Single", green, yellow),// i = 4
  Button(230, 120, 100, 30, "Red Single", red, blue), // i = 1
  Button(360, 180, 100, 30, "Blue Double", red, blue), // i = 3
  Button(360, 120, 100, 30, "Red Double", green, yellow) // i = 2
  
  
};


void pre_auton()
{

    vex::color tan;
    tan.web("#D2B48C");

color yellow1;
    yellow1.web("#f0e637");

color red1;
    red1.web("#8a2121");

    color grey1;
    grey1.web("#494949");

    color blue1;
    blue1.web("#3762f0");


  while(true)
  {

Brain.Screen.clearScreen(black);
Values();
Picture();

    if(1)
    {
      
      for(int i = 0; i < 5; i++)
      {
        autonButtons[i].render();
        if(autonButtons[i].isClicked())
        {
          autonButtons[autonToRun].buttonColor = red1 ;
          autonButtons[i].buttonColor = blue1;
          Brain.Screen.printAt(250, 80,"%d",i);
          autonToRun = i;
          break;
        
        }
      }
    }

    Brain.Screen.render();
    
    vex::task::sleep(7);

}
}


void auton1()

{
  if (autonToRun == 1){
forwardInches(20, 80);
}

else if (autonToRun == 2){
backwardInches(20, 80);
  forwardInches(20, 80);
}

else if(autonToRun == 3)
{
  forwardInches(20, 80);
  backwardInches(20, 80);
}

else if(autonToRun == 4)
{
  forwardInches(20, 80);
  backwardInches(20, 80);
  forwardInches(20, 80);
}

else{
  
}

}








    






int main(){
Competition.autonomous(auton1);



   pre_auton();
}
  


    
