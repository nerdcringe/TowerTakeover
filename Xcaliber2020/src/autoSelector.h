#include "vex.h"

#include "auton.h"



#define LEFT_ARROW  247
#define RIGHT_ARROW 246
static  char l_arr_str[4] = { LEFT_ARROW,  LEFT_ARROW,  LEFT_ARROW,  0};
static  char r_arr_str[4] = { RIGHT_ARROW, RIGHT_ARROW, RIGHT_ARROW, 0};

static int MyAutonomous = 0;
#define MAX_CHOICE  3


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


void selectionMenu()
{

    color tan;
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


void runSelectedAuto()
{
  if (autonToRun == 1){
  forwardInches(20, 80);

  redSingle();

  }

  else if (autonToRun == 2)
  {
    backwardInches(20, 80);
    forwardInches(20, 80);
    
    redDouble();
  }

  else if(autonToRun == 3)
  {
    forwardInches(20, 80);
    backwardInches(20, 80);

    blueDouble();
  }

  else if(autonToRun == 4)
  {
    forwardInches(20, 80);
    backwardInches(20, 80);
    forwardInches(20, 80);

    blueSingle();
  }

  else {
    
  }

}