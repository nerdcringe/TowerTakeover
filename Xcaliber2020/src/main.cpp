/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// We can only include this file because all of its included files are automatically included in this file, and including them again will be a repeat definition
#include "driver.h"

// A global instance of competition
competition Competition;


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  Gyro1.startCalibration(5000);
  //selectionMenu();
}



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  //railToDeg(237, 80);
  //railToDeg(115, 80);
  //runSelectedAuto();
}



float sensitivity = 1;
float railSpeed = 80;

void railToDegDriver(double deg) {
  const float acceptableError = 4;
  double error = deg-RailPot.value(rotationUnits::deg); // Error = desired - actual measurement.;

  double railSpeed;

  while (fabs(error) > acceptableError) {

    error = deg-RailPot.value(rotationUnits::deg); // Error = desired - actual measurement.

    // Calculate rail speed so it moves up slower if rotated further than specified position, to increase precision of stacking.
    if (RailPot.value(rotationUnits::deg) <= RAIL_POT_DEACCEL && error < 0) {
      railSpeed = 50; // slower speed 
    } else {
      railSpeed = 100;
    }

    if (error <= 0)  {
      RAIL.spin(directionType::fwd, railSpeed, velocityUnits::pct);
    } else {
      RAIL.spin(directionType::rev, railSpeed, velocityUnits::pct);
    }
    
    Brain.Screen.printAt(0, 20, "%.2f, %.2f", RailPot.value(rotationUnits::deg), RAIL.rotation(rotationUnits::deg));
    Brain.Screen.printAt(0, 40, "Error: %.2f, Speed: %.2f", error, railSpeed);
    
    
    // Run driver control for other components to control robot from inside this loop.

    int leftSpeed = controllerPrim.Axis3.value() * sensitivity;
    int rightSpeed = controllerPrim.Axis2.value() * sensitivity;
    
    RFBASE.spin(fwd, rightSpeed, velocityUnits::pct);
    LFBASE.spin(fwd, leftSpeed, velocityUnits::pct);
    RBBASE.spin(fwd, rightSpeed, velocityUnits::pct);
    LBBASE.spin(fwd, leftSpeed, velocityUnits::pct);


    if (controllerPrim.ButtonL2.pressing())
    {
      LINTAKE.spin(directionType::fwd, 100, velocityUnits::pct);
      RINTAKE.spin(directionType::rev, 100, velocityUnits::pct);
    }
    else if (controllerPrim.ButtonL1.pressing())
    {
      LINTAKE.spin(directionType::rev, 100, velocityUnits::pct);
      RINTAKE.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    else if (LINTAKE.isSpinning() || RINTAKE.isSpinning()) 
    {
      LINTAKE.stop(brakeType::hold);
      RINTAKE.stop(brakeType::hold);
    }

    // Manual arm control (Scuf) - move arm for duration of button press.
    if (controllerPrim.ButtonDown.pressing())
    {
      ARM.spin(fwd, 100, velocityUnits::pct);
    }
    else if (controllerPrim.ButtonB.pressing())
    {
      ARM.spin(directionType::rev, 100, velocityUnits::pct);
    } else if (ARM.isSpinning()) {
      ARM.stop();
    }

    task::sleep(2);

  }
}



void armToDegDriver(double deg, int maxSpeed) {
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

    // Run driver control for other components to control robot from inside this loop.
    
    int leftSpeed = controllerPrim.Axis3.value() * sensitivity;
    int rightSpeed = controllerPrim.Axis2.value() * sensitivity;
    
    RFBASE.spin(fwd, rightSpeed, velocityUnits::pct);
    LFBASE.spin(fwd, leftSpeed, velocityUnits::pct);
    RBBASE.spin(fwd, rightSpeed, velocityUnits::pct);
    LBBASE.spin(fwd, leftSpeed, velocityUnits::pct);


 if (controllerPrim.ButtonL2.pressing())
    {
      LINTAKE.spin(directionType::fwd, 100, velocityUnits::pct);
      RINTAKE.spin(directionType::rev, 100, velocityUnits::pct);
    }
    else if (controllerPrim.ButtonL1.pressing())
    {
      LINTAKE.spin(directionType::rev, 100, velocityUnits::pct);
      RINTAKE.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    else
    {
      LINTAKE.stop(brakeType::hold);
      RINTAKE.stop(brakeType::hold);
    }


    // Calculate rail speed so it moves up slower if rotated further than specified position, to increase precision.
    if (RailPot.value(rotationUnits::deg) > RAIL_POT_DEACCEL) {
      railSpeed = 100;
    } else {
      railSpeed = 100;
    }

    // Move rail for duration that ButtonLeft or ButtonUp is pressed.
    if (controllerPrim.ButtonUp.pressing())
    {
        RAIL.spin(directionType::fwd, railSpeed, velocityUnits::pct);
    }
    else if (controllerPrim.ButtonLeft.pressing())
    {
      RAIL.spin(directionType::rev, 100, velocityUnits::pct);
    }
    else
    {
      RAIL.stop(brakeType::hold);
    }

  }

}


// Parameterless function callbacks to go to preset positions on button press.

void autoStackDriver() { railToDegDriver(POT_RAIL_STACK); }

void resetRailDriver() { railToDegDriver(POT_RAIL_RESET); }


void armLowDriver() { armToDegDriver(POT_ARM_LOW, 70); }

void armMidDriver() { armToDegDriver(POT_ARM_MID, 70); }

void armHighDriver() { armToDegDriver(POT_ARM_HIGH, 70); }



void setCallbacks() {
  
  // Set up callbacks for moving rail and arm to preset positions 
  controllerPrim.ButtonRight.pressed(autoStackDriver);
  controllerPrim.ButtonA.pressed(resetRailDriver);

  controllerPrim.ButtonY.pressed(armLowDriver);
  controllerPrim.ButtonR2.pressed(armMidDriver);
  controllerPrim.ButtonR1.pressed(armHighDriver);

}




/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/

int i = 0;
void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {

 //runDriver();



    int leftSpeed = controllerPrim.Axis3.value() * sensitivity;
    int rightSpeed = controllerPrim.Axis2.value() * sensitivity;
    
    RFBASE.spin(directionType::fwd, rightSpeed, velocityUnits::pct);
    LFBASE.spin(directionType::fwd, leftSpeed, velocityUnits::pct);
    RBBASE.spin(directionType::fwd, rightSpeed, velocityUnits::pct);
    LBBASE.spin(directionType::fwd, leftSpeed, velocityUnits::pct);



  if (controllerPrim.ButtonL2.pressing())
    {
      LINTAKE.spin(directionType::fwd, 100, velocityUnits::pct);
      RINTAKE.spin(directionType::rev, 100, velocityUnits::pct);
    }
    else if (controllerPrim.ButtonL1.pressing())
    {
      LINTAKE.spin(directionType::rev, 100, velocityUnits::pct);
      RINTAKE.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    else
    {
      LINTAKE.stop(brakeType::hold);
      RINTAKE.stop(brakeType::hold);
    }


    // Calculate rail speed so it moves up slower if rotated further than specified position, to increase precision.
    if (RailPot.value(rotationUnits::deg) > RAIL_POT_DEACCEL) {
      railSpeed = 100;
    } else {
      railSpeed = 50;
    }

    // Move rail for duration that ButtonLeft or ButtonUp is pressed.
    if (controllerPrim.ButtonUp.pressing())
    {
        RAIL.spin(directionType::fwd, railSpeed, velocityUnits::pct);
    }
    else if (controllerPrim.ButtonLeft.pressing())
    {
      RAIL.spin(directionType::rev, 100, velocityUnits::pct);
    }
    else
    {
      RAIL.stop(brakeType::hold);
    }


    // Manual arm control (Scuf) - move arm for duration of button press.
    if (controllerPrim.ButtonDown.pressing())
    {
      ARM.spin(fwd, 100, velocityUnits::pct);
    }
    else if (controllerPrim.ButtonB.pressing())
    {
      ARM.spin(directionType::rev, 100, velocityUnits::pct);
    } else {
      ARM.stop();
    }



    Brain.Screen.printAt(150, 60, "RailPot: %.2f", RailPot.value(rotationUnits::deg));

    Brain.Screen.printAt(150, 80, "i: %d", i++);
    controllerPrim.Screen.newLine();
    controllerPrim.Screen.print("Rail: %.2f, %.2f", RailPot.value(rotationUnits::deg), RAIL.rotation(rotationUnits::deg));

  }

  wait(20, msec); // Sleep the task for a short amount of time to
                  // prevent wasted resources.
}




int main() {

  LFBASE.setBrake(brakeType::coast);
  LBBASE.setBrake(brakeType::coast);
  RFBASE.setBrake(brakeType::coast);
  RBBASE.setBrake(brakeType::coast);
   
  RINTAKE.setBrake(brakeType::hold);
  LINTAKE.setBrake(brakeType::hold);

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  setCallbacks();
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
