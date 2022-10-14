#include "autoSelector.h"



float sensitivity = 0.65;
float railSpeed = 80;


void controlBase() {

    // Base controls - wheel speed matches controller input for respective side
    int leftSpeed = controllerPrim.Axis3.value() * sensitivity;
    int rightSpeed = controllerPrim.Axis3.value() * sensitivity;
    
    RFBASE.spin(fwd, rightSpeed, velocityUnits::pct);
    LFBASE.spin(fwd, leftSpeed, velocityUnits::pct);
    RBBASE.spin(fwd, rightSpeed, velocityUnits::pct);
    LBBASE.spin(fwd, leftSpeed, velocityUnits::pct);
    
}



void controlRail() {
  
    // Calculate rail speed so it moves up slower if rotated further than specified position, to increase precision.
    if (RailPot.value(rotationUnits::deg) < RAIL_POT_DEACCEL) {
      railSpeed = 80;
    } else {
      railSpeed = 45;
    }

    // Move rail for duration that ButtonLeft or ButtonUp is pressed.
    if (controllerPrim.ButtonUp.pressing())
    {
        RAIL.spin(fwd, railSpeed, velocityUnits::pct);
    }
    else if (controllerPrim.ButtonLeft.pressing())
    {
      RAIL.startRotateTo(0, rotationUnits::deg, 100, velocityUnits::pct);
    }
    else
    {
      RAIL.stop(brakeType::hold);
    }

}



void controlArm() {

    // Manual arm control (Scuf) - move arm for duration of button press.
    if (controllerPrim.ButtonDown.pressing())
    {
      ARM.spin(fwd, 100, velocityUnits::pct);
    }
    else if (controllerPrim.ButtonB.pressing())
    {
      ARM.spin(directionType::rev, 100, velocityUnits::pct);
    }

}



void controlIntake() {
  
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

}






void railToDegDriver(double deg, int maxSpeed) {
   const float initialError = deg-RailPot.value(rotationUnits::deg); // Error = desired - actual mesasurement.

  float error = initialError;
  const float acceptableError = 3; // How far off from intended degree value that it's ok to stop at

  const int minSpeed = 3;
  const float deaccelRate = 1.5;
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

    // Run driver control for other components so robot can be controlled from inside this loop.
    controlBase();
    controlIntake();
    controlArm();
    

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
    controlBase();
    controlIntake();
    controlRail();

  }

}



// Parameterless function callbacks to go to preset positions on button press.
void autoStackDriver() { railToDegDriver(POT_RAIL_STACK, 65); }

void resetRailDriver() { railToDegDriver(0, 85); }


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



void runDriver() {

  controlBase();
  controlRail();
  controlArm();
  controlIntake();


}