#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
extern brain Brain;

extern motor RFBASE;
extern motor LFBASE;
extern motor RBBASE;
extern motor LBBASE;
extern motor ARM;
extern motor RAIL;
extern gyro Gyro1;
extern pot Pot1;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}