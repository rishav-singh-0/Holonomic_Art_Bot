#include <AccelStepper.h>
#include "pins.h"

// Creating the motor force variables

float f_front = 0;
float f_left = 0;
float f_right = 0;

// local velocity variables 

float l_x = 0;
float l_y = 0;

// global velocities veriables

float g_x = 0;
float g_y =0;

// angular position will be same for both frame local and global frame

float w = 0;

// Creating the objects for controlling the motors
AccelStepper stepper_front(AccelStepper::DRIVER, FRONT_WHEEL_STEP, FRONT_WHEEL_DIR); 
AccelStepper stepper_left(AccelStepper::DRIVER, LEFT_WHEEL_STEP, LEFT_WHEEL_DIR); 
AccelStepper stepper_rigth(AccelStepper::DRIVER, RIGHT_WHEEL_STEP, RIGHT_WHEEL_DIR);


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void get_f_wheels( float x, float y, float w){

  f_front = (0.66667 * x) + (0 * y) + (-0.16667 * w);
  f_left = (-0.333 * x) + (0.577367 * y) + (-0.16667 * w);
  f_right = (-0.333 * x) + (-0.577367 * y) + (-0.16667 * w);

}
