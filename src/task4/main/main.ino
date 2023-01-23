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

  stepper_front.setMaxSpeed(3000);
  stepper_front.setAcceleration(1000);
  stepper_left.setMaxSpeed(3000);
  stepper_left.setAcceleration(1000);
  stepper_right.setMaxSpeed(3000);
  stepper_right.setAcceleration(1000);

}

void loop() {
  // put your main code here, to run repeatedly:
  // This loop is prefrablly made for making "L shape"


  // Giving constant Y velocity to travel vertical:
  l_y = 500;
  l_x = 0;
  w = 0

  get_f_wheels(l_x, l_y, w);

  // Setting the speed

  set_run(f_front, f_left, f_right);

  delay(4000);

  // Re-setting the speed

  set_run(0, 0, 0);

  delay(100);


  // Comming back

  l_y = -500;
  l_x = 0;
  w = 0

  get_f_wheels(l_x, l_y, w);

  // Setting the speed

  set_run(f_front, f_left, f_right);

  delay(4000);

  // Re-setting the speed

  set_run(0, 0, 0);

  delay(100);

  // Going to left side in x direction

  l_y = 0;
  l_x = -500;
  w = 0

  get_f_wheels(l_x, l_y, w);

  // Setting the speed

  set_run(f_front, f_left, f_right);

  delay(4000);

  // Re-setting the speed

  set_run(0, 0, 0);

  delay(10000);
  
}

void get_f_wheels( float x, float y, float w){

  f_front = (0.66667 * x) + (0 * y) + (-0.16667 * w);
  f_left = (-0.333 * x) + (0.577367 * y) + (-0.16667 * w);
  f_right = (-0.333 * x) + (-0.577367 * y) + (-0.16667 * w);

}

void set_run(float f_front, float f_left, float f_right){
    
  // Setting the speed

  stepper_front.setSpeed(f_front);
  stepper_left.setSpeed(f_left);
  stepper_right.setSpeed(f_right);

  // Giving the speed to motors
  
  stepper_front.runSpeed();
  stepper_left.runSpeed();
  stepper_right.runSpeed();
}

void reset_speed(){
  set_run(0, 0, 0);
}
