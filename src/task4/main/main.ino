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
int multiplier = 1000;

// Creating the objects for controlling the motors
AccelStepper stepper_front(AccelStepper::DRIVER, FRONT_WHEEL_STEP, FRONT_WHEEL_DIR); 
AccelStepper stepper_left(AccelStepper::DRIVER, LEFT_WHEEL_STEP, LEFT_WHEEL_DIR); 
AccelStepper stepper_right(AccelStepper::DRIVER, RIGHT_WHEEL_STEP, RIGHT_WHEEL_DIR);


void setup() {
  // put your setup code here, to run once:

  stepper_front.setMaxSpeed(1000.);
  stepper_front.setAcceleration(100.);
  stepper_left.setMaxSpeed(1000.);
  stepper_left.setAcceleration(100.);
  stepper_right.setMaxSpeed(1000.);
  stepper_right.setAcceleration(100.);

  stepper_front.moveTo(650.);
  stepper_left.moveTo(650.);
  stepper_right.moveTo(650.);

  get_f_wheels(0,1,0);

}

void loop() {
  // put your main code here, to run repeatedly:

  // set_speed(0, 200, -200);
  boolean cond_f = stepper_front.run(), cond_l = stepper_left.run(), cond_r = stepper_right.run();

  if (!cond_f) {   // run() returns true as long as the final position has not been reached and speed is not 0.
    // stepper_front.moveTo(-stepper_front.currentPosition());
    // f_front = -f_front;
    stepper_front.moveTo(f_front*multiplier);
  }

  if (!cond_l && !cond_r) {   
    // run() returns true as long as the final position has not been reached and
    // speed is not 0.
    // f_left = -f_left;
    stepper_left.moveTo(f_left*multiplier);
    stepper_right.moveTo(f_right*multiplier);
  }
  // if (!stepper_right.run()) {   // run() returns true as long as the final position has not been reached and speed is not 0.
  //   // f_right = -f_right;
  //   stepper_right.moveTo(f_right*multiplier);
  // }
  // take_step();
  
}

void set_speed(float f_front, float f_left, float f_right){
    
  // Setting the speed

  stepper_front.setSpeed(f_front);
  stepper_left.setSpeed(f_left);
  stepper_right.setSpeed(f_right);

}

void take_step(){

  // Giving the speed to motors
  
  stepper_front.run();
  stepper_left.run();
  stepper_right.run();
}

void get_f_wheels( float x, float y, float w){

  f_front = (0.66667 * x) + (0 * y) + (-0.16667 * w);
  f_left = (-0.333 * x) + (0.577367 * y) + (-0.16667 * w);
  f_right = (-0.333 * x) + (-0.577367 * y) + (-0.16667 * w);

}
