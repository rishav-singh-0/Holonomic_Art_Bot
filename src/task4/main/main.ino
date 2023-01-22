#include <AccelStepper.h>

// Defining direction and step pins for the stepper motor

int front_wheel_dir = 6;
int front_wheel_step = 7;

int left_wheel_dir = 8;
int left_wheel_step = 9;

int right_wheel_dir = 10;
int right_wheel_step = 11;

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
AccelStepper stepper_front(AccelStepper::DRIVER, front_wheel_step, front_wheel_dir); 
AccelStepper stepper_left(AccelStepper::DRIVER, left_wheel_step, left_wheel_dir); 
AccelStepper stepper_rigth(AccelStepper::DRIVER, right_wheel_step, right_wheel_dir);


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void get_f_wheels( float x, float y, float w){

  f_front = (0.6667 * x) + (0 * y) + (-0.1667 * 

  
}
