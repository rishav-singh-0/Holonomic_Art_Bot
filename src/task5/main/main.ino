#include <AccelStepper.h>
#include "pins.h"

// Task 5

// Creating the motor force variables

enum WHEEL {FRONT, LEFT, RIGHT};

typedef struct{
  float force;
  bool isRunning;
} Wheel;

typedef struct {
  float local_x;
  float local_y;
  float global_x;
  float globla_y;

  // angular position will be same for both frame local and global frame
  float w;

  // max speed and acc
  float maxSpeed;
  float maxAccleration;
} Velocity;

float force[3] = {0, 0, 0};

// initializing velocity variables 

Velocity vel = {
  .local_x = 0,
  .local_y = 0,
  .global_x = 0,
  .globla_y = 0,
  .w = 0,
  .maxSpeed = 600,
  .maxAccleration = 100,
};

// Creating the objects for controlling the motors
AccelStepper stepper_front(AccelStepper::DRIVER, FRONT_WHEEL_STEP, FRONT_WHEEL_DIR); 
AccelStepper stepper_left(AccelStepper::DRIVER, LEFT_WHEEL_STEP, LEFT_WHEEL_DIR); 
AccelStepper stepper_right(AccelStepper::DRIVER, RIGHT_WHEEL_STEP, RIGHT_WHEEL_DIR);


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  stepper_front.setMaxSpeed(vel.maxSpeed);
  stepper_front.setAcceleration(vel.maxAccleration);
  stepper_left.setMaxSpeed(vel.maxSpeed);
  stepper_left.setAcceleration(vel.maxAccleration);
  stepper_right.setMaxSpeed(vel.maxSpeed);
  stepper_right.setAcceleration(vel.maxAccleration);

  stepper_front.move(0.);
  stepper_left.move(0.);
  stepper_right.move(0.);

  get_f_wheels(1,1,1);

}

void loop() {
  // put your main code here, to run repeatedly:

  boolean cond_f = stepper_front.runSpeed(), cond_l = stepper_left.runSpeed(), cond_r = stepper_right.runSpeed();

  static int count = 0;
  count++;
  if(count>10000){
    count = 0;
    Serial.print ("   ");
    Serial.print(stepper_front.speed());
    Serial.print ("   ");
    Serial.print(stepper_left.speed());
    Serial.print ("   ");
    Serial.print(stepper_right.speed());
    Serial.print ("   ");
    Serial.print ("\n");
  }
    

  // run() returns true as long as the final position has not been reached and
  // speed is not 0.

  bool ready = false;
  if (!ready && !cond_f && !cond_l && !cond_r) {
    ready = true;
  }
    // f_left = -f_left;
    // stepper_left.move(500);
    // stepper_right.move(500);

    stepper_front.setSpeed(force[FRONT]*vel.maxSpeed);
    stepper_left.setSpeed(force[LEFT]*vel.maxSpeed);
    stepper_right.setSpeed(force[RIGHT]*vel.maxSpeed);

    // int s1 =  stepper_front.speed();
    // int s2 =  stepper_left.speed();
    // int s3 =  stepper_right.speed();
    // int avg = (s1+s2+s3)/3;

    // stepper_front.setAcceleration((avg*s1*s1)%200);
    // stepper_left.setAcceleration((avg*s2*s2)%200);
    // stepper_right.setAcceleration((avg*s3*s3)%200);
    // stepper_front.setAcceleration(100);
    // stepper_left.setAcceleration(100);
    // stepper_right.setAcceleration(100);
  
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

  force[FRONT] = (0.66667 * x) + (0 * y) + (-0.16667 * w);
  force[LEFT] = (-0.333 * x) + (0.577367 * y) + (-0.16667 * w);
  force[RIGHT] = (-0.333 * x) + (-0.577367 * y) + (-0.16667 * w);

}
