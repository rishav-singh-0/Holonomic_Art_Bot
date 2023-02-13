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

Wheel wheel[3] = {{0, 0}, {0, 0}, {0, 0}};

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

  stepper_front.move(670.);
  stepper_left.move(670.);
  stepper_right.move(670.);

  get_f_wheels(1,1,1);

}

void loop() {
  
  run_speed();

  // static int count = 0;
  // count++;
  // if(count>10000){
  //   count = 0;
  //   Serial.print ("   ");
  //   Serial.print(stepper_front.speed());
  //   Serial.print ("   ");
  //   Serial.print(stepper_left.speed());
  //   Serial.print ("   ");
  //   Serial.print(stepper_right.speed());
  //   Serial.print ("   ");
  //   Serial.print ("\n");
  // }

  // checking if bot is ready to move
  static bool ready = false;
  if (!ready && ) {
    continue;
  }
  
  // validating conditions before moving
  if (!ready && !cond_f && !cond_l && !cond_r){
    ready = true;
  }

  // setting new speed for each wheel
  set_speed(
    wheel[FRONT].force % vel.maxSpeed,
    wheel[LEFT].force % vel.maxSpeed,
    wheel[RIGHT].force % vel.maxSpeed
  );

}

void set_speed(float f_front, float f_left, float f_right){
  /*
  Sets new speed
  */
    
  // Setting the speed
  stepper_front.setSpeed(wheel[FRONT].force);
  stepper_left.setSpeed(wheel[LEFT].force);
  stepper_right.setSpeed(wheel[RIGHT].force);
}

void run_speed(){
  /*
  Run the motors
  Store if motor has moved
  */

  wheel[FRONT].isRunning = stepper_front.runSpeed();
  wheel[LEFT].isRunning = stepper_left.runSpeed();
  wheel[RIGHT].isRunning = stepper_right.runSpeed();

}

void get_f_wheels( float x, float y, float w){

  wheel[FRONT].force  = (0.66667 * x) + (0 * y) + (-0.16667 * w);
  wheel[LEFT].force  = (-0.333 * x) + (0.577367 * y) + (-0.16667 * w);
  wheel[RIGHT].force  = (-0.333 * x) + (-0.577367 * y) + (-0.16667 * w);

}
