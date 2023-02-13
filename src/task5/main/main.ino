#include <AccelStepper.h>
#include "pins.h"

// Task 5

// Creating the motor force variables

enum WHEEL {FRONT, LEFT, RIGHT};

typedef struct{
  float force;
  bool isRunning;
} Wheel;

// initializing velocity variables 

Wheel wheel[3] = {{0, 0}, {0, 0}, {0, 0}};
maxSpeed = 600;
maxAccleration = 100;

// Creating the objects for controlling the motors
AccelStepper stepper_front(AccelStepper::DRIVER, FRONT_WHEEL_STEP, FRONT_WHEEL_DIR); 
AccelStepper stepper_left(AccelStepper::DRIVER, LEFT_WHEEL_STEP, LEFT_WHEEL_DIR); 
AccelStepper stepper_right(AccelStepper::DRIVER, RIGHT_WHEEL_STEP, RIGHT_WHEEL_DIR);


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  stepper_front.setMaxSpeed(maxSpeed);
  stepper_front.setAcceleration(maxAccleration);
  stepper_left.setMaxSpeed(maxSpeed);
  stepper_left.setAcceleration(maxAccleration);
  stepper_right.setMaxSpeed(maxSpeed);
  stepper_right.setAcceleration(maxAccleration);

  stepper_front.move(670.);
  stepper_left.move(670.);
  stepper_right.move(670.);

}

void loop() {
  
  // Printing the given speeds
  static int count = 1;
  count--;
  if(count==0){
    count = 10000;
    Serial.print ("   ");
    Serial.print(stepper_front.speed());
    Serial.print ("   ");
    Serial.print(stepper_left.speed());
    Serial.print ("   ");
    Serial.print(stepper_right.speed());
    Serial.print ("   ");
    Serial.print ("\n");
  }

  // running the motors
  run_speed();

  // checking if bot is ready to move
  static bool ready = false;
  if (!ready && ) {
    continue;
  }
  
  // validating conditions before moving
  if (!ready && !wheel[FRONT].isRunning && !wheel[LEFT].isRunning && !wheel[RIGHT].isRunning){
    ready = true;
  }

  // setting new speed for each wheel
  set_speed(
    wheel[FRONT].force % maxSpeed,
    wheel[LEFT].force % maxSpeed,
    wheel[RIGHT].force % maxSpeed
  );

}

void set_speed(float f_front, float f_left, float f_right){
  /*
  Sets new speed
  */
    
  stepper_front.setSpeed(wheel[FRONT].force);
  stepper_left.setSpeed(wheel[LEFT].force);
  stepper_right.setSpeed(wheel[RIGHT].force);
}

void run_speed(){
  /*
  Run the motors
  Stores true if the motor was stepped for each motor
  */

  wheel[FRONT].isRunning = stepper_front.runSpeed();
  wheel[LEFT].isRunning = stepper_left.runSpeed();
  wheel[RIGHT].isRunning = stepper_right.runSpeed();

}
