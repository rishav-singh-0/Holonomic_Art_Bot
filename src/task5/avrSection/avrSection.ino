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
int maxSpeed = 600;
int maxAccleration = 100;
String rec_data = "0";

// Creating the objects for controlling the motors
AccelStepper stepper_front(AccelStepper::DRIVER, FRONT_WHEEL_STEP, FRONT_WHEEL_DIR); 
AccelStepper stepper_left(AccelStepper::DRIVER, LEFT_WHEEL_STEP, LEFT_WHEEL_DIR); 
AccelStepper stepper_right(AccelStepper::DRIVER, RIGHT_WHEEL_STEP, RIGHT_WHEEL_DIR);


void setup() {
  // put your setup code here, to run once:

  pinMode(RED,OUTPUT);
  pinMode(GREEN,OUTPUT);
  pinMode(BLUE,OUTPUT);

  stepper_front.setMaxSpeed(maxSpeed);
  stepper_front.setAcceleration(maxAccleration);
  stepper_left.setMaxSpeed(maxSpeed);
  stepper_left.setAcceleration(maxAccleration);
  stepper_right.setMaxSpeed(maxSpeed);
  stepper_right.setAcceleration(maxAccleration);

  stepper_front.move(0.);
  stepper_left.move(0.);
  stepper_right.move(0.);

  Serial.begin(115200);
}

void loop() {

  static bool ready = false;

  if(Serial.available()){                  //Check if any data is available on Serial
    rec_data = Serial.readStringUntil('\n');    //Read message on Serial until new char(\n) which indicates end of message. Received data is stored in msg

    char *buf = rec_data.c_str();
    // sscanf(buf, "{%f, %f, %f}\n", &wheel[FRONT].force, &wheel[LEFT].force, &wheel[RIGHT].force);

    wheel[FRONT].force = atof(strtok(buf+1, ","));
    wheel[LEFT].force = atof(strtok(NULL, ","));
    wheel[RIGHT].force = atof(strtok(NULL, ","));

    // Serial.print(rec_data.c_str());

    // Serial.print ("\n");
    // Serial.print(wheel[FRONT].force);
    // Serial.print("   ");
    // Serial.print(wheel[LEFT].force);
    // Serial.print("   ");
    // Serial.print(wheel[RIGHT].force);
    // Serial.print ("\n");
    ready = true;
  }
  else{
    ready = false;
  }

  // Printing the given speeds
  static int count = 1;
  count--;
  if(count==0){
    count = 1000000;
    Serial.print("   ");
    Serial.print(stepper_front.speed());
    Serial.print("   ");
    Serial.print(stepper_left.speed());
    Serial.print("   ");
    Serial.print(stepper_right.speed());
    Serial.print("   ");
    Serial.print ("\n");
  }

  // running the motors
  run_speed();

  // checking if bot is ready to move
  if (!ready) {
    return;
  }
  
  // validating conditions before moving
  // if (!wheel[FRONT].isRunning && !wheel[LEFT].isRunning && !wheel[RIGHT].isRunning){
  //   ready = true;
  // }

  // setting new speed for each wheel
  set_speed(wheel[FRONT].force, wheel[LEFT].force, wheel[RIGHT].force);

}

void set_speed(float f_front, float f_left, float f_right){
  /*
  Sets new speed
  */
    
  stepper_front.setSpeed(f_front);
  stepper_left.setSpeed(f_left);
  stepper_right.setSpeed(f_right);
  
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
