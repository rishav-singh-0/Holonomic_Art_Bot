/*
* Team Id: HB#1254
* Author List: Rishav
* Filename: avrSection.ino
* Theme: Hola Bot -- Specific to eYRC 2022-23
* Functions: setup(), loop(), set_speed(), run_speed()
* Global Variables: 
*    wheel, maxSpeed, maxAccleration, recData, stepper, servo_obj, servo_pos, up_pos, down_pos
*/

#include <AccelStepper.h>
#include <Servo.h>
#include "pins.h"

// Task 5

// Creating the motor force variables

enum WHEEL {FRONT, LEFT, RIGHT};

typedef struct{
  float force;
  bool isRunning;
} Wheel;

// initializing velocity variables 

Wheel wheel[3] = {{0, 0}, {0, 0}, {0, 0}};    // to store wheel running status and its velocity
int maxSpeed = 600;                           // maximum allowed speed
int maxAccleration = 100;                     // maximum allowed accleration
String recData = "0";                         // storing received data from espSection

// Creating the objects for controlling the motors
AccelStepper stepper_front(AccelStepper::DRIVER, FRONT_WHEEL_STEP, FRONT_WHEEL_DIR); 
AccelStepper stepper_left(AccelStepper::DRIVER, LEFT_WHEEL_STEP, LEFT_WHEEL_DIR); 
AccelStepper stepper_right(AccelStepper::DRIVER, RIGHT_WHEEL_STEP, RIGHT_WHEEL_DIR);

Servo servo_obj;        // create servo object to control a servo
bool servo_pos = 0;     // up position(do not draw)
int up_pos = 10;        // variable to store the servo position
int down_pos = 95;      // variable to store the servo position

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

  servo_obj.attach(SERVO);

  Serial.begin(115200);
}

void loop() {

  static bool ready = false;

  //Check if any data is available on Serial
  if(Serial.available()){                  

    //Read message on Serial until new char(\n) which indicates end of message. Received data is stored in msg
    recData = Serial.readStringUntil('\n');

    char *buf = recData.c_str();

    wheel[FRONT].force = atof(strtok(buf+1, ","));
    wheel[LEFT].force = atof(strtok(NULL, ","));
    wheel[RIGHT].force = atof(strtok(NULL, ","));
    servo_pos = atoi(strtok(NULL, ","));

    ready = true;
  }
  else{
    ready = false;
  }

  // Printing the given speeds for debugging
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
    Serial.print(servo_pos);
    Serial.print ("\n");
  }

  if(servo_pos == 1){
    servo_obj.write(down_pos);         // tell servo to go to position in variable 'down_pos'
  }
  else{
    servo_obj.write(up_pos);           // tell servo to go to position in variable 'up_pos'
  }

  // running the motors
  run_speed();

  // checking if bot is ready to move
  if (!ready) {
    return;
  }

  // setting new speed for each wheel
  set_speed();

}
 
/*
* Function Name: set_speed
* Logic: sets the new wheel speed to the AccelStepper objects of each wheel
* Example Call: 
*   set_speed()
*/ 
void set_speed(){
  
  stepper_front.setSpeed(wheel[FRONT].force);
  stepper_left.setSpeed(wheel[LEFT].force);
  stepper_right.setSpeed(wheel[RIGHT].force);
  
}

/*
* Function Name: run_speed
* Logic: Runs the motors and stores true if the motor was stepped for each motor
* Example Call: 
*   run_speed()
*/ 
void run_speed(){

  wheel[FRONT].isRunning = stepper_front.runSpeed();
  wheel[LEFT].isRunning = stepper_left.runSpeed();
  wheel[RIGHT].isRunning = stepper_right.runSpeed();

}
