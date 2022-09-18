#include <ros.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <std_msgs/Float64MultiArray.h>

// Define pins that connect to stepper drivers
#define STEP1_PIN 8//2
#define DIR1_PIN 9//3
#define EN1_PIN 10//6
#define EN2_PIN 11//7
#define STEP2_PIN 13//4
#define DIR2_PIN 12//5

int maxMotorSpeed = 10000;

long newPos[2];
volatile long absolute[2]; // Array of desired stepper positions
float vel[2];

AccelStepper stepperA(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);
AccelStepper stepperB(AccelStepper::DRIVER, STEP2_PIN, DIR2_PIN);

ros::NodeHandle nh;

void messageResponse(const std_msgs::Float64MultiArray& step_msg)
{
  float rad2step0 = 32000.0/9.0/3.1416;
  float rad2step1 = 13200.0/3.1416;
  newPos[0] = (long)(step_msg.data[0]*rad2step0);
  if(newPos[0] > absolute[0]){ // Set velocity of StepperA
    vel[0] = step_msg.data[1]*rad2step0;
  }
  else{
    vel[0] = -step_msg.data[1]*rad2step0;
  }
  
  newPos[1] = (long)(step_msg.data[2]*rad2step1);
  if(newPos[1] > absolute[1]){ // Set velocity of StepperB
    vel[1] = step_msg.data[3]*rad2step1;
  }
  else{
    vel[1] = -step_msg.data[3]*rad2step1;
  }

  absolute[0] = newPos[0];
  absolute[1] = newPos[1];
  setStepperPosAndSpeed();
}

ros::Subscriber<std_msgs::Float64MultiArray> sub("stepper_go", &messageResponse );

/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////  T H E    S E T U P  ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(13, OUTPUT);
  // Configure each stepper
  stepperA.setMaxSpeed(maxMotorSpeed);
  stepperA.setEnablePin(EN1_PIN);
  stepperA.setPinsInverted(false, false, false);
  stepperA.enableOutputs();
  
  stepperB.setMaxSpeed(maxMotorSpeed);
  stepperB.setEnablePin(EN2_PIN);
  stepperB.setPinsInverted(true, false, false);
  stepperB.enableOutputs();

  // Then give them to MultiStepper to manage
  
  absolute[0] = 0;
  absolute[1] = 0;
  stepperA.setAcceleration(100000.0);
  stepperA.setSpeed(200);
  stepperA.moveTo(absolute[0]);
  stepperB.setAcceleration(100000.0);
  stepperB.setSpeed(200);
  stepperB.moveTo(absolute[1]);
  
  nh.initNode();
  nh.subscribe(sub);
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  T H E    L O O P  ////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  nh.spinOnce();
  delayMicroseconds(1);
  stepperA.runSpeedToPosition(); // Need to be called frequently until all are in position
  stepperB.runSpeedToPosition(); // Need to be called frequently until all are in position
}

/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////  T H E   F U N C T I O N S  ////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

void setStepperPosAndSpeed(){
  stepperA.moveTo(absolute[0]);
  stepperA.setSpeed(vel[0]);
  stepperB.moveTo(absolute[1]);
  stepperB.setSpeed(vel[1]);
}
