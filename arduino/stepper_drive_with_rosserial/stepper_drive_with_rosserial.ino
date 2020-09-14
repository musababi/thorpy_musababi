#include <ros.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <std_msgs/Float64MultiArray.h>

// Define pins that connect to stepper drivers
#define STEP_PIN 3
#define DIR_PIN 2
#define EN_PIN 4

int maxMotorSpeed = 10000;

// Serial package parameters
const byte packageSizeMoveTo = 4;
byte packageSizeReceived;
char serialChars[4];
byte serialBytes[4];
byte setZeroByte;
byte checkSum;
byte initialByte;

long newPos;
volatile long absolute; // Array of desired stepper positions
float vel;

AccelStepper stepperA(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

ros::NodeHandle nh;

void messageResponse(const std_msgs::Float64MultiArray& step_msg)
{
  float rad2step = 3200.0/3.0/3.1416;
  newPos = (long)(step_msg.data[0]*rad2step);
  if(newPos > absolute){ // Set velocity of StepperA
    vel = step_msg.data[1]*rad2step;
  }
  else{
    vel = -step_msg.data[1]*rad2step;
  }

  absolute = newPos;
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
  stepperA.setEnablePin(EN_PIN);
  stepperA.setPinsInverted(true, false, true);
  stepperA.enableOutputs();

  // Then give them to MultiStepper to manage
  
  absolute = 0;
  stepperA.setAcceleration(100000.0);
  stepperA.setSpeed(200);
  stepperA.moveTo(absolute);
  
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
}

/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////  T H E   F U N C T I O N S  ////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

void setStepperPosAndSpeed(){
  stepperA.moveTo(absolute);
  stepperA.setSpeed(vel);
}
