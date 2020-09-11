#include <ros.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <std_msgs/Float64MultiArray.h>

///////////////////////////////////////////////////////////
//               ! ! A T T E N S I O N ! !               //
// Works at Arduino MEGA 2560 unlike previous versions.  //
///////////////////////////////////////////////////////////

// Define pins that connect to stepper drivers
#define STEP_PIN 3

#define DIR_PIN 2

#define EN_PIN 4

int maxMotorSpeed = 2000;

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
  newPos = (long) (step_msg.data[0]*3200.0/3.0/3.1416);
  if(newPos > absolute){ // Set velocity of StepperA
    vel = step_msg.data[1]*3200.0/3.0/3.1416;
  }
  else{
    vel = -step_msg.data[1]*3200.0/3.0/3.1416;
  }

  absolute = newPos;
  setStepperPosAndSpeed();
  /*
  initialByte = Serial.read();
  if(initialByte == 0x71){
//  Package form: A1|FF_FF|FF_FF|
//                  |pos1 | v1  |
//                  |steps|5 step/s|
    packageSizeReceived = Serial.readBytes(serialChars, packageSizeMoveTo);
    serialBytes[0] = (byte)serialChars[0];
    serialBytes[1] = (byte)serialChars[1];
    serialBytes[2] = (byte)serialChars[2];
    serialBytes[3] = (byte)serialChars[3];
    printSerialPackage();

    if((packageSizeReceived == packageSizeMoveTo)){ 
      setPos();
      setVel();
    }
    setStepperPosAndSpeed();
  }
  
  Serial.print(initialByte);*/
}

ros::Subscriber<std_msgs::Float64MultiArray> sub("stepper_one_rev", &messageResponse );

//////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////  T H E    S E T U P  ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Configure each stepper
  stepperA.setMaxSpeed(maxMotorSpeed);
  stepperA.setEnablePin(EN_PIN);
  stepperA.setPinsInverted(false, false, true);
  stepperA.enableOutputs();

  // Then give them to MultiStepper to manage
  
  absolute = 3200;
  stepperA.setAcceleration(5000.0);
  stepperA.setSpeed(200);
  stepperA.moveTo(absolute);
  
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  T H E    L O O P  /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  nh.spinOnce();
  delay(1);
  stepperA.run(); // Need to be called frequently until all are in position
}

//////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////  T H E   F U N C T I O N S  /////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
void setVel(){
  if(newPos > absolute){ // Set velocity of StepperA
    vel = 0.2*(256*serialBytes[2] + serialBytes[3]);
  }
  else{
    vel = -0.2*(256*serialBytes[2] + serialBytes[3]);
  }

  absolute = newPos;
}

void setPos(){
  if(serialBytes[0] & 0x80){
    newPos = - 256*(serialBytes[0] & 0x7F) + serialBytes[1];
  }
  else{
    newPos = 256*serialBytes[0] + serialBytes[1];
  }
}

void setStepperPosAndSpeed(){  
  stepperA.moveTo(absolute);
  stepperA.setSpeed(vel);
}
//////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////  I N T E R R U P T   C A L L   F U N C T I O N S  //////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
