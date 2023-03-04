#include "AccelStepper.h"

const int MAXSPEED=2500;

AccelStepper stepper1(AccelStepper::DRIVER, 2,3);
AccelStepper stepper2(AccelStepper::DRIVER, 4,5);
AccelStepper stepper3(AccelStepper::DRIVER, 6,7);
float v1,v2,v3;

void setup(){
  stepper1.setMaxSpeed(MAXSPEED);
  stepper2.setMaxSpeed(MAXSPEED);
  stepper3.setMaxSpeed(MAXSPEED);
  Serial.begin(115200);
}

void fine(){
  // Send the signal to drive the motor.
  stepper1.setSpeed(v1);
  stepper2.setSpeed(v2);
  stepper3.setSpeed(v3);
  stepper1.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
}

void loop(){
  if (Serial.available()){
    msg = Serial.readStringUntil('\n');
    sscanf(msg.c_str(),"%f %f %f",&v1,&v2,&v3);
    fine();
  }
}
