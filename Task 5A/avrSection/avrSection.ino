#include "AccelStepper.h"

AccelStepper stepper1(AccelStepper::DRIVER, 19,18);
AccelStepper stepper2(AccelStepper::DRIVER, 5,17);
AccelStepper stepper3(AccelStepper::DRIVER, 16,4);
float v1,v2,v3;


void fine(){
  // Send the signal to drive the motor.
  stepper1.setSpeed(v1);
  stepper2.setSpeed(v2);
  stepper3.setSpeed(v3);
  unsigned long prev_timer = millis();
  while(millis() - prev_timer < 3000){
    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();
  }
}

void avr_section(String msg){

  sscanf(msg.c_str(),"%f %f %f",&v1,&v2,&v3);
  fine();

}

//bool flag = false;
void loop2(void* pvParameters){
//  if (flag=~flag){
//    pinMode(2,(flag)?HIGH:LOW);
//    sleep(1000);
//  }

  if (msg.length()) avr_section(msg);

}
