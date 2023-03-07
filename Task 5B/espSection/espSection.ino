/**
 * We had used ESP 32 ( Node MCU 32s ) module so it might be different from the origial eYFi ESP code
 */

#include <WiFi.h>

// WiFi credentials
const char* ssid = "RMI";                    //Enter your wifi hotspot ssid
const char* password =  "HAL9000T800";               //Enter your wifi hotspot password
const uint16_t port = 8002;
const char * host = "192.168.1.123";                   //Enter the ip address of your laptop after connecting it to wifi hotspot
String msg = "0";


WiFiClient client;

/**
 * AVR Presets
 */
void avr_section(String);


#include "AccelStepper.h"

const int MAXSPEED=2500;

AccelStepper stepper1(AccelStepper::DRIVER, 19,18);
AccelStepper stepper2(AccelStepper::DRIVER, 5,17);
AccelStepper stepper3(AccelStepper::DRIVER, 16,4);
float v1,v2,v3;
int penStatus,servoPin=26;

/**
 * Common setups
 */

void setup(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);
  analogWrite(26,1023);

//  WiFi.softAP(ssid,password);
//  xTaskCreatePinnedToCore(loop2,"loop2",1000,NULL,0,NULL,0);

  stepper1.setMaxSpeed(MAXSPEED);
  stepper2.setMaxSpeed(MAXSPEED);
  stepper3.setMaxSpeed(MAXSPEED);
  pinMode(servoPin,OUTPUT);
}


void loop() {
  if (!client.connect(host, port)) {
    delay(200);
    return;
  }
  while(1){
      msg = client.readStringUntil('\n');
      client.print(" ");
      //msg.trim();
      if (msg.length()) avr_section(msg);
  }
}


/**
 * 
 * AVR section codes
 *  
 */
void fine(){
  // Send the signal to drive the motor.
  stepper1.setSpeed(v1);
  stepper2.setSpeed(v2);
  stepper3.setSpeed(v3);
  stepper1.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
}
void switchPen(int pen){
  if (pen == 2){
    //Go up
    for (int i = 0;i<=200;i++){
      analogWrite(servoPin,i);
      delay(5);
    }
  } else if (pen==1){
    //Go down
    for (int i = 200;i>=0;i--){
      analogWrite(servoPin,i);
      delay(5);
    }
  }
}
void avr_section(String msg){
  /** 
   *  main loop of avr section
   */
  sscanf(msg.c_str(),"%f %f %f %d",&v1,&v2,&v3,&penStatus);
  if (penStatus){switchPen(penStatus);return;}
  fine();

}
