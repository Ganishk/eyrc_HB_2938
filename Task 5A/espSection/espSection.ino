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

/**
 * Common setups
 */

void setup(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);

//  WiFi.softAP(ssid,password);
//  xTaskCreatePinnedToCore(loop2,"loop2",1000,NULL,0,NULL,0);

  stepper1.setMaxSpeed(MAXSPEED);
  stepper2.setMaxSpeed(MAXSPEED);
  stepper3.setMaxSpeed(MAXSPEED);
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

void avr_section(String msg){
  /** 
   *  main loop of avr section
   */
  sscanf(msg.c_str(),"%f %f %f",&v1,&v2,&v3);
  fine();

}
