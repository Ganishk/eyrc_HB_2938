#include <WiFi.h>

// WiFi credentials
const char* ssid = "Alola";                    //Enter your wifi hotspot ssid
const char* password =  "theriyaathu";               //Enter your wifi hotspot password
const uint16_t port = 8002;
const char * host = "192.168.4.2";                   //Enter the ip address of your laptop after connecting it to wifi hotspot
String msg = "0";


WiFiClient client;

void avr_section(String);

void setup(){   
  Serial.begin(9600);                          //Serial to print data on Serial Monitor
//  Serial1.begin(115200,SERIAL_8N1,33,32);        //Serial to transfer data between ESP and AVR. The Serial connection is inbuilt.
  
  
  Serial.println("Setting AP ...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid,password);

  Serial.println();
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP Address: ");
  Serial.println(IP);
  xTaskCreatePinnedToCore(loop2,"loop2",1000,NULL,0,NULL,0);
}


void loop() {
  if (!client.connect(host, port)) {
    Serial.println("Connection to host failed");
    delay(200);
    return;
  }
  while(1){
      msg = client.readStringUntil('\n');
      msg.trim();
      //if (msg.length()) avr_section(msg);
  }
}

/**
 * 
 * AVR section codes
 *  
 */

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
  /** 
   *  main loop of avr section
   */
  sscanf(msg.c_str(),"%f %f %f",&v1,&v2,&v3);
  fine();
//  Serial.print(v1);
//  Serial.print(" ");
//  Serial.print(v2);
//  Serial.print(" ");
//  Serial.println(v3);
}
//bool flag = false;
void loop2(void* pvParameters){
//  if (flag=~flag){
//    pinMode(2,(flag)?HIGH:LOW);
//    sleep(1000);
//  }
  if (msg.length()) avr_section(msg);
}
