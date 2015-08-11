
#include <Servo.h>


int value = 40; // set values you need to zero

Servo m1; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time

void setup() {

  m1.attach(9);    // attached to pin 9 I just do this with 1 Servo
  Serial.begin(9600);    // start serial at 9600 baud
  m1.write(value);
}

void loop() {

//First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions
 
  m1.write(value);
 
  if(Serial.available()) 
    value = Serial.parseInt();    // Parse an Integer from Serial

}

