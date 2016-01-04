#include <Adafruit_Sensor.h>
#include <Adafruit_10DOF.h>
#include <Wire.h>
#include <Adafruit_LSM303_U.h> //Accel + Mag
#include <Adafruit_L3GD20.h>   //Also Gyro
#include <Adafruit_L3GD20_U.h> //Gyro
#include <Adafruit_BMP085_U.h> //Barometer


int now = 0;
int last = 0;

//////////////////////////////////////////////
/////////////////DECLARE SENSORS//////////////
//////////////////////////////////////////////

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

//////////////////////////////////////////////
//////////////////////SETUP///////////////////
//////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  Serial.println("Accelerometer Test"); Serial.println("");

  gyro.enableAutoRange(true);
  if (!gyro.begin())
  {
    Serial.println("Oops ... unable to initialize the GYRO. Check your wiring!");
    while (1);
  }

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ACCELEROMETER detected ... Check your wiring!");
    while(1);
  }
}

void loop() {
  now = millis();
    /* Get a new sensor event */ 
  sensors_event_t event; 
  gyro.getEvent(&event);
 
  /* Display the results (speed is measured in rad/s) */
  Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");
  Serial.println("rad/s ");
  //delay(500);

  sensors_event_t event2; 
  accel.getEvent(&event2);
 
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event2.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event2.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event2.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  //delay(500);
  //last = millis();
  Serial.println(now - last);
  last = now;
}
