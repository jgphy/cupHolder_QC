#include <Adafruit_L3GD20.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <math.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

int pin  = 3;
int pin2 = 5;
int pin3 = 6;//in use
int pin4 = 9;
int pin5 = 10;//in use
int pin6 = 11;//in use

int motor1 = 2; //pin
int motor2 = 4;
int motor3 = 12;
int motor4 = 13;

const int channel1Min = 995;
const int channel1Max = 1984;

const int channel2Min = 990;
const int channel2Max = 1984;

const int channel3Min = 990;
const int channel3Max = 1984;

const int channel4Min = 1018;
const int channel4Max = 1984;

const int channel5Min = 989;
const int channel5Max = 1984;

const int channel6Min = 989;
const int channel6Max = 1984;


void setup() {

  analogReference(EXTERNAL);
  Serial.begin(9600);
  //setup input pins
  pinMode(pin, INPUT);
  pinMode(pin2, INPUT);
  pinMode(pin3, INPUT);
  pinMode(pin4, INPUT);
  //setup output pins
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);



  //makes sure that the gyroscope is plugged in
  //We should think about adding a screen somewhere on the quadcopter to diagnose errors like this
  /*if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  {
    Serial.println("You fucked up the wiring for the gyro...bitch");
    while (1);
  }
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections
    Serial.println("Ooops, no LSM303 detected ... Check your wiring...bitch!");
    while(1);
  }
  */

  sensor_t sensor;
  accel.getSensor(&sensor);
  accelMax = sensor.max_value;
  accelMin = sensor.max_value;

  //arming the motors
  for (Arming_Time = 0; Arming_Time < 500; Arming_Time += 1)
  {
    digitalWrite(motor1,HIGH);
    digitalWrite(motor2,HIGH);
    digitalWrite(motor3,HIGH);
    digitalWrite(motor4,HIGH);
    delayMicroseconds(1100);
    digitalWrite(motor1,LOW);
    digitalWrite(motor2,LOW);
    digitalWrite(motor3,LOW);
    digitalWrite(motor4,LOW);
    delay(20-(Pulse/1000));

  }


}


void loop() {
  channelVal1= pulseIn(pin,HIGH);
  channelVal2= pulseIn(pin2, HIGH);
  channelVal3= pulseIn(pin3, HIGH);   //throttle
  channelVal4= pulseIn(pin4, HIGH);
  channelVal5= pulseIn(pin5, HIGH);  //kp value for pitch angle PID
  channelVal6= pulseIn(pin6, HIGH);  //kp value for roll angle  PID

  channelVal1= map(channelVal1,channel1Min,channel1Max,1100,2000);
  channelVal1= constrain(channelVal1,1100,2000);

  channelVal2= map(channelVal2,channel2Min,channel2Max,1100,2000);
  channelVal2= constrain(channelVal2,1100,2000);

  channelVal3= map(channelVal3,channel3Min,channel3Max,1100,2000);
  channelVal3= constrain(channelVal3,1100,2000);

  channelVal4= map(channelVal4,channel4Min,channel4Max,1100,2000);
  channelVal4= constrain(channelVal4,1100,2000);

  channelVal5 = map(channelVal5, channel5Min, channel5Max, 0, 2);   //0 to 2 are just numbers that i picked, probably need to manually check these
  channelVal5 = constrain(channelVal5, 0, 2);
  channelVal6 = map(channelVal6, channel6Min, channel6Max, 0, 2);
  channelVal6 = constrain(channelVal6, 0, 2);

  digitalWrite(motor1, HIGH);
  delayMicroseconds(channelVal1);
  digitalWrite(motor1, LOW);

  digitalWrite(motor2, HIGH);
  delayMicroseconds(channelVal2);
  digitalWrite(motor2, LOW);

  digitalWrite(motor3, HIGH);
  delayMicroseconds(channelVal3);
  digitalWrite(motor3, LOW);

  digitalWrite(motor4, HIGH);
  delayMicroseconds(channelVal4);
  digitalWrite(motor4, LOW);




}
