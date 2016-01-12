#include <Adafruit_L3GD20_U.h>

#include <Adafruit_L3GD20.h>
//#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <math.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

int dt = 10;//

const int sampleSize = 10;
int sampleRate= 1000; //one second


int pin  = 3;
int pin2 = 5;
int pin3 = 6;//in use
int pin4 = 9;
int pin5 = 10;//in use
int pin6 = 11;//in use
double oldChannelVal3=1100;


double xGyro = 0;
double yGyro = 0;
double zGyro = 0;



const double hoverAngle = 0;
double lastPErr = 0;
double pErrSum = 0;
double pitchOut = 0;
double kp = 1;

double lastRErr = 0;
double rErrSum = 0;
double rollOut = 0;
double kp2 = 1;

const float pi=3.14159;

unsigned long lastTime = 0;
unsigned long now;
unsigned long late;
unsigned long temp;

double xRaw = 0;
double yRaw = 0;
double zRaw = 0;

double accelMax;
double accelMin;

double w_1 = 1100;
double w_2 = 1100;
double w_3 = 1100;
double w_4 = 1100;
double w   = 1100;

//Pulled out of the loop to decrease time
long xScaled = map(xRaw, accelMin, accelMax, -1000, 1000);
  //might have to change this -1000 to 1000 cause i don't think its actually going to give us G's
  //actually i don't think it'll matter, but it's late at this point
long yScaled = map(yRaw, accelMin, accelMax, -1000, 1000);
long zScaled = map(zRaw, accelMin, accelMax, -1000, 1000);

float xAccel = xScaled / 1000.0;
float yAccel = yScaled / 1000.0;
float zAccel = zScaled / 1000.0;


double channelVal1;
double channelVal2;
double channelVal3;
double channelVal4;
double channelVal5;
double channelVal6;

double throttleIn = 1100;

double minChange=50;

double tiltangle = 0;
double pitchAngle; 
double rollAngle;

double pitchDeg;
double yawDeg;
double finalPitchAngle;
double pError;
double dPErr;

double rollDeg;
double finalRollAngle;
double rError;
double dRErr;
double change;

int skip = 0;

int tempo = 0;

void setup() {

  analogReference(EXTERNAL);
  Serial.begin(9600);
  //setup input pins
  pinMode(pin, INPUT);
  pinMode(pin2, INPUT);
  pinMode(pin3, INPUT);
  pinMode(pin4, INPUT);
 

 // //Serial.println("Setting up...");

  //makes sure that the gyroscope is plugged in
  //We should think about adding a screen somewhere on the quadcopter to diagnose errors like this
  gyro.enableAutoRange(true);
  if (!gyro.begin())
  {
    //Serial.println("Oops ... unable to initialize the GYRO. Check your wiring!");
    while (1);
  }

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    //Serial.println("Ooops, no ACCELEROMETER detected ... Check your wiring!");
    while(1);
  }

  sensor_t sensor;
  accel.getSensor(&sensor);
  accelMax = sensor.max_value;
  accelMin = sensor.max_value;

  //delay(1000);
  //Serial.println("Starting Loop");
}



void loop() {
  now = millis();

  
  //////////////////// SENSOR EVENT SETUP /////////////////
  ////////////////////// V IMPORTANT //////////////////////
  ////////////////////// DONT DELETE //////////////////////

  sensors_event_t event; 
  gyro.getEvent(&event);

  sensors_event_t event2; 
  accel.getEvent(&event2);

  ///////////////////////////////////////////////////////// 
  ///////////////////////////////////////////////////////// 
  ///////////////////////////////////////////////////////// 
  
  ////Serial.println("right before loop ");
  //read accelerometer stuff
  readAccel(&event2); //sets x- y- and z- raw;
  //else tempo += temp;
  //Serial.println("Scaling");

//not sure about this stuff. getting rid of it because it's giving us all 0's
//using raw values instead to see whats good
  xScaled = map(xRaw, accelMin, accelMax, -1000, 1000);
  yScaled = map(yRaw, accelMin, accelMax, -1000, 1000);
  zScaled = map(zRaw, accelMin, accelMax, -1000, 1000);
  
  xAccel = xScaled / 1000.0;  
  yAccel = yScaled / 1000.0;
  zAccel = zScaled / 1000.0;

  xAccel = xAccel/9.8;
  yAccel = yAccel/9.8;
  zAccel = zAccel/9.8;

  
  //JUAN LOOK: they're in rad/s, so i converted them in the readGyro function
  //that I wrote to match readAccel()
  if(tempo >= dt) readGyro(&event); // sets xGyro, yGyro and zGyro
  else tempo += temp;
  //Serial.println("probing Gyro");

//  kp  = channelVal5;
//  kp2 = channelVal6;
 
  //Serial.println(throttleIn);
  double minChange=0;
  //Serial.println("set W");

  //first need to get the pitch and the roll angles
  //NOTE:QUADCOPTER FRONT IS FACING +X DIRECTION

  tiltangle = 0;
  pitchAngle = findPitch(xRaw, yRaw, zRaw);
  rollAngle =  findRoll(xRaw, zRaw);
  //these values are from the acceleremoter

  dt = 10;//
  pitchDeg += xGyro * .01;  //angle using the gyroscope, not sure if this should be zGyro or another axis
  
  yawDeg += zGyro * .01;

  //using a complementary filter
  finalPitchAngle = .98*(tiltangle + pitchDeg) +.02*pitchAngle;
  // now to "hover" we want our angle to be zero
  pError = hoverAngle - finalPitchAngle;
  pErrSum += pError;
  dPErr = pError - lastPErr;
  pitchOut = kp * pError;
  lastPErr = pError;

  rollDeg += yGyro * .01; //not sure if its the correct axis

  //COMPLEMENTARY FILTER NEEDS TO BE REPLACED WITH KALMAN FILTER!!!!!
  //using a complementary filter
  finalRollAngle = .98*(tiltangle + rollDeg) +.02*rollAngle;
  // now to "hover" we want our angle to be zero
  rError = hoverAngle - finalRollAngle;
  rErrSum += rError;
  dRErr = rError - lastRErr;
  rollOut = kp2 * rError;
  lastRErr = rError;

//  Serial.print("finalRollAngle: ");
//  Serial.print(finalRollAngle);
//  Serial.print("; rError: ");
//  Serial.print(rError);

  lastTime = now;
  //Serial.println("-------------------------------------------");
  //Serial.print("");
  //if (skip % 100 == 0)
  
  Serial.print(rollAngle);
  Serial.print(";");
  Serial.print(xRaw);
  Serial.print(";");
  Serial.println(zRaw);
   //delay(1000);

   late = millis();
   temp = late - now;
}




double findPitch(double xAccel, double yAccel, double zAccel)
{
  return atan2(xAccel,sqrt((yAccel*yAccel)+(zAccel*zAccel))) * RAD_TO_DEG;
}



double findRoll(double xAccel, double zAccel)
{
  return atan2(-xAccel, zAccel) * RAD_TO_DEG;
}



//this read axis just gets 10 readings from the accelerometer then takes the average
//to get something more accurare

void readAccel(sensors_event_t* event)
{
  long xReading = 0;
  long yReading = 0;
  long zReading = 0;
  //delay(1);

  //for (int i = 0; i < sampleSize; i++)
  //{
    //sensors_event_t event;
    //accel.getEvent(&event);
  xRaw = event->acceleration.x;
  yRaw = event->acceleration.y;
  zRaw = event->acceleration.z; //values in 10's of Gs

  //convert to G's
  xRaw = xRaw / 10;
  yRaw = yRaw / 10;
  zRaw = zRaw / 10;

  
//  Serial.println("accel");
//  Serial.println(xRaw);
//  Serial.println(yRaw);
//  Serial.println(zRaw);
}

void readGyro(sensors_event_t* event)
{
  long xReading = 0;
  long yReading = 0;
  long zReading = 0;
  //delay(1);

  //for (int i = 0; i < sampleSize; i++)
  //{
    //sensors_event_t event;
    //accel.getEvent(&event);
    xReading = event->gyro.x;
    yReading = event->gyro.y;
    zReading = event->gyro.z;
    
  //}

  xGyro = xReading * RAD_TO_DEG;
  yGyro = yReading * RAD_TO_DEG;
  zGyro = zReading * RAD_TO_DEG;
  //Serial.println("gyro");
//Serial.println(xGyro);
//Serial.println(yGyro);
//Serial.println(zGyro);
}
