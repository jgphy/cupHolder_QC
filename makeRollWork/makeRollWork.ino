#include <Adafruit_L3GD20_U.h>
#include <Adafruit_L3GD20.h>
#include <Adafruit_LSM303_U.h>
#include <math.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

////////////////////////////////////////////////////////////////
////////////////// DECLARING SENSOR STUFF ///////////////////////
////////////////////////////////////////////////////////////////

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321); //Declaring accelerometer
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);                 //Declaring Gyro

//our code uses the Adafruit unified 10DOF sensor and libraries

double xGyro = 0;
double yGyro = 0;
double zGyro = 0;

double xAccel = 0;
double yAccel = 0;
double zAccel = 0;

////////////////////////////////////////////////////////////////
////////////////////ANGLE PROCESSING STUFF///////////////////////
////////////////////////////////////////////////////////////////
//
//most of this is not even being used at the moment.
//remember to check if we can get rid of some of these 
//when we're done
//

int       dt = 10;          // theoretical 'change in time
const int sampleSize = 10;  // Take multiple samples to reduce noise
int       sampleRate= 1000; // Samble accel 1/s; NOT BEING USED

const double hoverAngle = 0;

double lastPErr = 0;
double pErrSum = 0;
double pitchOut = 0;
double kp = 1;

double lastRErr = 0;
double rErrSum = 0;
double rollOut = 0;
double kp2 = 1;

double tiltangle = 0;
double pitchAngle; 
double rollAngle;

double pitchDeg;
double finalPitchAngle;
double pError;
double dPErr;

double rollDeg;
double finalRollAngle;
double rError;
double dRErr;
double change;


////////////////////////////////////////////////////////////////
///////////////// MOTOR INTERFACE (OUTPUT)  ////////////////////
////////////////////////////////////////////////////////////////

int Pulse = 1100;     //Speed at which motors are armed
int STATE=1;          // dont know, not being used


////////////////////////////////////////////////////////////////
////////////////////////  PIN SETUP  ///////////////////////////
////////////////////////////////////////////////////////////////

//input pins

int pin  = 3;
int pin2 = 5; 
int pin3 = 6; // in use
int pin4 = 9;
int pin5 = 10; // in use FUTURE PID
int pin6 = 11; // in use FUTURE PID

//OUTPUT pins

int motor1 = 2; 
int motor2 = 4;
int motor3 = 12;
int motor4 = 13;

////////////////////////////////////////////////////////////////
////////////////////  TRANSMITTER INTERFACE  ///////////////////
////////////////////////////////////////////////////////////////


double oldChannelVal3 = 1100; // old as in "what it was in the last loop"
                              // not as in deprecated
                              // used to find the change in throttle

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

const int channel6Min = 990;
const int channel6Max = 1977;

unsigned long lastTime = 0;
unsigned long now;

double w_1 = 1100;
double w_2 = 1100;
double w_3 = 1100;
double w_4 = 1100;
double w   = 1100;


double channelVal1;
double channelVal2;
double channelVal3;
double channelVal4;
double channelVal5;
double channelVal6;

double throttleIn = 1100;

const double minChange = 0; //min change in throttle

////////////////////////////////////////////////////////////////
//////////////////////////  SETUP()  ///////////////////////////
////////////////////////////////////////////////////////////////

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

//initialize gyro
  gyro.enableAutoRange(true);
  if (!gyro.begin())
  {
    Serial.println("Oops ... unable to initialize the GYRO. Check your wiring!");
    while (1);
  }

// Initialise accel
  if(!accel.begin())
  {
    Serial.println("Ooops, no ACCELEROMETER detected ... Check your wiring!");
    while(1);
  }

//We initialize sensor object here as per API specification.
  sensor_t sensor;
  accel.getSensor(&sensor);

//arming the motors
//We feed the motor 1100 microseconds of HIGH
//NOTE: we dont use built in PWM functions because
//their range is too constricted.
  for (int i = 500; i > 0; i--)
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

////////////////////////////////////////////////////////////////
///////////////////////////  LOOP()  ///////////////////////////
////////////////////////////////////////////////////////////////

void loop() {
  //////////////////// SENSOR EVENT SETUP /////////////////
  ////////////////////// V IMPORTANT //////////////////////
  ////////////////////// DONT DELETE //////////////////////

//Initialize sensor events, so that we can read them later.
  sensors_event_t event; 
  gyro.getEvent(&event);

  sensors_event_t event2; 
  accel.getEvent(&event2);

  ///////////////////////////////////////////////////////// 
  ///////////////////////////////////////////////////////// 
  ///////////////////////////////////////////////////////// 

  readAccel(&event2); //sets x- y- and z- raw;
  readGyro(&event);   // sets xGyro, yGyro and zGyro

  channelVal3 = pulseIn(pin3, HIGH); //read throttle
  //channelVal5= pulseIn(pin5, HIGH);  //kp value for pitch angle PID
  channelVal6= pulseIn(pin, HIGH);  //kp value for roll angle  PID

  channelVal3= map(channelVal3,channel3Min,channel3Max,1100,2000);
  channelVal3= constrain(channelVal3,1100,2000);

//  channelVal5 = map(channelVal5, channel5Min, channel5Max, 0, 2);   //0 to 2 are just numbers that i picked, probably need to manually check these
//  channelVal5 = constrain(channelVal5, 0, 2);
  channelVal6 = map(channelVal6, channel6Min, channel6Max, 0.0, 2000.0);
  channelVal6 = constrain(channelVal6, 0.0, 2000.0);
  channelVal6=channelVal6/1000.0;
//  kp  = channelVal5;
  kp2 = channelVal6;
  Serial.println("kp2");
  Serial.println(kp2);

  throttleIn = channelVal3;
  Serial.println("throttleIn");
  Serial.println(throttleIn);

//first thing to do is to use the accelerometer and gyroscope to get an orientation
//the second thing to do is use that orientation and change the ouput of one of the motors
//which is the PID algorith that im first just going to write as a P algorithm
//but i'll do that tomorrow

//first need to get the pitch and the roll angles
//NOTE:QUADCOPTER FRONT IS FACING +X DIRECTION LIKE THE LITTLE PICTURE ON THE SENSOR

  tiltangle = 0;
  pitchAngle = findPitch(xAccel, yAccel, zAccel);
  rollAngle =  findRoll(xAccel, zAccel);

  pitchDeg = yGyro * dt;
  finalPitchAngle = .98*(tiltangle + pitchDeg) +.02*pitchAngle;
  //To "hover" we want our angle to be zero
  pError = hoverAngle - finalPitchAngle;
  pErrSum += pError;
  dPErr = pError - lastPErr;
  pitchOut = kp * pError;
  lastPErr = pError;

  rollDeg = xGyro * dt;
  //COMPLEMENTARY FILTER NEEDS TO BE REPLACED WITH KALMAN FILTER!!!!!
  //using a complementary filter
  finalRollAngle = rollAngle;
  // now to "hover" we want our angle to be zero
//  Serial.println("finalRollAngle");
//  Serial.println(finalRollAngle);
  rError = hoverAngle - finalRollAngle;
  rErrSum += rError;
  dRErr = rError - lastRErr;
  rollOut = kp2 * rError;
//  Serial.println("rollout");
//  Serial.println(rollOut);
  lastRErr = rError;

  lastTime = now;

  if(abs(throttleIn - oldChannelVal3) > minChange)
    {
      change = throttleIn - oldChannelVal3;
//      Serial.print("change: ");
//      Serial.println(change);
      if((w_2 + (change)) > 2000)
      {
        w_1 = 2000;
        w_2 = 2000;
        w_3 = 2000;
        w_4 = 2000;
      }
      else if((w_2 + (change)) < 1100)
      {
        //Serial.println("in the wrong loop");
        w_1 = 1100;
        w_2 = 1100;
        w_3 = 1100;
        w_4 = 1100;
      }
      else
      {
        w_1 += change;
        w_2 += change;
        w_3 += change;
        w_4 += change;
  
  
      }
    }
  
  if(throttleIn == 1100)
  {
    w_1 = 1100;
    w_2 = 1100;
    w_3 = 1100;
    w_4 = 1100;
  }
  
//  w_1 += w_1 * rollOut / 4;
  w_4 -= 15 * rollOut;

  w_2 += 15 * rollOut;
//  w_3 += -w_3 * rollOut /4;
//
//  w_1 += w_1 * pitchOut / 4;
//  w_3 += w_3 * pitchOut / 4;
//
//  w_2 += -w_2 * pitchOut /4;
//  w_4 += -w_4 * pitchOut /4;

  if (w_2 >=2000){
    w_2=2000;
  }

    if (w_4 >=2000){
    w_4=2000;
  }

  if (w_2 <=1100){
    w_2=1100;
  }

    if (w_4 <=1100){
    w_4=1100;
  }

//  Serial.println(w_1);
//  Serial.println(w_2);
//  Serial.println(w_3);
//  Serial.println(w_4);
  
  oldChannelVal3 = throttleIn;

  w_1 = 1100;
  w_3 = 1100;  
  
  writeAll(motor1, w_1, motor2, w_2, motor3, w_3, motor4, w_4);

}

void writeAll(int motor1, double w_1, int motor2, double w_2, int motor3, double w_3, int motor4, double w_4)
{
// Serial.println("set values");
// Serial.println(w_1);
 digitalWrite(motor1,HIGH);
 delayMicroseconds(w_1);
 digitalWrite(motor1,LOW);

 Serial.println(w_2);
 digitalWrite(motor2,HIGH);
 delayMicroseconds(w_2);
 digitalWrite(motor2,LOW);

// Serial.println(w_3);
 digitalWrite(motor3,HIGH);
 delayMicroseconds(w_3);
 digitalWrite(motor3,LOW);

// Serial.println(w_4);
 digitalWrite(motor4,HIGH);
 delayMicroseconds(w_4);
 digitalWrite(motor4,LOW);
}

double findPitch(double xAccel, double yAccel, double zAccel)
{
  return atan2(xAccel,sqrt((yAccel*yAccel)+(zAccel*zAccel))) * RAD_TO_DEG;
}

double findRoll(double xAccel, double zAccel)
{
  return atan2(-xAccel, zAccel) * RAD_TO_DEG;
}

void readAccel(sensors_event_t* event)
{
  long xReading = 0;
  long yReading = 0;
  long zReading = 0;
  //delay(1);

  xAccel += event->acceleration.x;
  yAccel += event->acceleration.y;
  zAccel += event->acceleration.z;
  //units are in 10's of Gs
  //so we convert to G's
  xAccel = xAccel / 10;
  yAccel = yAccel / 10;
  zAccel = zAccel / 10;
}

void readGyro(sensors_event_t* event)
{
  long xReading = 0;
  long yReading = 0;
  long zReading = 0;
  //delay(1);

  for (int i = 0; i < sampleSize; i++)
  {
    //sensors_event_t event;
    //accel.getEvent(&event);
    xReading += event->gyro.x;
    yReading += event->gyro.y;
    zReading += event->gyro.z;
    
  }

  xGyro = xReading * RAD_TO_DEG;
  yGyro = yReading * RAD_TO_DEG;
  zGyro = zReading * RAD_TO_DEG;
  //Serial.println("gyro");
//Serial.println(xGyro);
//Serial.println(yGyro);
//Serial.println(zGyro);
}

/////////////////////////////////////////////////////
/////////////      KALMAN FILTER      ///////////////
/////////////////////////////////////////////////////

// float error = 0; //initial value

// float gyroBias = 0.003;
// float accelVar = 0.001;
// //these are suggested,
// //supposedly well balanced values for these variables

// float P [2][2] = {{1000., 0.}, {0., 1000.}};
// //# initial uncertainty matrix

// float R = 0.03;
// //# measurement uncertainty -- some suggested constant correlated with how accurate our selnsors are. may be non-optimal

// float K [2] = {0, 0};
// //kalman gain gets updated over time to sshift weight to accelerometer, and away from the gyroscope

// float S; //some intermediate value in the algorithm

// float rate = 0;
//angular velocity according to gyro

/////////////////////////////////////////
/*
please feed:

accelIn -> ANGLE according to accelerometer

gyroIn -> Angular VELOCITY according to gyroscope

angle -> whatever the latest value we have for the angle in this pair of axes is
*/
/////////////////////////////////////////

// void kalman(float &accelIn, float &gyroIn, float &angle)
// {
//     // PREDICTION STEP
//     rate = gyroIn - gyroBias;  //offset accounting for bias
//     angle += dt * rate;
//     //^^^ predicts angle based on millis().
//     //might have to call millis() again to see if it gets more accurate.
//     //leaving as- is for now, or just forgo this altogether

//     P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + accelVar);
//     P[0][1] -= dt * P[1][1];
//     P[1][0] -= dt * P[1][1];
//     P[1][1] += gyroBias * dt;
//     //^^ predicts uncertainty matrix to account for time passed


//     // MEASUREMENT STEP
//     error = accelIn - angle;
//     // difference between latest measured value and current angle value

//     S = P[0][0] + R;
//     //some intermediate step needed for kalman gain...

//     K[0] = P[0][0] / S;
//     K[1] = P[1][0] / S;
//     // updates kalman gain.

//     //UPDATE STEP
//     angle += K[0] * error;
//     gyroBias += K[1] * error;
//     //updates abgle and gyroBias based on gain and error

//     float P00_temp = P[0][0];
//     float P01_temp = P[0][1];

//     P[0][0] -= K[0] * P00_temp;
//     P[0][1] -= K[0] * P01_temp;
//     P[1][0] -= K[1] * P00_temp;
//     P[1][1] -= K[1] * P01_temp;
//     // Updates uncertainty matrix
//     //based on gain and predicted uncertainty
//}
