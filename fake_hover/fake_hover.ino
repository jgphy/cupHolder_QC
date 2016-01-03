//#include <Adafruit_L3GD20.h>
//#include <Adafruit_L3GD20_U.h>
//#include <Adafruit_LSM303_U.h>
#include <math.h>
//#include <Adafruit_BMP085_U.h>
//#include <Adafruit_10DOF.h>
//#include <Adafruit_Sensor.h>
#include <Wire.h>

//first part comes straight from the gyroTest we dowloaded
//Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
//Adafruit_L3GD20 gyro;
//Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

int dt = 10;//

// Take multiple samples to reduce noise
const int sampleSize = 10;
//we need to introduce a sample rate as well
int sampleRate= 1000; //one second

int Pulse=1100;
//Ok now next up is the esc stuff
int STATE=1;
int Arming_Time=0;

int pin  = 3;
int pin2 = 5;
int pin3 = 6;//in use
int pin4 = 9;
int pin5 = 10;//in use
int pin6 = 11;//in use


// these are all the pins that can use pulseIn()
//channel1=3,channel2=5,channel3=6,channel4=9,channel5=10,channl6=11
//also in 'hover mode' we're only using one channel thats going to be throttle

int motor1 = 2; //pin
int motor2 = 4;
int motor3 = 12;
int motor4 = 13;


//pins we're going to use for output
//note:we're not using arduino's analogwrite() which is their built in pwm

//I think we need some Receiver stuff here so i'll add what i think it is

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

/*When i was reading these values in using "transmitter.ino" these values weren't super consistent,
 i dont know if thats going to be and issueor not
*/


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

double xRaw = 0;
double yRaw = 0;
double zRaw = 0;

double accelMax;
double accelMin;

double w_1 = 0;
double w_2 = 0;
double w_3 = 0;
double w_4 = 0;
double w   = 0;


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

 // Serial.println("Setting up...");

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

  //sensor_t sensor;
  //accel.getSensor(&sensor);
  accelMax = 0;
  accelMin = 0;

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

  Serial.println("Starting Loop");
}



void loop() {
  //Serial.println("right before loop ");
  //read accelerometer stuff
  //readAccel(); //sets x- y- and z- raw;
  now = millis();
  Serial.println("Scaling");
  // Convert raw values to 'milli-Gs"
  long xScaled = map(xRaw, accelMin, accelMax, -1000, 1000);
  //might have to change this -1000 to 1000 cause i don't think its actually going to give us G's
  //actually i don't think it'll matter, but it's late at this point
  long yScaled = map(yRaw, accelMin, accelMax, -1000, 1000);
  long zScaled = map(zRaw, accelMin, accelMax, -1000, 1000);
  /*
  map is a built in function map(value, fromLow, fromHigh, toLow, toHigh)
  you guys can look it up but essentially remaps a value xRaw that is within some
  range [xRawMin,xRawMax and scales it to another range [-1000,1000]
  note: im going to use this later (attempt to) on for the receiver input to the output going
  into the esc's , but i think theres a function constraint() that we should also
  use if i want to do that
  */
  // re-scale to fractional Gs
  float xAccel = xScaled / 1000.0;
  float yAccel = yScaled / 1000.0;
  float zAccel = zScaled / 1000.0;
  /*
  these values are scaled so their in 'g' units to get m/s^2 we'd have to divide
  by 9.8...i think, i need a little bit more tome
  xAccel=xAccel/9.8
  yAccel=yAccel/9.8
  zAccel=zAccel/9.8
  */

  //now the gyroscope stuff
  //these should be in deg/s if not we have to convert them
  //one of us should look
  //gyro.read();
  int xGyro = 0;
  int yGyro  = 0;
  int zGyro = 0;
  Serial.println("probing Gyro");
  /*
  at this point im not sure if there are delays written into the gyro.read()
  and im not sure how much delays are going to affect our errors
  everything for one 'reading of the sensors and update outout to esc' should happening
  within 10ms
  */

  double channelVal1;
  double channelVal2;
  double channelVal3;
  double channelVal4;
  double channelVal5;
  double channelVal6;
  //why were these started in loop()
  //int sensorConvert1, sensorConvert2, sensorConvert3, sensorConvert4; dont think i need these anymore


  channelVal1= pulseIn(pin,HIGH);
  channelVal2= pulseIn(pin2, HIGH);
  channelVal3= pulseIn(pin3, HIGH);   //throttle
  channelVal4= pulseIn(pin4, HIGH);
  channelVal5= pulseIn(pin5, HIGH);  //kp value for pitch angle PID
  channelVal6= pulseIn(pin6, HIGH);  //kp value for roll angle  PID

  //for this hover function im going to use channelVal1 as a throttle
  //1200 and 2000 come from the range we had for the esc before
  channelVal3= map(channelVal3,channel3Min,channel3Max,1100,2000);
  channelVal3= constrain(channelVal3,1100,2000);

  channelVal5 = map(channelVal5, channel5Min, channel5Max, 0, 2);   //0 to 2 are just numbers that i picked, probably need to manually check these
  channelVal5 = constrain(channelVal5, 0, 2);
  channelVal6 = map(channelVal6, channel6Min, channel6Max, 0, 2);
  channelVal6 = constrain(channelVal6, 0, 2);

  kp  = channelVal5;
  kp2 = channelVal6;
  w   = channelVal3;
  Serial.println("set W");
  /*

  first thing to do is to use the accelerometer and gyroscope to get an orientation
  the second thing to do is use that orientation and change the ouput of one of the motors
  which is the PID algorith that im first just going to write as a P algorithm
  but i'll do that tomorrow

  */

  //first need to get the pitch and the roll angles
  //NOTE:QUADCOPTER FRONT IS FACING +X DIRECTION

  double tiltangle = 0;
  double pitchAngle = findPitch(xAccel, yAccel, zAccel);
  double rollAngle =  findRoll(xAccel, zAccel);
  //these values are from the acceleremoter

  dt = 10;//
  double pitchDeg = zGyro * dt; //angle using the gyroscope, not sure if this should be zGyro or another axis
  //using a complementary filter
  double finalPitchAngle = .98*(tiltangle + pitchDeg) +.02*pitchAngle;
  // now to "hover" we want our angle to be zero
  double pError = hoverAngle - finalPitchAngle;
  pErrSum += pError;
  double dPErr = pError - lastPErr;
  pitchOut = kp * pError;
  lastPErr = pError;

  double rollDeg = yGyro * dt; //not sure if its the correct axis

  //COMPLEMENTARY FILTER NEEDS TO BE REPLACED WITH KALMAN FILTER!!!!!
  //using a complementary filter
  double finalRollAngle = .98*(tiltangle + rollDeg) +.02*rollAngle;


  // now to "hover" we want our angle to be zero
  double rError = hoverAngle - finalRollAngle;
  rErrSum += rError;
  double dRErr = rError - lastRErr;
  rollOut = kp2 * rError;
  lastRErr = rError;


  lastTime = now;

  //these are all calculations that have to be done but im not quite sure how to
  //arrange all of them exactly

   w_1 += channelVal3 - w;
   w_2 += channelVal3 - w;
   w_3 += channelVal3 - w;
   w_4 += channelVal3 - w;

   w_1 += w_1 * rollOut / 2;
   w_4 += w_4 * rollOut / 2;

   w_2 += -w_2 * rollOut /2;
   w_3 += -w_3 * rollOut /2;

   w_1 += w_1 * pitchOut / 2;
   w_3 += w_3 * pitchOut / 2;

   w_2 += -w_2 * pitchOut /2;
   w_4 += -w_4 * pitchOut /2;

  if(w_1 < 1200) w_1 = 1200;
  if(w_2 < 1200) w_2 = 1200;
  if(w_3 < 1200) w_3 = 1200;
  if(w_4 < 1200) w_4 = 1200;

  if(w_1 > 2000) w_1 = 2000;
  if(w_2 > 2000) w_2 = 2000;
  if(w_3 > 2000) w_3 = 2000;
  if(w_4 > 2000) w_4 = 2000;


  writeAll(motor1, w_1, motor2, w_2, motor3, w_3, motor4, w_4);

}

void writeAll(int motor1, double w_1, int motor2, double w_2, int motor3, double w_3, int motor4, double w_4)
{
 Serial.println("w: motor1");
 digitalWrite(motor1,HIGH);
 delayMicroseconds(w_1);
 digitalWrite(motor1,LOW);

 Serial.println("w: motor2");
 digitalWrite(motor2,HIGH);
 delayMicroseconds(w_2);
 digitalWrite(motor2,LOW);

 Serial.println("w: motor3");
 digitalWrite(motor3,HIGH);
 delayMicroseconds(w_3);
 digitalWrite(motor3,LOW);

 Serial.println("w: motor4");
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




//this read axis just gets 10 readings from the accelerometer then takes the average
//to get something more accurare
void readAccel()
{
  long xReading = 0;
  long yReading = 0;
  long zReading = 0;
  delay(1);

  for (int i = 0; i < sampleSize; i++)
  {
    //sensors_event_t event;
    //accel.getEvent(&event);
    xReading += 0;//event.acceleration.x;
    yReading += 0;//event.acceleration.y;
    zReading += 0;//event.acceleration.z;
  }

  xRaw = xReading/sampleSize;
  yRaw = yReading/sampleSize;
  zRaw = zReading/sampleSize;
}



/////////////////////////////////////////////////////
/////////////      KALMAN FILTER      ///////////////
/////////////////////////////////////////////////////

float error = 0; //initial value

float gyroBias = 0.003;
float accelVar = 0.001;
//these are suggested,
//supposedly well balanced values for these variables

float P [2][2] = {{1000., 0.}, {0., 1000.}};
//# initial uncertainty matrix

float R = 0.03;
//# measurement uncertainty -- some suggested constant correlated with how accurate our selnsors are. may be non-optimal

float K [2] = {0, 0};
//kalman gain gets updated over time to sshift weight to accelerometer, and away from the gyroscope

float S; //some intermediate value in the algorithm

float rate = 0;
//angular velocity according to gyro

/////////////////////////////////////////
/*
please feed:

accelIn -> ANGLE according to accelerometer

gyroIn -> Angular VELOCITY according to gyroscope

angle -> whatever the latest value we have for the angle in this pair of axes is
*/
/////////////////////////////////////////

void kalman(float &accelIn, float &gyroIn, float &angle)
{
    // PREDICTION STEP
    rate = gyroIn - gyroBias;  //offset accounting for bias
    angle += dt * rate;
    //^^^ predicts angle based on millis().
    //might have to call millis() again to see if it gets more accurate.
    //leaving as- is for now, or just forgo this altogether

    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + accelVar);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += gyroBias * dt;
    //^^ predicts uncertainty matrix to account for time passed


    // MEASUREMENT STEP
    error = accelIn - angle;
    // difference between latest measured value and current angle value

    S = P[0][0] + R;
    //some intermediate step needed for kalman gain...

    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    // updates kalman gain.

    //UPDATE STEP
    angle += K[0] * error;
    gyroBias += K[1] * error;
    //updates abgle and gyroBias based on gain and error

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
    // Updates uncertainty matrix
    //based on gain and predicted uncertainty
}
