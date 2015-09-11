#include <Wire.h>

/*
Most of this stuff is just combining the accelerometer and gyroscope stuff that
we found online, i'll comment where i added stuff
ALSO i added a bunch of comments that probably aren't necessary
*/
#include <math.h> //included a math library for trig stuff
//first part comes straight from the gyroTest we dowloaded
#include <Adafruit_L3GD20.h>
Adafruit_L3GD20 gyro;
/*
we don't need xGyro yGyro and zGyro like in accelerometer because we're using the Adafruit_L3GD20 library thing
I dont know how i feel about not really knowing whats happening behind the scenes of that sensor
*/

//now the accelerometer part
const int xAccInput = A0;
const int yAccInput = A1;
const int zAccInput = A2;

//WHICH PINS ARE BEING USED UP by the gyroscope?????
//IN the adafruit website it says just 4 and 5 which seems to good to be true

// Raw Ranges:
/*
need to callibrate acccelerometer first using accelCalibration.ino then write down
these values so that we dont have to callibrate it every time
THESE values are temporary unti we get the actual values
*/
int xRawMin = 404;
int xRawMax = 610;

int yRawMin = 401;
int yRawMax = 609;

int zRawMin = 418;
int zRawMax = 630;
int dt = 10;//

// Take multiple samples to reduce noise
const int sampleSize = 10;
//we need to introduce a sample rate as well
int sampleRate= 1000; //one second

int Pulse=1100;
//Ok now next up is the esc stuff
int STATE=1;
int Arming_Time=0;

int pin=3, pin2=5, pin3=6, pin4=9, pin5=10, pin6=11; // these are all the pins that can use pulseIn()
//maybe change this to channel1=3,channel2=5,channel3=6,channel4=9,channel5=10,channl6=11
//also in 'hover mode' we're only using one channel thats going to be throttle
int motor1=2, motor2=4, motor3=12, motor4=3;         //pins we're going to use for output
//note:we're not using arduino's analogwrite() which is their built in pwm

//I think we need some Receiver stuff here so i'll add what i think it is
const int channel1Min= 0;
const int channel1Max=1000; //not real values we need to check what these are


const float pi=3.14159;

unsigned long lastTime;

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
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  {
    Serial.println("You fucked up the wiring for the gyro...bitch");
    while (1);
  }
  //arming the motors
  //LOOK HERE:this seems like a callibration type thing, do we actually need this?
  for (Arming_Time = 0; Arming_Time < 500; Arming_Time += 1)
  {
    digitalWrite(pin,HIGH);
    digitalWrite(pin2,HIGH);
    digitalWrite(pin3,HIGH);
    digitalWrite(pin4,HIGH);
    delayMicroseconds(1100);
    digitalWrite(pin,LOW);
    digitalWrite(pin2,LOW);
    digitalWrite(pin3,LOW);
    digitalWrite(pin4,LOW);
    delay(20-(Pulse/1000));

  }

}

void loop() {
  //read accelerometer stuff
  int xRaw = ReadAxis(xAccInput);
  int yRaw = ReadAxis(yAccInput);
  int zRaw = ReadAxis(zAccInput);

  // Convert raw values to 'milli-Gs"
  long xScaled = map(xRaw, xRawMin, xRawMax, -1000, 1000);
  //might have to change this -1000 to 1000 cause i don't think its actually going to give us G's
  //actually i don't think it'll matter, but it's late at this point
  long yScaled = map(yRaw, yRawMin, yRawMax, -1000, 1000);
  long zScaled = map(zRaw, zRawMin, zRawMax, -1000, 1000);
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
  gyro.read();
  int xGyro = gyro.data.x;
  int yGyro  = gyro.data.y;
  int zGyro = gyro.data.z;

  /*
  at this point im not sure if there are delays written into the gyro.read()
  and im not sure how much delays are going to affect our errors
  everything for one 'reading of the sensors and update outout to esc' should happening
  within 10ms
  */

  int sensorVal1, sensorVal2, sensorVal3,sensorVal4;  //why were these started in loop()
  //int sensorConvert1, sensorConvert2, sensorConvert3, sensorConvert4; dont think i need these anymore
  sensorVal1= pulseIn(pin3,HIGH);
  //sensorVal2= pulseIn(pin2, HIGH);
  //sensorVal3= pulseIn(pin3, HIGH);
  //sensorVal4= pulseIn(pin4, HIGH);

  //for this hover function im going to use sensorval1 as a throttle
  //1200 and 2000 come from the range we had for the esc before
  sensorVal1= map(sensorVal1,channel1Min,channel1Max,1200,2000);
  sensorVal1= constrain(sensorVal1,1200,2000);

  /*
  at this point we've read every input that we would need
  it doesnt seem so bad right now, but im a bit worried about the delays that
  we're introducing when we actually do outputs to the esc's
  There are several things i have questions about in my head

  for now im just going to work with two motors and try to balance them

  as a first attemp what im going to do is  say that both of the motors should throttle
  to whatever its receiving and then that one of them should go faster or slower
  deping on the orientation

  later on what we should do is either have an altitude sensor so we can set
  which motor goes faster or slower depends on which action wilk keep the quadcopter
  at the same height
  first thing to do is to use the accelerometer and gyroscope to get an orientation
  the second thing to do is use that orientation and change the ouput of one of the motors
  which is the PID algorith that im first just going to write as a P algorithm
  but i'll do that tomorrow
  */


  //setting up all the stuff that i need to figure out at some point
  //first need to get angle from accelerometer
  int tiltangle =0;
  int pitchAngle = atan2(xAccel,sqrt((yAccel*yAccel)+(zAccel*zAccel))) * RAD_TO_DEG;

  //you can also get angle by usin the gyroscope
  //only focusing on one axis at this point

  dt = 10;//
  double deg=zGyro * dt;  //this doesnt actually give anything at this point

  //using a complementary filter
  double angle = .98*(tiltangle+deg) +.02*pitchAngle;
  // now to "hover" we want our angle to be zero
  double offset =0-angle;

  //these are all calculations that have to be done but im not quite sure how to
  //arrange all of them exactly

}


//this read axis just gets 10 readings from the accelerometer then takes the average
//to get something more accurare
int ReadAxis(int axisPin)
{
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += analogRead(axisPin);
  }
  return reading/sampleSize; //returns an average
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

