/*
Most of this stuff is just combining the accelerometer and gyroscope stuff that
we found online, i'll comment where i added stuff
ALSO i added a bunch of comments that probably aren't necessary
*/

//first part comes straight from the gyroTest we dowloaded
#include <Adafruit_L3GD20.h>
Adafruit_L3GD20 gyro;
/*
we don't need xAccel yAccel and zAccel because we're using the Adafruit_L3GD20 library thing
I dont know how i feel about not really knowing whats happening behind the scenes of that sensor
*/

//now the accelerometer part
//they said const so i wrote constant
const int xAccInput = A0;
const int yAccInput = A1;
const int zAccInput = A2;

//WHICH PINS ARE BEING USED UP by the gyroscope?????

// Raw Ranges:
/*
need to callibrate acccelerometer first using accelCalibration.ino then write down
these values so that we dont have to callibrate it every time
THESE values are temporary unti we get the actual values
*/
int xRawMin = 512;
int xRawMax = 512;
                    //Remember to change these!
int yRawMin = 512;
int yRawMax = 512;

int zRawMin = 512;
int zRawMax = 512;

// Take multiple samples to reduce noise
const int sampleSize = 10;


void setup() {
  analogReference(EXTERNAL);
  Serial.begin(9600);

  //makes sure that the gyroscope is plugged in
  //We should think about adding a screen somewhere on the quadcopter to diagnose errors like this
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  {
    Serial.println("You fucked up the wiring for the gyro...bitch");
    while (1);
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
  // these values are scaled so their in 'g' units to get m/s^2 we'd have to divide
  //by 9.8...i think, i need a little bit more tome


  //now the gyroscope stuff
  //these should be in deg/s if not we have to convert them
  //one of us should look
  gyro.read();
  int roll = gyro.data.x;
  int pitch  = gyro.data.y;
  int yaw = gyro.data.z;

  /*
  at this point im not sure if there are delays written into the gyro.read()
  and im not sure how much delays are going to affect our errors
  everything for one 'reading of the sensors and update outout to esc' should happening
  within 10ms
  */

}

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
