#include <Wire.h>
#include <Adafruit_L3GD20.h>
#include <math.h>

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//                                                  //
//                      GYROSCOPE                   //
//                                                  //
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

/*
this is all using some library that we might want to look into so we know
whats really happening...or not
*/

int dt = 10;
float deg = 0;
float y;
float offset = 2.35;

// No need to specify pins for I2C
Adafruit_L3GD20 gyro;

float tiltangle = 0;

float angle = 0;

const int xInput = A0;
const int yInput = A1;
const int zInput = A2;
//const int buttonPin = 2;

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//                                                  //
//                 ACCELEROMETER                    //
//                                                  //
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

// Raw Ranges:
// initialize to mid-range and allow calibration to
// find the minimum and maximum for each axis
int xRawMin = 404;
int xRawMax = 610;

int yRawMin = 401;
int yRawMax = 609;

int zRawMin = 418;
int zRawMax = 630;

// Take multiple samples to reduce noised
const int sampleSize = 10;  //ReadAxis() uses this
float pi = 3.14159;


void setup()
{
  

  analogReference(EXTERNAL);
  Serial.begin(9600);
  
  //Serial.println("test");

  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }
}

void loop() 
{
   int xRaw = ReadAxis(xInput);
  int yRaw = ReadAxis(yInput);
  int zRaw = ReadAxis(zInput);
    /*
    Serial.print("Raw Ranges: X: ");
    Serial.print(xRawMin);
    Serial.print("-");
    Serial.print(xRawMax);

    Serial.print(", Y: ");
    Serial.print(yRawMin);
    Serial.print("-");
    Serial.print(yRawMax);

    Serial.print(", Z: ");
    Serial.print(zRawMin);
    Serial.print("-");
    Serial.print(zRawMax);
    Serial.println();
    Serial.print(xRaw);
    Serial.print(", ");
    Serial.print(yRaw);
    Serial.print(", ");
    Serial.print(zRaw);
*/
    // Convert raw values to 'milli-Gs"
    long xScaled = map(xRaw, xRawMin, xRawMax, -1000, 1000);
    long yScaled = map(yRaw, yRawMin, yRawMax, -1000, 1000);
    long zScaled = map(zRaw, zRawMin, zRawMax, -1000, 1000);

    // re-scale to fractional Gs
    float xAccel = xScaled / 1000.0;
    float yAccel = yScaled / 1000.0;
    float zAccel = zScaled / 1000.0;

    float pitchAngle = (atan(xAccel / sqrt((yAccel * yAccel) + (zAccel * zAccel)))) * (180 / pi);

    //float finalAngle = pitchAngle * 180 / pi;
    /*
    Serial.print(" angle: ");
    Serial.println(finalAngle);
    
    Serial.print(" :: ");
    Serial.print(xAccel);
    Serial.print("G, ");
    Serial.print(yAccel);
    Serial.print("G, ");
    Serial.print(zAccel);
    Serial.println("G");
*/
  if (millis() >= dt)
  {
    gyro.read();
    y = gyro.data.y - offset;
    //if((y > 2.6) || (y < 0)) 
    deg += y * dt;
    
    //Serial.print("THETA: "); Serial.println(deg/100);
    //Serial.print("Y: "); Serial.println(y);
      //Serial.print("X: "); Serial.print(gyro.data.x);   Serial.print(" ");
      //Serial.print("Y: "); Serial.print(y);   Serial.print(" ");
      //Serial.print("Z: "); Serial.println(gyro.data.z); Serial.print(" ");
  }


  kalman(pitchAngle, y, angle);

  //angle = .98*(tiltangle+(deg/100)) +.02*pitchAngle;
  Serial.print("PITCH: ");
  Serial.println(pitchAngle);
  Serial.print("ANGLE: ");
  Serial.println(angle);
  Serial.print("DEG: ");
  Serial.println(deg/100);
  
  
  delay(100);
  
  

}


//
// Read "sampleSize" samples and report the average
//
int ReadAxis(int axisPin)
{
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += analogRead(axisPin);
  }
  return reading/sampleSize;
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

