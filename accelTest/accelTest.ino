#include <math.h>

const int xInput = A0;
const int yInput = A1;
const int zInput = A2;
//const int buttonPin = 2;

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

void setup() {
  analogReference(EXTERNAL);
  Serial.begin(9600);
}

void loop() {
  int xRaw = ReadAxis(xInput);
  int yRaw = ReadAxis(yInput);
  int zRaw = ReadAxis(zInput);

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

    // Convert raw values to 'milli-Gs"
    long xScaled = map(xRaw, xRawMin, xRawMax, -1000, 1000);
    long yScaled = map(yRaw, yRawMin, yRawMax, -1000, 1000);
    long zScaled = map(zRaw, zRawMin, zRawMax, -1000, 1000);

    // re-scale to fractional Gs
    float xAccel = xScaled / 1000.0;
    float yAccel = yScaled / 1000.0;
    float zAccel = zScaled / 1000.0;

    float pitchAngle = atan(xAccel / sqrt((yAccel * yAccel) + (zAccel * zAccel)));

    float finalAngle = pitchAngle * 180 / pi;
    Serial.print(" angle: ");
    Serial.println(finalAngle);
    
    Serial.print(" :: ");
    Serial.print(xAccel);
    Serial.print("G, ");
    Serial.print(yAccel);
    Serial.print("G, ");
    Serial.print(zAccel);
    Serial.println("G");

  delay(200);

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
  return reading/sampleSize;
}

