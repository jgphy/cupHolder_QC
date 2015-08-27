#include <Wire.h>

#include <Adafruit_L3GD20.h>

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

void setup()
{
  Serial.begin(9600);

  // Try to initialise and warn if we couldn't detect the chip
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }
}

void loop()
{
  if (millis() >= dt)
  {
    gyro.read();
    y = gyro.data.y - offset;
    //if((y > 2.6) || (y < 0)) 
    deg += y * dt;
    
    //Serial.print("THETA: "); Serial.println(deg/100);
    //Serial.print("Y: "); Serial.println(y);
    Serial.print(gyro.data.x);
    Serial.print(y);
    Serial.println(gyro.data.z);
  }

  
  
  delay(100);
}
