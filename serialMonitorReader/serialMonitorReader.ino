#import <math.h>
double i=0;
void setup() {
  
  Serial.begin(9600);
}

void loop() {
  Serial.println(sin(i));
  i+=.10;
  delay(50);
}
