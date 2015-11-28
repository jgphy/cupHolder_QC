int STATE = 1;
int Arming_Time = 0;
int pin = 2, pin2 = 4, pin3 = 12, pin4 = 13;

int i = 1200;

//Enter value 1100us to Arm the ESC
//enter value >=1200us to run Motor but max 2000us
int Pulse = 1100;
int sensorConvert1;

void setup()
{
    Serial.begin(9600);
 // analogReference(EXTERNAL);
 // Serial.begin(9600);

  pinMode(pin, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);
  //Will create a 1100us pulse to arm the ESC
  //The pulse will last for 20ms * 500 counts = 10,00ms or 10 seconds
  for (Arming_Time = 0; Arming_Time < 500; Arming_Time += 1)
  {
    digitalWrite(pin, HIGH);
    digitalWrite(pin2, HIGH);
    digitalWrite(pin3, HIGH);
    digitalWrite(pin4, HIGH);
    delayMicroseconds(1100);
    digitalWrite(pin, LOW);
    digitalWrite(pin2, LOW);
    digitalWrite(pin3, LOW);
    digitalWrite(pin4, LOW);
    delay(20 - (Pulse / 1000));
  }
}

//the loop routine runs over and over again forever
void loop()
{
  i+=1 ;
  if (i > 2000) {i =1200;}
  Serial.println(i);
  sensorConvert1 = 1600;

  Serial.println("writing");
  digitalWrite(pin, HIGH);
  digitalWrite(pin2, HIGH);
  digitalWrite(pin3, HIGH);
  digitalWrite(pin4, HIGH);
  delayMicroseconds(i);
  digitalWrite(pin, LOW);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, LOW);
  digitalWrite(pin4, LOW);
/*
  digitalWrite(pin2, HIGH);
  delayMicroseconds(sensorConvert1);
  digitalWrite(pin2, LOW);

  digitalWrite(pin3, HIGH);
  delayMicroseconds(sensorConvert1);
  digitalWrite(pin3, LOW);

  digitalWrite(pin4, HIGH);
  delayMicroseconds(sensorConvert1);
  digitalWrite(pin4, LOW);
*/
}
