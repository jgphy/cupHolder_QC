int pin=3;
int pinOut=6;
int Pulse=1100;

const int channel3Min = 990;
const int channel3Max = 1984;

double channelInput;

void setup() {
  analogReference(EXTERNAL);
  Serial.begin(9600);
  pinMode(pin, INPUT);
  pinMode(pinOut, OUTPUT);

  for(int Arming_Time = 0; Arming_Time < 500; Arming_Time += 1)
  {
    digitalWrite(pinOut,HIGH);
    delayMicroseconds(1100);
    digitalWrite(pinOut,LOW);
    delay(20-(Pulse/1000));

  }
}

void loop() {
  channelInput=pulseIn(pin, HIGH);
  //Serial.println(channelInput);

  channelInput= map(channelInput,channel3Min,channel3Max,1100,2000);
  channelInput= constrain(channelInput,1100,2000);

  Serial.println(channelInput);

  digitalWrite(pinOut,HIGH);
  delayMicroseconds(channelInput);
  digitalWrite(pinOut,LOW);

}
