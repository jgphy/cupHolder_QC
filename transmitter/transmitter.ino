int channelIn1=3;
int channelIn2=5;
int channelIn3=6;
int channelIn4=9;
double channelInput;

void setup() {
  analogReference(EXTERNAL);
  Serial.begin(9600);

  pinMode(channelIn1, INPUT);
  pinMode(channelIn2, INPUT);
  pinMode(channelIn3, INPUT);
  pinMode(channelIn4, INPUT);


}

void loop() {
double channel1=pulseIn(channelIn1,HIGH);
double channel2=pulseIn(channelIn2,HIGH);
double channel3=pulseIn(channelIn3,HIGH);
double channel4=pulseIn(channelIn4,HIGH);
Serial.println("chanel1: ");
Serial.println(channel1);
Serial.println("chanel2: ");
Serial.println(channel2);
Serial.println("chanel3: ");
Serial.println(channel3);
Serial.println("chanel4: ");
Serial.println(channel4);
delay(500);

}
