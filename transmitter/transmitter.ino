int pinOut1=2;
int pinOut2=4;
int pin=6;
int pinOut3=12;
int pinOut4=13;
int Pulse=1100;

const int channel3Min = 990;
const int channel3Max = 1984;

double channelInput;

void setup() {
  analogReference(EXTERNAL);
  Serial.begin(9600);

  Serial.println("starting loop");
  pinMode(pin, INPUT);
  pinMode(pinOut1, OUTPUT);
  pinMode(pinOut2,OUTPUT);
  pinMode(pinOut3,OUTPUT);
  pinMode(pinOut4, OUTPUT);

  for(int Arming_Time = 0; Arming_Time < 500; Arming_Time += 1)
  {
    digitalWrite(pinOut1,HIGH);
    digitalWrite(pinOut2,HIGH);
    digitalWrite(pinOut3,HIGH);
    digitalWrite(pinOut4,HIGH);
    delayMicroseconds(1100);
    digitalWrite(pinOut1,LOW);
    digitalWrite(pinOut2,LOW);
    digitalWrite(pinOut3,LOW);
    digitalWrite(pinOut4,LOW);
    delay(20-(Pulse/1000));

  }

}

void loop() {
  Serial.print("poopie");
  channelInput=pulseIn(pin, HIGH);
  //Serial.println(channelInput);
  Serial.println("DEEZ NUTS");
  channelInput= map(channelInput,channel3Min,channel3Max,1100,2000);
  channelInput= constrain(channelInput,1100,2000);

  Serial.println(channelInput);

  digitalWrite(pinOut1,HIGH);
  digitalWrite(pinOut2,HIGH);
  digitalWrite(pinOut3,HIGH);
  digitalWrite(pinOut4,HIGH);
  delayMicroseconds(channelInput);
  digitalWrite(pinOut1,LOW);
  digitalWrite(pinOut2,LOW);
  digitalWrite(pinOut3,LOW);
  digitalWrite(pinOut4,LOW);
}
