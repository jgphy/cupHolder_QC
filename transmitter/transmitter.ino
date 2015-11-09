int pin=3;

double channelInput;

void setup() {
  analogReference(EXTERNAL);
  Serial.begin(9600);
  pinMode(pin, INPUT);
}

void loop() {
  channelInput=pulseIn(pin, HIGH);
  Serial.println(channelInput);
  
}
