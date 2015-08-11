int STATE=1;
int Arming_Time=0; 
int pin=9;
int pin2=3, pin3=5, pin4=6;
int sensorPin1=0, sensorPin2=1, sensorPin3=3, sensorPin4=4;


//Enter value 1100us to Arm the ESC
//enter value >=1200us to run Motor but max 2000us
int Pulse=1100;
//int sensorVal1, sensorVal2, sensorVal3,sensorVal4;
//int sensorConvert1, sensorConvert2, sensorConvert3, sensorConvert4;
//Arm the Servo within void setup()
void setup()
{
  Serial.begin(9600);
  //int sensorVal1, sensorVal2, sensorVal3,sensorVal4;
  
  //sensorVal1= analogRead(sensorPin1);
  //sensorVal2= analogRead(sensorPin2);
  //sensorVal3= analogRead(sensorPin3);
 // sensorVal4= analogRead(sensorPin4);
   //initiate the digital pinas an output
   pinMode(pin, OUTPUT);
   pinMode(pin2, OUTPUT);
   pinMode(pin3, OUTPUT);
   pinMode(pin4, OUTPUT);
   //Will create a 1100us pulse to arm the ESC
   //The pulse will last for 20ms * 500 counts = 10,00ms or 10 seconds
   for (Arming_Time = 0; Arming_Time < 500; Arming_Time += 1) 
   {
      digitalWrite(pin,HIGH);
      digitalWrite(pin2,HIGH);
      digitalWrite(pin3,HIGH);
      digitalWrite(pin4,HIGH);
      delayMicroseconds(1100);
      digitalWrite(pin,LOW);
      digitalWrite(pin2,LOW);
      digitalWrite(pin3,LOW);
      digitalWrite(pin4,LOW);
      delay(20-(Pulse/1000)); 
   }
}

//the loop routine runs over and over again forever
void loop()
{
  int sensorVal1, sensorVal2, sensorVal3,sensorVal4;
  int sensorConvert1, sensorConvert2, sensorConvert3, sensorConvert4;
  sensorVal1= analogRead(sensorPin1);
  sensorVal2= analogRead(sensorPin2);
  sensorVal3= analogRead(sensorPin3);
  sensorVal4= analogRead(sensorPin4);
  Serial.println(sensorVal1);
  Serial.println(sensorVal2);
  Serial.println(sensorVal3);
  Serial.println(sensorVal4);
  //Serial.println('bitch');
  //if(sensorVal1 > 2000)
  
  sensorConvert1= sensorVal1  + 1200;
  sensorConvert2= sensorVal2  + 1200;
  sensorConvert3= sensorVal3  + 1200;
  sensorConvert4= sensorVal4  + 1200;
  
  if(sensorConvert1 > 2000)
    {
     sensorConvert1 = 2000; 
    }
  if(sensorConvert2 > 2000)
    {
     sensorConvert2 = 2000; 
    } 
  if(sensorConvert3 > 2000)
    {
     sensorConvert3 = 2000; 
    }
  if(sensorConvert4 > 2000)
    {
     sensorConvert4 = 2000; 
    }
    
  //Increasing speed
  //for (Pulse= 1150; Pulse <= 1400; Pulse +=1)
  //for (int i = 1150; i <= 1400; i +=1)
  
      
      //digitalWrite(pin,HIGH);
      digitalWrite(pin,HIGH);
      delayMicroseconds(sensorConvert1);
      digitalWrite(pin,LOW);
      
     
      digitalWrite(pin2,HIGH);
      delayMicroseconds(sensorConvert1);
      digitalWrite(pin2,LOW);
      
      digitalWrite(pin3,HIGH);
      delayMicroseconds(sensorConvert1);
      digitalWrite(pin3,LOW);
      
      digitalWrite(pin4,HIGH);
      delayMicroseconds(sensorConvert1);
      digitalWrite(pin4,LOW);
      
      //delayMicroseconds(Pulse);
      //delayMicroseconds(sensorVal1);
     // delayMicroseconds(sensorVal2);
      //delayMicroseconds(sensorVal3);
     // delayMicroseconds(sensorVal4);
      
     // digitalWrite(pin2,LOW);
      //digitalWrite(pin3,LOW);
     // digitalWrite(pin4,LOW);
      
      //digitalWrite(pin,LOW);
      //delay(20-(Pulse/1000)); 
  
  //Decreasing speed
  //for (Pulse=1400; Pulse >= 1150; Pulse -=1)
  //for (int i = 1150; i <= 1400; i +=1)
  //{
     // Pulse = 1160;
    //  digitalWrite(pin,HIGH);
    //  delayMicroseconds(Pulse);
    //  digitalWrite(pin,LOW);
    //  delay(20-(Pulse/1000));  
  //}
}
