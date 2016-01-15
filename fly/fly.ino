#include <Wire.h>

#include <Adafruit_LSM303_U.h>

#include <Adafruit_L3GD20_U.h>
//#include <Adafruit_L3GD20.h>
#include <math.h>


////////////////////////////////////////SENSOR STUFF///////////////////////////////////////////////
//
//double xRaw = 0;
//double yRaw = 0;
//double zRaw = 0;
//Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
//Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
//
//double accelMax;
//double accelMin;
//
//long xScaled = map(xRaw, accelMin, accelMax, -1000, 1000);
//  //might have to change this -1000 to 1000 cause i don't think its actually going to give us G's
//  //actually i don't think it'll matter, but it's late at this point
//long yScaled = map(yRaw, accelMin, accelMax, -1000, 1000);
//long zScaled = map(zRaw, accelMin, accelMax, -1000, 1000);
//float xAccel = xScaled / 1000.0;
//float yAccel = yScaled / 1000.0;
//float zAccel = zScaled / 1000.0;
//
//
//int xGyro = 0;
//int yGyro  = 0;
//int zGyro = 0;

// Take multiple samples to reduce noise
const int sampleSize = 10;
//we need to introduce a sample rate as well
int sampleRate=10; //ms

int dt= 10;

unsigned long lastTime = 0;
unsigned long now;


//////////////////////////////////////////////PID STUFF/////////////////////////////////////////////

const double hoverAngle = 0;
double lastPErr = 0;
double pErrSum = 0;
double pitchOut = 0;
double kp = 1,ki=1,kd=1;

double lastRErr = 0;
double rErrSum = 0;
double rollOut = 0;
double kp2 =1,ki2=1,kd2=1;


double tiltangle = 0;
double pitchAngle;
double rollAngle;

double pitchDeg;
double finalPitchAngle;
double pError;
double dPErr;

double rollDeg;
double finalRollAngle;
double rError;
double dRErr;
double change;



///////////////////////////////////////STUFF WE'RE READING FROM THE TRANSMITTER////////////////////////////////////////



double ChannelVal1=0;
double ChannelVal2=0;
double ChannelVal4=0;
double ChannelVal5=0;
double ChannelVal3=1100;


const int channel1Min= 1000;
const int channel1Max= 1984;

const int channel2Min = 1000;
const int channel2Max = 1984;

const int channel3Min = 990;
const int channel3Max = 1984;

const int channel4Min = 1000;
const int channel4Max = 1984;

const int channel5Min = 1000;
const int channel5Max = 1984;

const int channel6Min = 1000;
const int channel6Max = 1984;



// Creating a global variable to keep track of the previous sensor value
// We will use this to compare with the new value and only add the change to
// the motor

double oldChannelVal1 = 0;
double oldChannelVal2 = 0;
double oldChannelVal3 = 1100;
double oldChannelVal4 = 0;

//arbitrary change before we add to motors
//made it zero, but we have to do something about the fact that channel values fluctuate just from reading them by a few number values
//it doesnt matter that much in terms of throttle but it does in terms of moving left and right
 double minChange = 0;

////////////////////////////////////////////////ARMING STUFF//////////////////////////////////////////////////////////

int Pulse = 1100;
int STATE=1;
int Arming_Time=0;


///////////////////////////////////////////////INPUT/OUTPUT Stuff////////////////////////////////////////////////////////



int channel1=3, channel2=5, channel3=6, channel4=9, channel5=10, channel6=11; //INPUT pins

int motor1=2, motor2=4, motor3=12, motor4=13;                                 //OUTPUT pins

///////////////////////////////////////////////GENERAL VARIABLE STUFF//////////////////////////////////////////////////////////

const float pi=3.14159;
double M_1,M_2,M_3,M_4;

// propeller speed
double w_1 = 1100;    // 1100 = off
double w_2 = 1100;
double w_3 = 1100;
double w_4 = 1100;

// Thrust from each motor
double T_1;
double T_2;
double T_3;
double T_4;
double T; //total thrust

//What the input from the transmitters translate to
double yawIn;
double rollIn;
double pitchIn;
double throttleIn;

void setup() {

  analogReference(EXTERNAL);
  Serial.begin(9600);



//////////////////////////////////////////////////PIN STUFF////////////////////////////////////////////////////////
  //setup input pins
  pinMode(channel1, INPUT);
  pinMode(channel2, INPUT);
  pinMode(channel3, INPUT);
  pinMode(channel4, INPUT);
//  pinMode(channel5, INPUT);
  //setup output pins
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);

//////////////////////////////////////////////////////SENSOR STUFF///////////////////////////////////////////////////
//
//  gyro.enableAutoRange(true);
//  if (!gyro.begin())
//  {
//    //Serial.println("Oops ... unable to initialize the GYRO. Check your wiring!");
//    while (1);
//  }
//
//  /* Initialise the sensor */
//  if(!accel.begin())
//  {
//    /* There was a problem detecting the ADXL345 ... check your connections */
//    //Serial.println("Ooops, no ACCELEROMETER detected ... Check your wiring!");
//    while(1);
//  }
//
//  sensor_t sensor;
//  accel.getSensor(&sensor);
//  accelMax = sensor.max_value;
//  accelMin = sensor.max_value;


////////////////////////////////////////////ARMING STUFF////////////////////////////////////////////////////////

  //arming the motors
  //LOOK HERE:this seems like a callibration type thing, do we actually need this?
  for (Arming_Time = 0; Arming_Time < 500; Arming_Time += 1)
  {
    digitalWrite(motor1,HIGH);
    digitalWrite(motor2,HIGH);
    digitalWrite(motor3,HIGH);
    digitalWrite(motor4,HIGH);
    delayMicroseconds(1100);
    digitalWrite(motor1,LOW);
    digitalWrite(motor2,LOW);
    digitalWrite(motor3,LOW);
    digitalWrite(motor4,LOW);
    delay(20-(Pulse/1000));

  }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

void loop() {


////////////////////////////////////////////SENSOR STUFF////////////////////////////////////////////////////////////
//  now = millis();
//  int timeChange = now - lastTime;
//  sensors_event_t event;
//  gyro.getEvent(&event);
//
//  sensors_event_t event2;
//  accel.getEvent(&event2);
//  readAccel(&event2); //sets x- y- and z- raw;
//  readGyro(&event); // sets xGyro, yGyro and zGyro


//////////////////////////////////////////ANGLE  STUFF///////////////////////////////////////////////////////////
//  double tiltangle = 0;
//  double pitchAngle = findPitch(xAccel, yAccel, zAccel);
//  double rollAngle =  findRoll(xAccel, zAccel);
//  //these values are from the acceleremoter
//
//  dt = 10;//
//  double pitchDeg = zGyro * dt; //angle using the gyroscope, not sure if this should be zGyro or another axis
//  //using a complementary filter
//  double finalPitchAngle = .98*(tiltangle + pitchDeg) +.02*pitchAngle;
//  // now to "hover" we want our angle to be zero
//
//
//


//////////////////////////////////////////PID STUFF////////////////////////////////////////////////////////////
//
//
//  double pError = hoverAngle - finalPitchAngle;
//  pErrSum += pError;
//  double dPErr = pError - lastPErr;
//  pitchOut = kp * pError;
//  lastPErr = pError;
//
//  double rollDeg = yGyro * dt; //not sure if its the correct axis
//
//  //COMPLEMENTARY FILTER NEEDS TO BE REPLACED WITH KALMAN FILTER!!!!!
//  //using a complementary filter
//  double finalRollAngle = .98*(tiltangle + rollDeg) +.02*rollAngle;
//
//
//  // now to "hover" we want our angle to be zero
//  double rError = hoverAngle - finalRollAngle;
//  rErrSum += rError;
//  double dRErr = rError - lastRErr;
//  rollOut = kp2 * rError;
//  lastRErr = rError;
//
//
//  lastTime = now;
//



/////////////////////////////////////TRANSMITTER READING AND PROCESSING///////////////////////////////////////////////////////


  //ChannelVal3 is thrust for all motors, ChannelVal1 is left/right, ChannelVal2 is forward/back, ChannelVal4 yaw
  ChannelVal1= pulseIn(channel1, HIGH);  // Left/Righ
  ChannelVal2= pulseIn(channel2, HIGH);  // Forward/back
  ChannelVal3= pulseIn(channel3, HIGH);  // Thrust
  ChannelVal4= pulseIn(channel4, HIGH);  // yaw
  // ChannelVal5= pulseIn(channel5, HIGH);  // not being used at the moment

//   Serial.println("channelValues before mapping:");
//   Serial.println(ChannelVal1);
//   Serial.println(ChannelVal2);
//   Serial.println(ChannelVal3);
//   Serial.println(ChannelVal4);

  ChannelVal1= map(ChannelVal1,channel1Min,channel1Max,-800,800);
  ChannelVal1= constrain(ChannelVal1,-800,800);

  ChannelVal2= map(ChannelVal2,channel2Min,channel2Max,-800,800);
  ChannelVal2= constrain(ChannelVal2,-800,800);

  ChannelVal3= map(ChannelVal3,channel3Min,channel3Max,1100,2000);
  ChannelVal3= constrain(ChannelVal3,1100,2000);

  ChannelVal4= map(ChannelVal4,channel4Min,channel4Max,-800,800);
  ChannelVal4= constrain(ChannelVal4,-800,800);

   // ChannelVal5= map(ChannelVal5,channel5Min,channel5Max,1200,2000);
   // ChannelVal5= constrain(ChannelVal5,-800,800);


  if(abs(ChannelVal1) < 40)
  {
    ChannelVal1=0;
  }

  if(abs(ChannelVal2) < 40)
  {
    ChannelVal2=0;
  }

  if(abs(ChannelVal4) < 50)
  {
    ChannelVal4=0;
  }

//   Serial.println("channelValues after mapping:");
//   Serial.println(ChannelVal1);
//   Serial.println(ChannelVal2);
//   Serial.println(ChannelVal3);
//   Serial.println(ChannelVal4);



/*
                                                                FLIGHT DYNAMICS:
first im gonna write all of the concepts in the simplest terms as comments and then i'll actually code them making sure im not making
any conceptual mistakes, but also if you guys have time, feel free to get started, Just write it generally so that its easy to modify if it needs to be.
assumptions:
motor 1 is accross from motor 3, and 2 accros from 4 and
that 1 and 3 are spinning in the same direction which is the opposite direction
of motors 2 and 4 and that we're flying in and 'x' orientation, meaning motors 1 and 2 are the 'front'
Modifying it so that it flies in "+" orientation is slightly more complicated but after writing all of this it should be an
easy modification that can be done during flight.

Note: that i separated all the cases, but in general we will want combinations of all of these "motions" so the case will be to either add
or subtract (modify rather than set) to the current w_n so that we get the correct thrust in all directions giving us the motion that we want.
also at this point, i assume that we've managed to figure out the sensors so we can appropriately set w_1-4
ALSO look out for any mistakes and make sure this make sense, and ask me for any clarifications!

T=total thrust
T_n=thrust of nth motor  n=[1,4]
M_n=moment generated by T_n
w_n=angular velocity of nth motor
T_n α (w_n)^2
M_n=L X T_n where L is distance from the center of mass (theoretically the center but not really)

*/

  T_1 = ChannelVal3 * ChannelVal3;
  T_2 = ChannelVal3 * ChannelVal3;
  T_3 = ChannelVal3 * ChannelVal3;
  T_4 = ChannelVal3 * ChannelVal3;
  T = T_1 + T_2 + T_3 + T_4;
  yawIn = ChannelVal4;
  rollIn = ChannelVal2;
  pitchIn = ChannelVal1;
  throttleIn = ChannelVal3;

/*
hover:
1)sum of T generated by all 4 motors = mg (thrust in the z axis but also read condition 2)
2)The direction of all T generated by motors is parallel to force of gravity
3)sum of moments =0
4)sum of angular velocities=0 ie (|w_1| +|w_3|) - (|w_2| +|w_4|)=0
if all of this is true then then qc should be "perfectly" hovering

motion in +/- Z:
This is done by changing condition 1 in 'hover'
we change the speed of all motors increasing them to go up and decreasing them to go down.
note that T is proportional to w^2

up and down(elevation on the z-axis):
*/
  if(abs(throttleIn - oldChannelVal3) > minChange)
  {
    double change = throttleIn - oldChannelVal3;
//    Serial.println("throttleIn");
//    Serial.println(throttleIn);
//    Serial.println("oldChannelVal3");
//    Serial.println(oldChannelVal3);
//    Serial.println("change");
//    Serial.println(change);
    if((w_1 + (change)) > 2000)
  {
    w_1 = 2000;
    w_2 = 2000;
    w_3 = 2000;
    w_4 = 2000;
  }
  else if((w_1 + (change)) < 1100)
  {
    w_1 = 1100;
    w_2 = 1100;
    w_3 = 1100;
      w_4 = 1100;
    }
    else
    {
      w_1 += change;
      w_2 += change;
      w_3 += change;
      w_4 += change;

    }
  }

  if(throttleIn == 1100)
  {
    w_1 = 1100;
    w_2 = 1100;
    w_3 = 1100;
    w_4 = 1100;
  }

//   Serial.println("motor Values after throttle input");
//   Serial.println(w_1);
//   Serial.println(w_2);
//   Serial.println(w_3);
//   Serial.println(w_4);

/*
yaw motion (rotation about z axis):
this is done by changing condition 4 in 'hover'
motion in the yaw direction (yaw_dot) will be proportional to (|w_1| +|w_3|) - (|w_2| +|w_4|) != 0
Note:
total T still has to be parallel to gravity
total T still has to equal gravity
yawAngle=yaw_dot*changeInTime
this shouldnt make the quadcopter change its position in space, just where the 'front' is
*/
//Im thinking if we add half of the sensor reading to each motor and subtract from the other pair it will keep
//the same amount of total thrust and rotate the copter
//ChannelVal4 is mapped to (-800,800);
//If statements check if the input pushes the value over 2000 or under 1200, If so we take the amount it took to reach the
//threshold and add or subtract it from the other, non excessive, value
//if(abs(pitchIn) < 10  && abs(rollIn) < 10 && abs(yawIn) < 10)
//{
//  hover();
//}
  if(abs(yawIn - oldChannelVal4) > minChange)
  {
   double yawChange = yawIn - oldChannelVal4;
    if(w_1 + (yawChange)/2 > 2000)
    {
      double tempMax = w_1 + (yawChange)/2;
      double newVal = tempMax-2000;
      w_1 = 2000;
      w_3 = 2000;
      w_2 -= newVal;
      w_4 -= newVal;

    }
    else if(w_2 + (yawChange)/2 < 1100)
      {
        double tempMin = w_2 - (yawChange)/2;
        double newVal2 = 1100 - tempMin;
        w_1 += newVal2;
        w_3 += newVal2;
        w_2 = 1100;
        w_4 = 1100;

      }
    else
      {
        w_1 += (yawChange)/2;
        w_3 += (yawChange)/2;
        w_2 -= (yawChange)/2;
        w_4 -= (yawChange)/2;

      }
  }


//  Serial.println("motor Values after channel 4 input");
//  Serial.println(w_1);
//  Serial.println(w_2);
//  Serial.println(w_3);
//  Serial.println(w_4);

/*
Now the slightly more complicated part

roll motion(moving left and right):
This is also achived by changing condition 4 in 'hover' but in a different way
this time the imbalance will be such that (|w_1| +|w_4|) - (|w_2| +|w_3|) !=0
similar to yaw motion rotation about the y axis ie tilting(8uj8u) the qc left and right and be proportional to (|w_1| +|w_4|) - (|w_2| +|w_3|) !=0
Note:
rollAngle=roll_dot*changeInTime
condition 3,4 in 'hover' still hold
condition 2 in 'hover' is no longer true
what this means is that the T are now going to be decomposed into a component in the +/- z direction  and a component in the +/- x direction
lift T=T*cos(rollAngle)
L/R  T=T*sin(rollAngle)
We want the qc to stay at the same height so we actually have to adjust this Lift T s.t lift T=T*cos(phi)=mg so w_n will have to be increased
by some factor that i'll figure out later
Torque might be off by a sqrt of something - Juan
*/
  if(abs(rollIn - oldChannelVal2) > minChange)
  {
    double rollChange = rollIn - oldChannelVal2;
    if(w_1 + (rollChange)/2 > 2000)
    {
      double tempMax = w_1 + (rollChange)/2;
      double newVal = tempMax-2000;
      w_1 = 2000;
      w_4 = 2000;
      w_2 -= newVal;
      w_3 -= newVal;
    }
    else if(w_2 + (rollChange)/2 < 1100)
    {
      double tempMin = w_2 - (rollChange)/2;
      double newVal2 = 1100 - tempMin;
      w_1 += newVal2;
      w_4 += newVal2;
      w_2 = 1100;
      w_3 = 1100;

    }
    else
    {
      w_1 += (rollChange)/2;
      w_4 += (rollChange)/2;
      w_2 -= (rollChange)/2;
      w_3 -= (rollChange)/2;

    }
//  double newT = calculateThrust(w_1,w_2,w_3,w_4,pitchAngle,rollAngle);
//  double offsetT = 1 - newT;
    // w_1 += offsetT;
    // w_2 += offsetT;
    // w_3 += offsetT;
    // w_4 += offsetT;

  }

  // Serial.println("motor Values after channel 2 input");
  // Serial.println(w_1);
  // Serial.println(w_2);
  // Serial.println(w_3);
  // Serial.println(w_4);


/*
pitch motion (moving forward and back):

similar to 'roll motion'
condtion 4 in 'hover' changes s.t. (|w_1| +|w_2|) - (|w_3| +|w_4|) !=0
pitch_dot proportional to (|w_1| +|w_2|) - (|w_3| +|w_4|) !=0
pitchAngle=pitch_dot*changeInTime
similarly to 'roll motion' 2 no longer holds and
lift T=T*cos(pitchAngle)
F/B  T=T*sin(pitchAngle)
and we want lift T=mg so that it stays at the same height so again w_n will have to be modified
*/

  if(abs(pitchIn - oldChannelVal1) > minChange)
  {
    double pitchChange = pitchIn - oldChannelVal1;
    if(w_1 + (pitchChange)/2 > 2000)
    {
      double tempMax = w_1 + (pitchChange)/2;
      double newVal = tempMax-2000;
      w_1 = 2000;
      w_2 = 2000;
      w_3 -= newVal;
      w_4 -= newVal;
    }
    else if(w_2 + (pitchChange)/2 < 1100)
    {
      double tempMin = w_2 - (pitchChange)/2;
      double newVal2 = 1100 - tempMin;
      w_1 += newVal2;
      w_2 += newVal2;
      w_3 = 1100;
      w_4 = 1100;
    }
    else
    {
      w_1 += (pitchChange)/2;
      w_2 += (pitchChange)/2;
      w_3 -= (pitchChange)/2;
      w_4 -= (pitchChange)/2;
    }


  double newPitchT = calculateThrust(w_1,w_2,w_3,w_4,pitchAngle,rollAngle);
  double offsetPT = 1 - newPitchT;
    // w_1 += offsetPT;
    // w_2 += offsetPT;
    // w_3 += offsetPT;
    // w_4 += offsetPT;
  }

//  Serial.println("motor Values after channel 1 input");
//  Serial.println(w_1);
//  Serial.println(w_2);
//  Serial.println(w_3);
//  Serial.println(w_4);

//saving old sensor vals;
  oldChannelVal1 = ChannelVal1;
  oldChannelVal2 = ChannelVal2;
  oldChannelVal3 = ChannelVal3;
  oldChannelVal4 = ChannelVal4;

//   Serial.println("motor Values");
//   Serial.println(w_1);
//   Serial.println(w_2);
//   Serial.println(w_3);
//   Serial.println(w_4);

  //Writing to all the motors before the loop finishes
  writeAll(motor1,w_1,motor2,w_2,motor3,w_3,motor4,w_4);

/*
                                                                    THINGS TO DO:
next i'll write how this is changed by flying in the '+' orientation and how we can easily modify the code so that we can change flight
orientations as well as some other more advanced things.
One thing that we should think about is how we want the quadcopter to do after we stop giving it commands from the transmitter.
If we are telling it to go to the left and then stop, should the qc keep going to the left and if we want it to stop completely we have to tell it
to go to the right a proportional amount so that it stays still or if we stop telling it to go left should it go automatically to a hover position?
or both... I actually think that it should be some more complicated combination of both, but i havent thought about it that much.
also if you guys have questions about dynamics things you should add it to this, even if we talk about it so that we dont forget and so that we can
potentially explain them better.
*/



}


////////////////////////////////////////////////MOTOR STUFF////////////////////////////////////////////////////////////////////////

void writeAll(int motor1, double w_1, int motor2, double w_2, int motor3, double w_3, int motor4, double w_4)
{
  Serial.println(motor1);
  Serial.println(motor2);
  Serial.println(motor3);
  Serial.println(motor4);
  //double wValues[4] = {w_1,w_2,w_3,w_4};
  double wValues[4] = {1190,1830,1430,1439};
  int low = 0;
  int high = 3;
  int mid = 1;

  double encValues[4];
  for(int i =0; i < 4; i++)
  {
    encValues[i] = ((wValues[i] * 100) + i);
  }

  double origValues[4] = {0,0,0,0};

  partition(encValues,low,high);

  int orderedMotors[4];
  for(int i = 0; i < 4; i++)
  {
    for(int k =0; k < 4;k++)
      {
        if(abs(((wValues[i] * 100.0) - encValues[k])) == 0)
        {
          origValues[k] = wValues[i];
          orderedMotors[k] = motor1;
        }
        if(abs(((wValues[i] * 100.0) - encValues[k])) == 1)
        {
          origValues[k] = wValues[i];
          orderedMotors[k] = motor2;
        }
        if(abs(((wValues[i] * 100.0) - encValues[k])) == 2)
        {
          origValues[k] = wValues[i];
          orderedMotors[k] = motor3;
        }
        if(abs(((wValues[i] * 100.0) - encValues[k])) == 3)
        {
          origValues[k] = wValues[i];
          orderedMotors[k] = motor4;
        }
      }
  }
 
  for(int i = 0; i < 4; i++)
  {
    Serial.println(orderedMotors[i]);
  }
  for(int i =0; i < 4; i++)
  {
    Serial.println(origValues[i]);
  }
  int tempDelay = origValues[0];
  for(int i = 0; i < 4; i++)
  {
    delayMicroseconds(tempDelay);
    Serial.println(tempDelay);
    digitalWrite(orderedMotors[i], LOW);
    if(i != 3)
    {
      tempDelay = (origValues[i+1] - origValues[i]);
    }
  }
}


 double calculateThrust(double w_1, double w_2, double w_3, double w_4, double pitchAngle, double rollAngle)
 {
   double thrust = ((w_1 * w_1 + w_2 * w_2 + w_3 * w_3 + w_4 * w_4) * cos(pitchAngle)) * cos(rollAngle);
   return thrust;
 }

 void partition(double arr[],int low,int high)
 {
    int mid;

    if(low<high){
         mid=(low+high)/2;
         partition(arr,low,mid);
         partition(arr,mid+1,high);
         mergeSort(arr,low,mid,high);
    }
}

#define MAX 50
void mergeSort(double arr[],int low,int mid,int high)
{ 
    int i,m,k,l;
    double temp[MAX];

    l=low;
    i=low;
    m=mid+1;

    while((l<=mid)&&(m<=high)){

         if(arr[l]<=arr[m]){
             temp[i]=arr[l];
             l++;
         }
         else{
             temp[i]=arr[m];
             m++;
         }
         i++;
    }

    if(l>mid){
         for(k=m;k<=high;k++){
             temp[i]=arr[k];
             i++;
         }
    }
    else{
         for(k=l;k<=mid;k++){
             temp[i]=arr[k];
             i++;
         }
    }

    for(k=low;k<=high;k++){
         arr[k]=temp[k];
    }
}



//this read axis just gets 10 readings from the accelerometer then takes the average
//to get something more accurare


////////////////////////////////////////////////SENSOR STUFF////////////////////////////////////////////////////////////
//
//void readAccel(sensors_event_t* event)
//{
//  long xReading = 0;
//  long yReading = 0;
//  long zReading = 0;
//  //delay(1);
//
//  for (int i = 0; i < sampleSize; i++)
//  {
//    //sensors_event_t event;
//    //accel.getEvent(&event);
//    xReading += event->acceleration.x;
//    yReading += event->acceleration.y;
//    zReading += event->acceleration.z;
//  }
//
//  xRaw = xReading/sampleSize;
//  yRaw = yReading/sampleSize;
//  zRaw = zReading/sampleSize;
////  Serial.println("accel");
////  Serial.println(xRaw);
////  Serial.println(yRaw);
////  Serial.println(zRaw);
//}
//void readGyro(sensors_event_t* event)
//{
//  long xReading = 0;
//  long yReading = 0;
//  long zReading = 0;
//  //delay(1);
//
//  for (int i = 0; i < sampleSize; i++)
//  {
//    //sensors_event_t event;
//    //accel.getEvent(&event);
//    xReading += event->gyro.x;
//    yReading += event->gyro.y;
//    zReading += event->gyro.z;
//
//  }
//
//  xGyro = xReading * RAD_TO_DEG;
//  yGyro = yReading * RAD_TO_DEG;
//  zGyro = zReading * RAD_TO_DEG;
//  //Serial.println("gyro");
////Serial.println(xGyro);
////Serial.println(yGyro);
////Serial.println(zGyro);
//}



/////////////////////////////////////////ANGLE STUFF////////////////////////////////////////////////////////////
//
//double findPitch(double xAccel, double yAccel, double zAccel)
//{
//  return atan2(xAccel,sqrt((yAccel*yAccel)+(zAccel*zAccel))) * RAD_TO_DEG;
//}
//
//
//
//double findRoll(double xAccel, double zAccel)
//{
//  return atan2(-xAccel, zAccel) * RAD_TO_DEG;
//}
