/*
Most of this stuff is just combining the accelerometer and gyroscope stuff that
we found online, i'll comment where i added stuff
ALSO i added a bunch of comments that probably aren't necessary
*/
#include <math.h> //included a math library for trig stuff
//first part comes straight from the gyroTest we dowloaded
#include <Adafruit_L3GD20.h>
Adafruit_L3GD20 gyro;
/*
we don't need xGyro yGyro and zGyro like in accelerometer because we're using the Adafruit_L3GD20 library thing
I dont know how i feel about not really knowing whats happening behind the scenes of that sensor
*/

//now the accelerometer part
const int xAccInput = A0;
const int yAccInput = A1;
const int zAccInput = A2;

//WHICH PINS ARE BEING USED UP by the gyroscope?????
//IN the adafruit website it says just 4 and 5 which seems to good to be true

// Raw Ranges:
/*
need to callibrate acccelerometer first using accelCalibration.ino then write down
these values so that we dont have to callibrate it every time
THESE values are temporary unti we get the actual values
*/
int xRawMin = 512;
int xRawMax = 512;
                    //Remember to change these!
int yRawMin = 512;
int yRawMax = 512;

int zRawMin = 512;
int zRawMax = 512;

// Take multiple samples to reduce noise
const int sampleSize = 10;
//we need to introduce a sample rate as well
int sampleRate=10 //ms


//Ok now next up is the esc stuff
int STATE=1;
int Arming_Time=0;

int pin=3, pin2=5, pin3=6, pin4=9, pin5=10, pin6=11; // these are all the pins that can use pulseIn()
//maybe change this to channel1=3,channel2=5,channel3=6,channel4=9,channel5=10,channl6=11
//also in 'hover mode' we're only using one channel thats going to be throttle
int motor1=2, motor2=4, motor3=12, motor4=3;         //pins we're going to use for output
//note:we're not using arduino's analogwrite() which is their built in pwm

//I think we need some Receiver stuff here so i'll add what i think it is
const int channel1Min= 0;
const int channel1Max=1000; //not real values we need to check what these are


const float pi=3.14159;

unsigned long lastTime=0;
double Output;
double errSum, lastErr;
double kp=1, ki=1, kd=1;

void setup() {

  analogReference(EXTERNAL);
  Serial.begin(9600);

  //setup input pins
  pinMode(pin, INPUT);
  pinMode(pin2, INPUT);
  pinMode(pin3, INPUT);
  pinMode(pin4, INPUT);
  //setup output pins
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);



  //makes sure that the gyroscope is plugged in
  //We should think about adding a screen somewhere on the quadcopter to diagnose errors like this
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  {
    Serial.println("You fucked up the wiring for the gyro...bitch");
    while (1);
  }
  //arming the motors
  //LOOK HERE:this seems like a callibration type thing, do we actually need this?
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

void loop() {
  int hoverAngle =0
  unsigned long now;
  int timeChange = now - lastTime;

  if(timeChange >=sampleRate)
  {
    //read accelerometer stuff
    int xRaw = ReadAxis(xAccInput);
    int yRaw = ReadAxis(yAccInput);
    int zRaw = ReadAxis(zAccInput);

    // Convert raw values to 'milli-Gs"
    long xScaled = map(xRaw, xRawMin, xRawMax, -1000, 1000);
    //might have to change this -1000 to 1000 cause i don't think its actually going to give us G's
    //actually i don't think it'll matter, but it's late at this point
    long yScaled = map(yRaw, yRawMin, yRawMax, -1000, 1000);
    long zScaled = map(zRaw, zRawMin, zRawMax, -1000, 1000);
    /*
    map is a built in function map(value, fromLow, fromHigh, toLow, toHigh)
    you guys can look it up but essentially remaps a value xRaw that is within some
    range [xRawMin,xRawMax and scales it to another range [-1000,1000]
    note: im going to use this later (attempt to) on for the receiver input to the output going
    into the esc's , but i think theres a function constraint() that we should also
    use if i want to do that
    */
    // re-scale to fractional Gs
    float xAccel = xScaled / 1000.0;
    float yAccel = yScaled / 1000.0;
    float zAccel = zScaled / 1000.0;
    /*
    these values are scaled so their in 'g' units to get m/s^2 we'd have to divide
    by 9.8...i think, i need a little bit more tome
    xAccel=xAccel/9.8
    yAccel=yAccel/9.8
    zAccel=zAccel/9.8
    */

    //now the gyroscope stuff
    //these should be in deg/s if not we have to convert them
    //one of us should look
    gyro.read();
    int xGyro = gyro.data.x;
    int yGyro  = gyro.data.y;
    int zGyro = gyro.data.z;

    /*
    at this point im not sure if there are delays written into the gyro.read()
    and im not sure how much delays are going to affect our errors
    everything for one 'reading of the sensors and update outout to esc' should happening
    within 10ms
    */

    //sensorVal3 is thrust for all motors, sensorVal1 is left/right, sensorVal2 is forward/back, sensorVal4 yaw
    int sensorVal1, sensorVal2, sensorVal3,sensorVal4;  //why were these started in loop()
    //int sensorConvert1, sensorConvert2, sensorConvert3, sensorConvert4; dont think i need these anymore
    sensorVal1= pulseIn(pin1,HIGH);
    sensorVal2= pulseIn(pin2, HIGH);
    sensorVal3= pulseIn(pin3, HIGH);
    sensorVal4= pulseIn(pin4, HIGH);

    //for this hover function im going to use sensorval1 as a throttle
    //1200 and 2000 come from the range we had for the esc before
    sensorVal1= map(sensorVal1,channel1Min,channel1Max,1200,2000);
    sensorVal1= constrain(sensorVal1,1200,2000);

    sensorVal2= map(sensorVal2,channel1Min,channel1Max,1200,2000);
    sensorVal2= constrain(sensorVal2,1200,2000);

    sensorVal3= map(sensorVal3,channel1Min,channel1Max,1200,2000);
    sensorVal3= constrain(sensorVal3,1200,2000);

    sensorVal4= map(sensorVal4,channel1Min,channel1Max,1200,2000);
    sensorVal4= constrain(sensorVal4,-800,800);





    /*
    at this point we've read every input that we would need
    it doesnt seem so bad right now, but im a bit worried about the delays that
    we're introducing when we actually do outputs to the esc's
    There are several things i have questions about in my head

    for now im just going to work with two motors and try to balance them

    as a first attemp what im going to do is  say that both of the motors should throttle
    to whatever its receiving and then that one of them should go faster or slower
    deping on the orientation

    later on what we should do is either have an altitude sensor so we can set
    which motor goes faster or slower depends on which action wilk keep the quadcopter
    at the same height
    first thing to do is to use the accelerometer and gyroscope to get an orientation
    the second thing to do is use that orientation and change the ouput of one of the motors
    which is the PID algorith that im first just going to write as a P algorithm
    but i'll do that tomorrow
    */


    //setting up all the stuff that i need to figure out at some point
    //first need to get angle from accelerometer
    int tiltangle =0;
    int pitchAngle = atan2(xAccel,sqrt((yAccel*yAccel)+(zAccel*zAccel)))*180.0/pi;

    //you can also get angle by usin the gyroscope
    //only focusing on one axis at this point

    //int dt = 10;//
    deg=zGyro*dt;  //this doesnt actually give anything at this point

    //using a complementary filter
    double angle = .98*(tiltangle+deg) +.02*pitchAngle;
    // now to "hover" we want our angle to be zero
    double error =hoverAngle-angle; //error
    errSum += error;                //sum of the errors, going to be used with the Integral part of the PID algorithm
    double dErr = (error - lastErr);//diff of errors, going to be used with the derivative part
    Output = kp * error// + ki * errSum*sampleRate + kd * dErr/sampleRate;
    lastErr = error;
    lastTime = now;


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
    double M_1,M_2,M_3,M_4;

    double w_1 = sensorVal3;
    double w_2 = sensorVal3;
    double w_3 = sensorVal3;
    double w_4 = sensorVal3;
    double T_1 = sensorVal3 * sensorVal3;
    double T_2 = sensorVal3 * sensorVal3;
    double T_3 = sensorVal3 * sensorVal3;
    double T_4 = sensorVal3 * sensorVal3;
    double T = T_1 + T_2 + T_3 + T_4;
    double yawIn = sensorVal4;
    double rollIn = sensorVal2;
    double pitchIn = sensorVal1;

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
//sensorVal4 is mapped to (-800,800);
//If statements check if the input pushes the value over 2000 or under 1200, If so we take the amount it took to reach the
//threshold and add or subtract it from the other, non excessive, value
    if(w_1 + (yawIn)/2 > 2000)
      {
        double tempMax = w_1 + (yawIn)/2;
        newVal = tempMax-2000;
        w_1 = 2000;
        w_3 = 2000;
        w_2 -= newVal;
        w_4 -= newVal;
      }
    else if(w_2 - (yawIn)/2 < 1200)
      {
        double tempMin = w_2 - (yawIn)/2;
        newVal2 = 1200 - tempMin;
        w_1 += newVal;
        w_3 += newVal;
        w_2 = 1200;
        w_4 = 1200;
      }
    else
      {
        w_1 += (yawIn)/2;
        w_3 += (yawIn)/2;
        w_2 -= (yawIn)/2;
        w_4 -= (yawIn)/2;
      }

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
//THOUGHTS: If we constantly feed the sensor val to the motors will it continue to roll even if we only want it to keep moving to the left
//with a constant angular acceleration
//add 1-lift to each motor
*/

if(w_1 + (pitchIn)/2 > 2000)
  {
    double tempMax = w_1 + (pitchIn)/2;
    newVal = tempMax-2000;
    w_1 = 2000;
    w_4 = 2000;
    w_2 -= newVal;
    w_3 -= newVal;
  }
else if(w_2 - (pitchIn)/2 < 1200)
  {
    double tempMin = w_2 - (pitchIn)/2;
    newVal2 = 1200 - tempMin;
    w_1 += newVal;
    w_4 += newVal;
    w_2 = 1200;
    w_3 = 1200;
  }
else
  {
    w_1 += (pitchIn)/2;
    w_4 += (pitchIn)/2;
    w_2 -= (pitchIn)/2;
    w_3 -= (pitchIn)/2;
  }
  double newT = T * cos(rollAngle);
  double offsetT = 1 - newT;
  w_1 += offsetT/4;
  w_2 += offsetT/4;
  w_3 += offsetT/4;
  w_4 += offsetT/4;


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

}
//this read axis just gets 10 readings from the accelerometer then takes the average
//to get something more accurare
int ReadAxis(int axisPin)
{
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += analogRead(axisPin);
  }
  return reading/sampleSize; //returns an average
}
