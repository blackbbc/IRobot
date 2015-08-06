#include "stdlib.h"
#include "Wire.h"

#include "FreeSixIMU.h"
#include "FIMU_ADXL345.h"
#include "FIMU_ITG3200.h"
#include "HMC5883L.h"
#include "BMP085.h"
#include "Metro.h"


//all the info we need from robot
int URMdata[9]={0,0,0,0,0,0,0,0,0};
static float IRdata[7] = { 0,0,0,0,0,0,0 };
int BumperValue =7;
float _speedleft, _speedright;
long Temperature = 0, Pressure = 0, Altitude = 0;
float angles[3];
float heading;

Metro behavior = Metro(25, true);
Metro motor = Metro(500, true);

//Standard PWM DC control
int E1 = 9;     //M1 Speed Control
int E2 = 10;     //M2 Speed Control
int M1 = 8;    //M1 Direction Control
int M2 = 11;    //M2 Direction Control


BMP085 dps = BMP085();                    //Temperature, Altitude, Pressure
FreeSixIMU sixDOF = FreeSixIMU();
HMC5883L compass = HMC5883L();            //Compass
int compassError =0;


//encoder
#define Pbn 8
int counter = 0;
const byte encoder0pinA = 2;
const byte encoder0pinB = 4;
const byte encoder1pinA = 3;
const byte encoder1pinB = 5;

byte encoder0PinALast;
byte encoder1PinALast;

int Lduration;
int Rduration;
boolean LDirection;
boolean RDirection;
byte wheelDir = 0x00;
int _CMDspeed[2]={0,0};


float _perimeterA, _FirmPulsePG;
int pastCoder[2];
long totalCoder[2];
#define LF 0
#define RT 1
float _proportion, _integral, _derivative, _maximum, _minimum;
int _speedtarget[2];
double _lasterror[2];
double _preverror[2];
int i;
float _Loutput=0.0, _Routput=0.0;

Metro DataTrans = Metro(1000, true);
Metro BehaviorInterval = Metro(25, true);

int speedleft = 25, speedright = 25;


void setup()
{
    Serial.begin(9600);
    Wire.begin();

    int i;
    for(i = 9; i <= 11; i++)
        pinMode(i, OUTPUT);

    for(i=4;i<=7;i++)
        pinMode(i, OUTPUT);

    delay(1000);
    digitalWrite(E1, LOW);
    digitalWrite(E2, LOW);

    initSpeed();

    EncoderInit();
    _perimeterA = 42.72566*1000;
    _FirmPulsePG = 1326;
    // PID
    _proportion = 3;
    _integral = 0.5;
    _derivative = 0.6;
    _maximum = 500;
    _minimum = _maximum*(-1);

    delay(1000);
    dps.init();
    dps.dumpCalData();

    delay(1000);
    sixDOF.init();

    delay(1000);
    compassError = compass.SetScale(1.3);// Set the scale of the compass
    compassError = compass.SetMeasurementMode(Measurement_Continuous);
    if (compassError!=0)
        Serial.println(compass.GetErrorText(compassError));
}

void loop()
{
    if (motor.check())
    {

        //flag 0: forward 1:turn left 2:turn right 3: back
        int flag =0;
        URMreader();
        IRBumperReader();

        Motor(2000, LF);
        Motor(-2000, RT);
    }
}

void initSpeed()
{
    _speedtarget[LF] = speedleft;
    _speedtarget[RT] = -speedright;
}

void URMreader()
{
    Wire.requestFrom(8,9);
    int i=0;
    Serial.print("URM_Data:,");
    while (Wire.available())
    {
        int c = Wire.read();
        URMdata[i]=c;
        i++;
    }

    // from 1 to 6;
    for(int j =1;j<=6;j++)
    {
        int c= URMdata[j];
        Serial.print(c);
        Serial.print(",");
    }
    Serial.println();
}

void IRBumperReader()
{
    Wire.requestFrom(7,8);
    int i=0;
    Serial.print("IR_Bumper_Data:,");
    while(Wire.available())
    {
        if(i<7)
        {
            int c = Wire.read();
            IRdata[i]=c;
        }
        else
        {
            int c = Wire.read();
            BumperValue = c;
        }
        i++;
    }

    for(i = 0;i<7;i++)
    {
        Serial.print(IRdata[i]);
        Serial.print(",");
    }
    Serial.print(BumperValue);
    Serial.print(",");
    Serial.println();
}


/************************************************************
wheelSpeed
This is used for reading the value of pulses of the encoder
variable:encoder0pinB and encoder0pin--the output if the sensor
         Direction--the direction of wheel turning
output:duration--the number of the pulses
Pepin
************************************************************/
void LwheelSpeed()  // the encoder code
{
  int Lstate = digitalRead(encoder0pinA);
  if((encoder0PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder0pinB);
    if(val == LOW && LDirection)
    {
      LDirection = false; //Reverse
    }
    else if(val == HIGH && !LDirection)
    {
      LDirection = true;  //Forward
    }
  }
  encoder0PinALast = Lstate;

  if(!LDirection)  Lduration++;
  else  Lduration--;
}


void RwheelSpeed()
{
  int Rstate = digitalRead(encoder1pinA);
  if((encoder1PinALast == LOW) && Rstate==HIGH)
  {
    int val = digitalRead(encoder1pinB);
    if(val == LOW && RDirection)
    {
      RDirection = false; //Reverse
    }
    else if(val == HIGH && !RDirection)
    {
      RDirection = true;  //Forward
    }
  }
  encoder1PinALast = Rstate;

  if(!RDirection)  Rduration++;
  else  Rduration--;
}



/************************************************************
EncoderInit
This is used for initializing the encoder module
variable:encoder0pinB--the output of the encoder
Pepin
************************************************************/
void EncoderInit() //init of the encoder
{
  LDirection = true;
  RDirection = true;//default -> Forward
  pinMode(encoder0pinA,INPUT_PULLUP);
  pinMode(encoder0pinB,INPUT_PULLUP);
  pinMode(encoder1pinA,INPUT_PULLUP);
  pinMode(encoder1pinB,INPUT_PULLUP);
  attachInterrupt(0, LwheelSpeed, CHANGE);
  attachInterrupt(1, RwheelSpeed, CHANGE);
}


/************************************************* Motor Control ***********************************************/
/************************************************************
Motor
This is used for controlling the motors
inout:value--the PWM value;input of motor
      whichwheel--decide which wheel to control
Pepin
************************************************************/
void Motor(int value,byte whichwheel)
{
    int a, b;
    value = constrain(value,1000,2000);

    if(whichwheel == LF)
    {
        if(value>1500)
        {
            a=(value-1500)/1.961;
            analogWrite (E1,a);
            digitalWrite(M1,HIGH);
        }
        else
        {
            a=(1500-value)/2;
            analogWrite (E1,-a);
            digitalWrite(M1,HIGH);
        }
    }
    else if(whichwheel == RT)
    {
        if(value>1500)
        {
            b=(value-1500)/1.961;
            analogWrite (E2,b);
            digitalWrite(M2,HIGH);
        }
        else
        {
            b=(1500-value)/2;
            analogWrite (E2,-b);
            digitalWrite(M2,HIGH);
        }
    }
}
