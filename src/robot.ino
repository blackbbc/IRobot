#include "stdlib.h"
#include "Wire.h"

#include "FreeSixIMU.h"
#include "FIMU_ADXL345.h"
#include "FIMU_ITG3200.h"
#include "HMC5883L.h"
#include "BMP085.h"
#include "Metro.h"

#define LF 0
#define RT 1

int URMdata[9]={0,0,0,0,0,0,0,0,0};
static float IRdata[7] = { 0,0,0,0,0,0,0 };
int BumperValue =7;
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

float _speedleft, _speedright;
int speedleft = 25, speedright = 25;
int _speedtarget[2];


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

    dps.init();
    dps.dumpCalData();
    sixDOF.init();
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
        Motor(2000, RT);
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
    int i = 0;
    Serial.print("URM_Data:,");
    while(Wire.available())
    {
        int c = Wire.read();
        URMdata[i] = c;
        i++;
    }

    // from 1 to 6;
    for(int j = 1; j <= 6; j++)
    {
        int c = URMdata[j];
        Serial.print(c);
        Serial.print(",");
    }
    Serial.println();
}

void IRBumperReader()
{
    Wire.requestFrom(7,8);
    int i = 0;
    Serial.print("IR_Bumper_Data:,");
    while(Wire.available())
    {
        if(i < 7)
        {
            int c = Wire.read();
            IRdata[i] = c;
        }
        else
        {
            int c = Wire.read();
            BumperValue = c;
        }
        i++;
    }

    for(i = 0;i < 7; i++)
    {
        Serial.print(IRdata[i]);
        Serial.print(",");
    }
    Serial.print(BumperValue);
    Serial.print(",");
    Serial.println();
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
