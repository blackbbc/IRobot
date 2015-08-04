/*
 *  iLab@Tongji
 *  Author : Yiren Lu
 *  Date : 06/02/2014
 *
 */


#include "FreeSixIMU.h"
#include "FIMU_ADXL345.h"
#include "FIMU_ITG3200.h"
#include "HMC5883L.h"
#include "Wire.h"
#include "BMP085.h"

#include "Wire.h"
#include "stdlib.h"
#include "Metro.h"


// all the info we need from Robot
int URMdata[9]={0,0,0,0,0,0,0,0,0};
static float IRdata[7] = { 0,0,0,0,0,0,0 };
int BumperValue =7;
float _speedleft, _speedright;
long Temperature = 0, Pressure = 0, Altitude = 0;
float angles[3];
float heading;


void serialEvent(){
    while (Serial.available())
    {
        char inChar = (char)Serial.read();
        if(inChar == 'u')
        {
            //Serial.print(c);

            for (int i =1 ;i<= 6 ;i++)
            {
                Serial.print(URMdata[i]);
                Serial.print(" ");
                //int val = URMdata[i];
                //Serial.write(5);
            }
            Serial.print(_speedleft);
            Serial.print(" ");
            Serial.print(_speedright);
            Serial.print(" ");
            for(int i=0;i<7;i++)
            {
                Serial.print(IRdata[i]);
                Serial.print(" ");
            }
            Serial.print(BumperValue);
            Serial.print(" ");
            Serial.println();
        }
        else if (inChar == 'a')
        {
            int leftWheelSpeed = Serial.parseInt();
            int rightWheelSpeed = Serial.parseInt();
            advance(leftWheelSpeed, -rightWheelSpeed);
        }
    }
}


// declarations for IMU
BMP085 dps = BMP085();


// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

HMC5883L compass;
// Record any errors that may occur in the compass;
int error =0;


//declarations for wheel control

int a, b;

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


//Standard PWM DC control
int E1 = 9;     //M1 Speed Control
int E2 = 10;     //M2 Speed Control
int M1 = 8;    //M1 Direction Control
int M2 = 11;    //M2 Direction Control


int validURM=6;

Metro motorMetro = Metro(500,true);


int count =0;



// The actual speed left and right wheels are not the same under the same speed in code.
int speedleft=25;
int speedright=25;

void initSpeed()
{
    _speedtarget[LF] = speedleft;
    _speedtarget[RT] = -speedright;
}

void advance(char a, char b)
{
    //analogWrite (E1,a);      //PWM Speed Control
    //digitalWrite(M1,HIGH);
    //analogWrite (E2,b);
    //digitalWrite(M2,HIGH);
    _speedtarget[LF] = a;
    _speedtarget[RT] = b;
}


void URMreader()
{
    Wire.requestFrom(8,9);
    int i = 0;
    //Serial.print("URM_Data:,");
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
        //Serial.print(c);
        //Serial.print(",");
    }
    //Serial.println();
}



void IRBumperReader()
{
    Wire.requestFrom(7,8);
    int i = 0;
    //Serial.print("IR_Bumper_Data:,");
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
        //Serial.print(IRdata[i]);
        //Serial.print(",");
    }
    //Serial.print(BumperValue);
    //Serial.print(",");
    //Serial.println();
}


void setup(void)
{
    int i;
    for(i = 9; i <= 11; i++)
        pinMode(i, OUTPUT);

    for(i=4;i<=7;i++)
        pinMode(i, OUTPUT);

    delay(1000);
    digitalWrite(E1, LOW);
    digitalWrite(E2, LOW);

    initSpeed();

    Serial.begin(19200);      //Set Baud Rate
    EncoderInit();
    _perimeterA = 42.72566*1000;
    _FirmPulsePG = 1326;
    // PID
    _proportion = 3;
    _integral = 0.5;
    _derivative = 0.6;
    _maximum = 500;
    _minimum = _maximum*(-1);
    i = 0;


    Wire.begin();
    delay(1000);
    dps.init();
    dps.dumpCalData();
    delay(1000);

    delay(5);
    sixDOF.init(); // init the Acc and Gyro
    delay(5);
    compass = HMC5883L();// init HMC5883

    error = compass.SetScale(1.3);// Set the scale of the compass
    error = compass.SetMeasurementMode(Measurement_Continuous);
    if(error != 0)
        Serial.println(compass.GetErrorText(error));
}

int go =0;

void goback()
{
   advance(-168,168);
}


void getHeading()
{
    // Retrive the raw values from the compass (not scaled).
    MagnetometerRaw raw = compass.ReadRawAxis();
    // Retrived the scaled values from the compass (scaled to the configured scale).
    MagnetometerScaled scaled = compass.ReadScaledAxis();

    // Values are accessed like so:
    int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    heading = atan2(scaled.YAxis, scaled.XAxis);

    float declinationAngle = 0.0457;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if(heading < 0)
    heading += 2*PI;

    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
    heading -= 2*PI;

    // Convert radians to degrees for readability.
    heading = heading * 180/M_PI;
}

void PrintData()
{
  Serial.print("Eular Angle: ");
  Serial.print(angles[0]);
  Serial.print("  ");
  Serial.print(angles[1]);
  Serial.print("  ");
  Serial.print(angles[2]);
  Serial.print("  ");
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print("  ");
  Serial.print("Temperature: ");
  Serial.print(Temperature);
  Serial.print("C");
  Serial.print("  ");
  Serial.print("Altitude: ");
  Serial.print(Altitude);
  Serial.print("cm");
  Serial.print("  ");
  Serial.print("Pressure: ");
  Serial.print(Pressure);
  Serial.println(" Pa");
}


void loop(void)
{
    if (motorMetro.check())
    {
        //read data from IMU
        dps.getTemperature(&Temperature);
        dps.getPressure(&Pressure);
        dps.getAltitude(&Altitude);
        sixDOF.getEuler(angles);
        getHeading();
        //PrintData();

        /// Low level control logics:

        //flag 0: forward 1:turn left 2:turn right 3: backup
        int flag =0;
        URMreader();
        IRBumperReader();

        /*
        Serial.print(_speedtarget[LF]);
            Serial.print(",");
        Serial.print(_speedtarget[RT]);
            Serial.print(",");
            Serial.print(pastCoder[LF]);
            Serial.print(",");
            Serial.print(pastCoder[RT]);
            Serial.print(",");
            // output the current wheel speed
            Serial.print(_speedleft);
            Serial.print(",");
            Serial.print(_speedright);
            Serial.print(",");
            Serial.print(_Loutput);
            Serial.print(",");
            Serial.print(_Routput);
            Serial.print("\n");
        */

        /*
        for(int i = 2;i<=4;i++)
        {
            if(URMdata[i]<20)
            {
                flag=0;
            }
        }
        */

        if (URMdata[3] < 27 || IRdata[2]>200 || BumperValue != 7)
        {
            flag = 3; //backup
        }
        if (URMdata[2] <27 || IRdata[3]>200 || IRdata[4] > 200)
        {
            flag = 1; //turn left
        }
        if (URMdata[6] < 27)
        {
            flag = 0; //forward
        }
        if (URMdata[4] < 27 || IRdata[0] > 200 || IRdata[1] > 200)
        {
            flag = 2; //turn right
        }

        // This is low level control logic
        // Now we use signals from PC
        //go = flag;
    }

    if (BehaviorInterval.check())
    {
        static int lastLspeed = 0;
        static int lastRspeed = 0;
        ResentSpeed();
        static int lastLOutput = 0;
        static int lastROutput = 0;

        float Lpara, Rpara;
        // calcuate the targetspeed to the PWM number;
        if (_Loutput ==0 || _Routput == 0)
        {
            Lpara = TVPIDcal(_speedleft, true);
            Rpara = TVPIDcal(_speedright, false);
            _Loutput = int(TVAffect(Lpara));
            _Routput = int(TVAffect(Rpara));
        }

        _Loutput += (_speedtarget[LF] - _speedleft);
        _Routput += (_speedtarget[RT] - _speedright);

        Motor(_Loutput, LF);
        Motor(_Routput, RT);
    }

    // High level control signals from serial port
    // example : "a 25 25"  (no quotation mark)
    /*
    if (Serial.available() > 0) {
        int inByte = Serial.read();
    if(inByte == 'a'){
        int leftWheelSpeed = Serial.parseInt();
        int rightWheelSpeed = Serial.parseInt();
        advance(leftWheelSpeed, -rightWheelSpeed);
    }
    }*/

    /*
    // do something different depending on the character received.
    // The switch statement expects single number values for each case;
    // in this exmaple, though, you're using single quotes to tell
    // the controller to get the ASCII value for the character.  For
    // example 'a' = 97, 'b' = 98, and so forth:
    switch (inByte) {
    case 'a':
        //Serial.print('a');
        go = 0;
        //forward
        break;
    case 'b':
        //Serial.print('b');
        go = 3;
        //backup
        break;
    case 'l':
        //Serial.print('l');
        go = 1;
        break;
    case 'r':
        //Serial.print('r');
        go = 2;
        break;
    default:
        break;
    }
    */


    // Low level control logics

    // if you want the robot run autonomously (avoiding obstacles), then umcomment below code:
    /*
    if(go ==0){ //forward
        advance (speedleft,-speedright);//forward
    }else if(go==1) { // turn left
        advance(-speedleft,-speedright); //turn left
    }else if(go==2) { // turn right
        advance(speedleft, speedright); //turn right
    }else if(go==3) { // back up
        advance(-speedleft, speedright); //backup
    }
    */
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



/************************************************************
ResentSpeed
This is used for calculating the real time speed of 2 wheels
variable:pasttime-- the past time after the wheels run
         Lduration-- the duration of left wheel
         Rduration--the duration of right wheel
output:_speedleft--the speed of left wheel
       _speedright--the speed of right wheeel
Lauren
************************************************************/
void ResentSpeed()
{
    static int pasttime;
    static unsigned long lasttime;
    static unsigned long now;
    now = millis();
    pasttime = now - lasttime;
    lasttime = now;

    _speedleft = lastspeed(Lduration , pasttime);
    _speedright = lastspeed(Rduration , pasttime);
    /* check the encoder */
    //  Serial.println(pasttime);
    //  Serial.print("  ");
    //  Serial.println(Lduration);

    pastCoder[LF] += Lduration;
    pastCoder[RT] += Rduration;

    Lduration = 0;
    Rduration = 0;
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

    if(!LDirection)
        Lduration++;
    else
        Lduration--;
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

    if(!RDirection)
        Rduration++;
    else
        Rduration--;
}


/************************************************ calculate speed (cm/s) ***********************************************/
/************************************************************
lastspeed
This is used for calculating the real time speed
inout:longs--pulses of the encoder
      diff--the times past
variable:_perimeterA--preimter of the wheel
         _FirmPulsePG--total pulses of one circle
output:dist--the speed of the motor
Lauren
************************************************************/
float lastspeed(int longs,int diff)
{
    double internum = _perimeterA;
    internum /= diff;
    float dist = float(internum*longs);
    dist /= _FirmPulsePG;
    return dist;
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

/************************************************* PID Control ***********************************************/

/************************************************************
TVPIDcal
This is used for calculating the output of PID controller
inout:prevspeed--the real time speed
      target--determine whether the first error
variable:_proportion--the proportion constant
         error--the error of the real time speed and the target speed
         _integral--the integral constant
         sumerror[i]--the sum of the errors
         _derivative--the derivative constant
         derror--the derivative of the error
output: TVPIDcal()-- output of the PID controller
Lauren
************************************************************/
float TVPIDcal(float prevspeed,boolean target)
{
    static int sumerror[2];
    static int i;
    if(target)
        i = 0;
    else
        i = 1;
    int derror;
    int error = _speedtarget[i] - prevspeed;

    sumerror[i] += error;
    sumerror[i] = min(_maximum,sumerror[i]); //limit the range of intergral segment
    sumerror[i] = max(_minimum,sumerror[i]);

    derror = _lasterror[i] - _preverror[i];
    _preverror[i] = _lasterror[i];
    _lasterror[i] = error;

    return (_proportion*error+_integral*sumerror[i]+_derivative*derror);
}

/************************************************************
TVAffect
This is used for limiting the output of the PID controller
inout:pidpara--the output of the PID controller
output:TVAffect()
Lauren
************************************************************/
int TVAffect(float pidpara)
{
    float result = 500;
    float factor;

    if(pidpara>_maximum)
        factor = 1;
    else if (pidpara>0)
        factor = pidpara/_maximum;
    else if (pidpara<_minimum)
        factor = -1;
    else
        factor = pidpara/_maximum;

    result *= factor;
    result += 1500;

    return result;
}
