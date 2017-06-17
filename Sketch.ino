
#include <avr/pgmspace.h>
#include <EEPROM.h> //Needed for reading/writing to EEPROM
#include <ctype.h> //Needed for some additional type functions that arn't in default arduino
#include <PID_v1.h>//Needed for PID
#include <PID_AutoTune_v0.h> //Needed for Autotuner
//if you run into problems with this library you'll need to install it manually

//ENTER THIS
#define VOFFSETA 0 //in volts
#define VOFFSETB 0 //in volts
//
//pins
#define THERM1 A0
#define THERM2 A1
#define HLED 11 //heater LED
//give some sort of indication if the heater has been turned on
#define PWM_OUT 12 //heater coil

//EEPROM adresses for persistent stored variables
#define E_KP 101
#define E_KI 102
#define E_KD 103
#define E_TARGET 104

//min and max temperature values
#define MINVALIDTEMP 0
#define MAXVALIDTEMP 200

//PID/Autotune pid constants
//Autotune stuff
#define AUTOTUNESTEP 5 // 2% change in output
#define AUTOTUNENOISEBAND 1 // fluxuations of 1degree seems reasonable for now.
#define OUTPUTSTART 20
#define AUTOTUNELOOKBACK 20
double temp, PIDPWMout, targettemp, kp, ki, kd,voffset;
bool logdata, heating,tuning;
int Bytes2Read = 0;
int temporarytemp,toread, readpin,Itemp;
String StTemp;
PID myPID(&temp, &PIDPWMout, &targettemp,kp,ki,kd, DIRECT);
//direct because add heat(PWM) -> output++
// temp = input(from thermistor) PIDPWMout = pwm output for heater
//kp ki kd are PID values
PID_ATune aTune(&temp, &PIDPWMout);
//set up an autotuner

// LONG table
//from 0 to 200deg c in 5 degree increments
float in[] = { 100, 100.7228, 102.6745, 104.6232, 106.5691, 108.5121, 110.4522, 112.3894, 114.3237, 116.2551, 118.1836, 120.1093, 122.032, 123.9519, 125.8689, 127.783, 129.6942, 131.6025, 133.508, 135.4105, 137.3102, 139.207, 141.1009, 142.9919, 144.88, 146.7652, 148.6475, 150.527, 152.4035, 154.2772, 156.148, 158.0159, 159.8809, 161.743, 163.6023, 165.4586, 167.3121, 169.1627, 171.0104, 172.8552, 174.6971, 176.5361, 178.3722, 180.2055, 182.0359, 183.8633, 185.6879, 187.5096, 189.3284, 191.1444, 192.9574, 194.7675, 196.5748 };

void setup()
{
  //restore previous tunings
  kp = EEPROM.read(E_KP);
  ki = EEPROM.read(E_KI);
  kd = EEPROM.read(E_KD);
  targettemp = EEPROM.read(E_TARGET);
  //set PID stuff
  myPID.SetOutputLimits(0,255); //pwm output from 0-255
  aTune.SetNoiseBand(AUTOTUNENOISEBAND);
    aTune.SetOutputStep(AUTOTUNESTEP);
    aTune.SetLookbackSec(AUTOTUNELOOKBACK);
  //pin setup
  pinMode(PWM_OUT, OUTPUT);     // 24v output
  pinMode(HLED, OUTPUT);   
  pinMode(THERM1 ,INPUT);    //thermistor 1 (brown and white)
  pinMode(THERM2 ,INPUT);    //thermistor 2 (green and white)
  
  Serial.begin(9600);
  Serial.flush();
    logdata = false;
  myPID.SetMode(AUTOMATIC);
  PrintCMDMessage();
  delay(200);
}

void loop() {

  Bytes2Read=Serial.available();
  if (Bytes2Read > 0) 
  {
    int inByte = Serial.read();
    if (inByte>96) inByte=inByte-32;  //convert to upper case
  {
  switch (inByte)
    {
    case 'S': 
    logdata = true;
      break;
    
      case 'X': 
      logdata = false;
      break;
    
    case 'T':
    //set the temp, enter as 100d for 100 degrees
    //if you do something other than this then this section will be skipped
     StTemp= Serial.readStringUntil('d');
    temporarytemp = ConfirmTemp(StTemp);
    if (temporarytemp != -1) //-1 is error state
    //either contains something other than numbers or out of range
    {
      targettemp = temporarytemp;
      EEPROM.write(E_TARGET,targettemp);
    }
    break;
    
    case 'H':
    //set heating to on
    heating = true;
      digitalWrite(HLED,HIGH);
    break;
    
    case 'O':
    //turns heater off
    heating = false;
      digitalWrite(HLED,LOW);
     break;
    case 'A':
    //runs PID autotuner
    tuning = true;
    TunePID();
     break;
    
  }
  }
  }
  
//default behaviours
if (logdata)
  //log the data to serial in the form of
  //temp1,temp2\n
{
//analog to temp conversions
int therm1temp = analogRead(THERM1);
int therm2temp = analogRead(THERM2);
Serial.print(therm1temp);
Serial.print(",");
Serial.print(therm2temp);
Serial.println("");

//therm1temp = AnalogToResistance(therm1temp,1);
//therm2temp = AnalogToResistance(therm2temp,2);
//Serial.print(therm1temp);
//Serial.print(",");
//Serial.print(therm2temp);
//Serial.println(",");

//therm1temp = ResistanceToTemp(therm1temp,in,200);
//
//therm2temp = ResistanceToTemp(therm2temp,in,200);
//Serial.print(therm1temp);
//Serial.print(",");
//Serial.print(therm2temp);
//Serial.println(",");
delay(1000);
//print


}
if (heating)
{

  if (analogRead(THERM1) > analogRead(THERM2))
  {
    toread = THERM1;
    readpin = 1; //bad name - read thermistor 1 or 2
  }
  else
  {
    toread = THERM2;
    readpin=2;
  }
  temp = ResistanceToTemp(AnalogToResistance(analogRead(toread),readpin),in,200);
  myPID.Compute(); //updates PIDPWMout aka the PWM signal out based on input aka current temp
  analogWrite(PWM_OUT,PIDPWMout); // write that value to heater.
}
}
  
  
  int ConfirmTemp(String input)
  //validates input to see if it's a valid temp
  //returns -1 if an error occcurs (out of range, non numeric)
  //I miss pythons string handling.
  {
    bool valid = false;
    if (StTemp.length()!=0)
    {
      bool allNumbers = true;
      for (int ii=0;ii <= StTemp.length();ii++)
      {
        if (!isDigit(StTemp.charAt(ii)))
        {
        allNumbers = false;
        }
      }
    if(allNumbers)
    {
      Itemp = StTemp.toInt(); //integer temporary
      if (Itemp > MINVALIDTEMP && Itemp <MAXVALIDTEMP)
      {
        // a valid value has been entered
      valid = true;
      }
    }
    }
      if (valid)
      {
      return Itemp;
      }
      else
      {
        return -1;
      }
  }
  
void TunePID()
//set tuning to true and call this
{
    byte val = (aTune.Runtime()); //returns 1 once tuning is done, 0 if still needs to tune
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
  EEPROM.write(E_KP, kp);
  EEPROM.write(E_KI, ki);
  EEPROM.write(E_KD, kd);
    }
}


double AnalogToResistance(int analog, int tempsensor)
{
  // Rx =    (R2*R3 + R3* (R1+R2)Vdiff/Vin )/ (R1- (R1+R2)*Vdiff/ Vin)
  //Don't know VB directly
  //But feeds through op amp
  // Vinto analog is  Voffset + (R7/R6)(vdiff)
  // where R7 = 10k, R6 = 1k (for op amp only)
  //rearrange to find vdiff
  if (tempsensor = 0)
{
  voffset = VOFFSETA;
}
if (tempsensor = 1)
{
  voffset = VOFFSETB;
}
  //for this section all resistors except thermistor = 100ohm, 5v input
  float vanalog = analog*(5/1023); // convert analogin to voltage. -> 10 bit ADC = 2^10-1 positions==1023
  float vdiff = (vanalog - voffset)/(10); //10 == gain
  float Rx = (100*100 + 100*(100*100)*vdiff/5)/(100-(100+100)*vdiff/5);
  //
  return Rx;
}


float ResistanceToTemp(float val, float* _in, uint8_t size)
//http://stackoverflow.com/questions/30012866/how-to-read-temperature-using-arduino-uno-board-with-pt100-rtd-sensor
//my table is in increments of 5 degrees starting at 0
{
  // calculate if value is out of range 
  if (val < _in[0] ) return -0.99;
  if (val > _in[size-1] ) return 199.99;

  //  search for 'value' in _in array to get the position No.
  uint8_t pos = 0;
  while(val > _in[pos]) pos++;  

  // handles the 'rare' equality case
  if (val == _in[pos]) return pos;

  float r1 = _in[pos-1];
  float r2 = _in[pos];
  int c1 = (pos-1)*5; //modification from original code - in 5degree increments
  int c2 = pos*5;

 return c1 + (val - r1) / (r2-r1) * (c2-c1);
}

void PrintCMDMessage()
{
  Serial.print("INSTRUCTIONS \n");
  Serial.println("NOTE: IF controling this via matlab it's worth using a newline character");
  Serial.println("S = Turn data logging out (Returns temp1 (Green and White),temp2(Brown and White) in degrees celcius");
  Serial.println("X = turn logging off");
  Serial.println("T = Set temperature. end tempereture with d to finish, will reject non valid: Example 100d");
  Serial.println("A = AutoTune PID");
  Serial.println("H = Heater on");
  Serial.println("O = Heater off");
}


void printDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

   Serial.print (int(val));  //prints the int part
   Serial.print("."); // print the decimal point
   unsigned int frac;
   if(val >= 0)
     frac = (val - int(val)) * precision;
   else
      frac = (int(val)- val ) * precision;
   int frac1 = frac;
   while( frac1 /= 10 )
       precision /= 10;
   precision /= 10;
   while(  precision /= 10)
       Serial.print("0");

   Serial.println(frac,DEC) ;
}
