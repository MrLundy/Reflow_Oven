/*Reflow Oven Controller
  CfAR
  Pablo Fernandez
  May 2018
  This code is non-blocking
*/


/*Pin Connections: 
  AD8495 out -> A0
  SerLCD RX -> Digital 2
  Start/Stop Button -> D7
  Speaker -> D9
*/

#include <SoftwareSerial.h>
#include <SerLCD.h>
#include "Keypad.h"
#include <EEPROM.h>

#include <PID_v1.h>

#define RelayPin 3
#define StartStopButtonPin 4
#define SpeakerPin 5

#include "SevSeg.h"
SevSeg sevseg; //Instantiate a seven segment controller object

SoftwareSerial NSS(0,2);
SerLCD theLCD(NSS);

//****************//
//     Keypad     //
//****************//
const byte Rows= 4; //number of rows on the keypad i.e. 4
const byte Cols= 3; //number of columns on the keypad i,e, 3

//we will definne the key map as on the key pad:

char keymap[Rows][Cols]=
{
{'1', '2', '3'},
{'4', '5', '6'},
{'7', '8', '9'},
{'*', '0', '#'}
};

char keypressed;
char prevkeypressed;
//  a char array is defined as it can be seen on the above
//keypad connections to the arduino terminals is given as:

byte rPins[Rows]= {A1,A6,A5,A3}; //Rows 0 to 3
byte cPins[Cols]= {A2,A0,A4}; //Columns 0 to 2

// command for library forkeypad
//initializes an instance of the Keypad class
Keypad kpd= Keypad(makeKeymap(keymap), rPins, cPins, Rows, Cols);

//****************//
//   End Keypad   //
//****************//

//Define Variables we'll be connecting to
double PID_Setpoint = 0;
double PID_Input, PID_Output;

//Specify the links and initial tuning parameters
PID myPID(&PID_Input, &PID_Output, &PID_Setpoint, 1, 0.05, 0.25, DIRECT);

int WindowSize = 500;
unsigned long ProgramStartTime;

#define RefState_RtS 0
#define RefState_SOAK 1
#define RefState_RtR 2
#define RefState_RF 3
#define RefState_COOLING 4
#define RefState_SAFE 5
#define RefState_STOP 6
#define RefState_ERROR 7

#define SettingState_STemp 0
#define SettingState_STime 1
#define SettingState_RTemp 2
#define SettingState_RTime 3


SoftwareSerial mySerial(3,2); // pin 2 = TX, pin 3 = RX (unused)
int     TempPin = 7;     
float    val = 0;           // variable to store the value read
int  temperature;       // Allow to play with the values
int SoakTemp = 150;
int SoakTime = 60;
int ReflowTemp = 215;
int ReflowTime = 45;

int ReflState = 0;
int SettingState = 0;
int SettingsCursor = 0;

int PWM = 0;
unsigned long TimerStart = 0;
int StartOfState = 1;
char buff[4];
unsigned char StartReflow = 0;
int StateTime = 0;
int StartStopButton = 0;
int PrevStartStopButton = 0;
int SpeakerOn = 0;
int BeepStart = 0;
int BeepTime = 0;
int PrevStateTime = 0;
int StartOfBeep = 1;
int BeepDurationStart = 0;
int TotalBeepTime = 0;
int ones;
int tens;
int hundreds;
int InSoakSettings = 1;
int PrevTemperature = 0;
unsigned long LoopTimer = 0;
int OvenOn = 0;

int EEPROM_SoakTemp = 0;
int EEPROM_SoakTime = 1;
int EEPROM_ReflowTemp = 2;
int EEPROM_ReflowTime = 3;


void printlcd( int line, int pos, char message[]);
void clear_display(void);
void SetCursor(int line, int pos);
void printInt(int line, int pos, int num);

void reflowFSM(void);
void RampToSoak(void);    
void Soak(void);
void RamptoReflow(void);
void Reflow(void);
void Cooling(void);
void Safe(void);
void Stop(void);
void Error(void);
void printSettings(int setting);
void InitState(void);
void SettingsFSM(void);

void setup() {
  digitalWrite(RelayPin, LOW);
  Serial.begin(9600); 
  mySerial.begin(9600); // set up serial port for 9600 baud
  pinMode(7, INPUT);

  pinMode(RelayPin, OUTPUT);
  Serial.begin(9600); 
  ProgramStartTime = millis();

   NSS.begin(9600); 
   theLCD.begin();   

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
 // myPID.SetMode(AUTOMATIC);

/*   // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 31250/10;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts  */

  //7Seg
  byte numDigits = 3;
  byte digitPins[] = {48, 52, 50};
  byte segmentPins[] = {53, 49,44 , 46, 47, 51, 43,45};
  bool resistorsOnSegments = false; // 'false' means resistors are on digit pins
  byte hardwareConfig = 3; // See README.md for options
  bool updateWithDelays = false; // Default. Recommended
  bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
  
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments, updateWithDelays, leadingZeros);
  sevseg.setBrightness(90);

  
  clear_display();
  printlcd(0,0,"UVic CfAR Reflow");
  printlcd(1,0,"Oven Controller ");

  delay(2000);

  /* for (int index = 0 ; index < EEPROM.length() ; index++) {

     //Add one to each cell in the EEPROM
     EEPROM[ index ] = 0;
   }

  SoakTemp = EEPROM.read(EEPROM_SoakTemp*3)*100 + EEPROM.read(EEPROM_SoakTemp*3+1)*10 + EEPROM.read(EEPROM_SoakTemp*3+2);
  SoakTime = EEPROM.read(EEPROM_SoakTime*3)*100 + EEPROM.read(EEPROM_SoakTime*3+1)*10 + EEPROM.read(EEPROM_SoakTime*3+2);
  ReflowTemp = EEPROM.read(EEPROM_ReflowTemp*3)*100 + EEPROM.read(EEPROM_ReflowTemp*3+1)*10 + EEPROM.read(EEPROM_ReflowTemp*3+2);
  ReflowTime = EEPROM.read(EEPROM_ReflowTime*3)*100 + EEPROM.read(EEPROM_ReflowTime*3+1)*10 + EEPROM.read(EEPROM_ReflowTime*3+2);*/

  SoakTemp = EEPROM.read(EEPROM_SoakTemp);
  SoakTime = EEPROM.read(EEPROM_SoakTime);
  ReflowTemp = EEPROM.read(EEPROM_ReflowTemp);
  ReflowTime = EEPROM.read(EEPROM_ReflowTime);

  printSettings('S');
  SetCursor(SettingsCursor/3, SettingsCursor+11-3*(SettingsCursor/3));
  theLCD.boxCursorOn();
  //mySerial.write(1);
  //digitalWrite(RelayPin, HIGH); ///////******************************************************************************************************************************************
  val = analogRead(TempPin); // read the input pin
  val = float(val);
  temperature = 0.9583*val-246.94;  // convert the 1024 bit ADC
  sevseg.setNumber(temperature, 0);
}
/*
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  val = analogRead(TempPin); // read the input pin
val = float(val);
val = val*4.839/1024;

temperature = (val - 1.25) / 0.005;  // convert the 1024 bit ADC
//Serial.println(millis());
//Serial.println(temperature);             // debug value
}*/

/*Input: "R" or "S"
  Output: Respective settings displayed on LCD
*/
void printSettings(int setting){
  if (setting == 'S'){
    clear_display();
    printlcd(0,0, "Soak Temp:    C");
    mySerial.write(223);                  //print degree symbol
    sprintf(buff, "%d", SoakTemp);
    printlcd(0, 11, buff);
    printlcd(1,0, "Soak Time:    s");
    sprintf(buff, "%d", SoakTime);
    printlcd(1, 11, buff);
  }

  if (setting == 'R'){
    clear_display();
    printlcd(0,0, "Refl Temp:    C");
    mySerial.write(223);                  //print degree symbol
    sprintf(buff, "%d", ReflowTemp);
    printlcd(0, 11, buff);
    printlcd(1,0, "Refl Time:    s");
    sprintf(buff, "%d", ReflowTime);
    printlcd(1, 11, buff);
  }
}
void printlcd( int line, int pos, char message[]){
   SetCursor(line,pos);
   mySerial.write(message);
}

void SetCursor(int line, int pos){
   if (line == 1){
   mySerial.write(254);
   mySerial.write(pos+192);
  }

  if (line == 0){
   mySerial.write(254);
   mySerial.write(pos+128);
  }
}

void printInt(int line, int pos, int num){
  SetCursor(line,pos);
  theLCD.print(num);
}

void clear_display(void){
  mySerial.write(254); // move cursor to beginning of first line
  mySerial.write(128);

  mySerial.write("                "); // clear display
  mySerial.write("                ");
}

/*  Inputs:
      pin: speaker pin
      freq: output frequency in Hz
      duration: total time in milli seconds
      time_on: in milli seconds
      time_off: in milli seconds

    Only works inside a Reflow State
*/

void beep( int pin, int freq, int duration, int time_on, int time_off){

  if (StartOfBeep == 1){
    BeepDurationStart = millis();
    BeepStart = millis();
    StartOfBeep = 0;
    TotalBeepTime = 0;
  }

  TotalBeepTime = millis() - BeepDurationStart;

  if(TotalBeepTime < duration){
    BeepTime = millis() - BeepStart;
    if(BeepTime >= time_on && SpeakerOn == 1){
      noTone(pin);
      SpeakerOn = 0;
      BeepStart = millis();
    }
    else if(BeepTime >= time_off && SpeakerOn == 0){
      tone(pin, freq);
      SpeakerOn = 1;
      BeepStart = millis();
    }
  }
  else{
    noTone(pin);
    SpeakerOn = 0;
  }
}  

void InitState(void){
  StartOfState = 0;
  TimerStart = millis();
  tone(SpeakerPin,2048);
  SpeakerOn = 1;
  StartOfBeep = 1;
}

void reflowFSM(){
  switch(ReflState){
    case RefState_RtS:
      RampToSoak();
      break;

    case RefState_SOAK:
      Soak();
      break;

    case RefState_RtR:
      RamptoReflow();
      break;

    case RefState_RF:
      Reflow();
      break;

    case RefState_COOLING:
      Cooling();
      break;

    case RefState_SAFE:
      Safe();
      break;

    case RefState_STOP:
      Stop();
      break;

    case RefState_ERROR:
      Error();
      break;

    default:
      Stop();

  }
}

void RampToSoak(void){
  if (StartOfState == 1){
  OvenOn = 1;
  myPID.SetMode(AUTOMATIC);  
  clear_display();
  printlcd(0,0, "Ramp to Soak");
  printlcd(1,0, "Target Temp:   C");
  sprintf(buff, "%d", SoakTemp);
  printlcd(1, 12, buff);
  printlcd(0,13,"0");

  PID_Setpoint = SoakTemp;
  InitState();
  myPID.SetTunings(4,0.2,1);
}
  StateTime = (millis()-TimerStart)/1000;

  if(StateTime < 1)
  beep(SpeakerPin, 2048, 1000, 250, 750);
  
  if(StateTime != PrevStateTime){
    sprintf(buff, "%d", StateTime);
    printlcd(0, 13, buff);
  }
  
  PrevStateTime = StateTime;

  if(temperature <= 40 && StateTime >= 60){
    ReflState = RefState_ERROR;
    StartOfState = 1;
  }
  
  if(temperature >= SoakTemp){
    ReflState = RefState_SOAK;
    StartOfState = 1;
  } 
  
}

void Soak(void){
  if(StartOfState == 1){
  clear_display();
  printlcd(0,0, "Soak");
  printlcd(1,0, "Target Time:    s");
  sprintf(buff, "%d", SoakTime);
  printlcd(1, 13, buff);
  printlcd(0,13,"0");

  InitState();
  
  }

  StateTime = (millis()-TimerStart)/1000;

  if(StateTime < 1)
  	beep(SpeakerPin, 2048, 1000, 250, 750);

  if(StateTime != PrevStateTime){
    sprintf(buff, "%d", StateTime);
    printlcd(0, 13, buff);
  }
  
  PrevStateTime = StateTime;

  if(StateTime == 15)
  	myPID.SetTunings(1,0.05,0.25);
  
  if( StateTime == SoakTime){
    ReflState = RefState_RtR;
    StartOfState = 1;
  }
}

void RamptoReflow(){
  if(StartOfState == 1){
    clear_display();
    printlcd(0,0, "Ramp to Ref.");
    printlcd(1,0, "Target Temp:    C");
    sprintf(buff, "%d", ReflowTemp);
    printlcd(1, 12, buff);
   // mySerial.write(223);                  //print degree symbol
    //printlcd(0,13,"0");

    PID_Setpoint = ReflowTemp;
    InitState();
    //myPID.SetTunings(4,0.2,1);
  }

  StateTime = (millis()-TimerStart)/1000;

  if(StateTime < 1)
  	beep(SpeakerPin, 2048, 1000, 250, 750);

  if(StateTime != PrevStateTime){
    sprintf(buff, "%d", StateTime);
    printlcd(0, 13, buff);
  }
  
  PrevStateTime = StateTime;
  
  if(StateTime == 15)
  	myPID.SetTunings(4,0.2,1);
  
  if(temperature >= ReflowTemp){
    ReflState = RefState_RF;
    StartOfState = 1;
  } 
}

void Reflow(){
  if(StartOfState == 1){
  clear_display();
  printlcd(0,0, "Reflow");
  printlcd(1,0, "Target Time:   s");
  sprintf(buff, "%d", ReflowTime);
  printlcd(1, 12, buff);
  printlcd(0,13,"0");

  InitState();
  }
  StateTime = (millis()-TimerStart)/1000;

  if(StateTime < 1)
  	beep(SpeakerPin, 2048, 1000, 250, 750);

  if(StateTime != PrevStateTime){
    sprintf(buff, "%d", StateTime);
    printlcd(0, 13, buff);
  }
  
  PrevStateTime = StateTime;

  if( StateTime == ReflowTime){
    ReflState = RefState_COOLING;
    StartOfState = 1;
  }
}

void Cooling(){
  if(StartOfState == 1){
  	OvenOn = 0;
    PID_Setpoint = 0;
    //myPID.SetMode(MANUAL);
    clear_display();
    printlcd(0,4, "Cooling");
    printlcd(1,3, "Open Door!");

    InitState();
  }

  StateTime = (millis()-TimerStart)/1000;

  if(StateTime < 3)
  beep(SpeakerPin,2048,3000,250,250);

 if(temperature <= 60){
  ReflState = RefState_SAFE;
  StartOfState = 1;
 }

}

void Safe(){
  if(StartOfState == 1){
    clear_display();
    printlcd(0,0,"Safe to Remove");
    printlcd(1,0,"Board");

    InitState();
  }
  StateTime = (millis()-TimerStart)/1000;

  if(StateTime < 1)
  	beep(SpeakerPin, 2048, 1000, 250, 750);

  if(StateTime == 3){
    ReflState = RefState_STOP;
    StartOfState = 1;
  }
}

void Stop(){
  if(StartOfState == 1){
  	OvenOn = 0;
    PID_Setpoint = 0;
    //myPID.SetMode(MANUAL);
    clear_display();
    printlcd(0,1, "Reflow Stopped");

    InitState();
  }
 
  StateTime = (millis()-TimerStart)/1000;
  if(StateTime <= 1)
    beep(SpeakerPin,2048,2000,500,500);
  else
    beep(SpeakerPin,2048,3000,1000,0);

  if( StateTime == 5){
    ReflState = RefState_RtS;
    StartReflow = 0;
    StartOfState = 1;
    printSettings('S');
    SettingsCursor = 0;
    SetCursor(SettingsCursor/3, SettingsCursor+11-3*(SettingsCursor/3));
    theLCD.boxCursorOn();
  }
}

void Error(void){
  if(StartOfState == 1){
  	OvenOn = 0;
    PID_Setpoint == 0;
    //myPID.SetMode(MANUAL);
    clear_display();
    printlcd(0,5,"ERROR");
    printlcd(1,0,"  Check Sensor ");

    InitState();
  }
    StateTime = (millis()-TimerStart)/1000;
    beep(SpeakerPin,2048,2000,250,250);

    if(StateTime == 3){
      ReflState = RefState_STOP;
      StartOfState = 1;
    }
}

void UpdateSetting(int *digit, int *variable, int EEPROMadd){
  hundreds = *variable/100;
  tens = (*variable-hundreds*100)/10;
  ones = (*variable-hundreds*100-tens*10);
  *digit = keypressed - '0';  //changing char to int
  theLCD.print(*digit);
  if(SettingsCursor != 5){
    SettingsCursor++;
  }
  else
    theLCD.cursorLeft();
  *variable = hundreds*100 + tens*10 + ones;
  /*EEPROM.write(EEPROMadd*3, hundreds);
  EEPROM.write(EEPROMadd*3+1, tens);
  EEPROM.write(EEPROMadd*3+2, ones);*/
  EEPROM.write(EEPROMadd, *variable);
  SetCursor(SettingsCursor/3, SettingsCursor+11-3*(SettingsCursor/3));
  //Serial.println(SettingsCursor);
  //sevseg.setNumber(ReflowTemp,0);
 // Serial.println(ReflowTemp);
}


void SettingsFSM(void){

  keypressed = kpd.getKey();
  if(keypressed){
    if(keypressed == '*'){
      SettingsCursor--;
      if(SettingsCursor == -1){
        SettingsCursor = 5;
        InSoakSettings ^= 1;
        if(InSoakSettings)
          printSettings('S');
        else
          printSettings('R');
      }
    SetCursor(SettingsCursor/3, SettingsCursor+11-3*(SettingsCursor/3));
 
    }
    else if(keypressed == '#'){
      SettingsCursor++;
      if(SettingsCursor == 6){
        SettingsCursor = 0;
        InSoakSettings ^= 1;
        if(InSoakSettings)
          printSettings('S');
        else
          printSettings('R');
      }
    SetCursor(SettingsCursor/3, SettingsCursor+11-3*(SettingsCursor/3));

    }
    else{
      switch(SettingsCursor){
        case 0:
          if(InSoakSettings)
            UpdateSetting(&hundreds, &SoakTemp, EEPROM_SoakTemp);
          else
            UpdateSetting(&hundreds, &ReflowTemp, EEPROM_ReflowTemp);
          break;

        case 1:
          if(InSoakSettings)
            UpdateSetting(&tens, &SoakTemp, EEPROM_SoakTemp);
          else
            UpdateSetting(&tens, &ReflowTemp, EEPROM_ReflowTemp);
          break;

        case 2:
          if(InSoakSettings)
            UpdateSetting(&ones, &SoakTemp, EEPROM_SoakTemp);
          else
            UpdateSetting(&ones, &ReflowTemp, EEPROM_ReflowTemp);
          break; 

        case 3:
          if(InSoakSettings)
            UpdateSetting(&hundreds, &SoakTime, EEPROM_SoakTime);
          else
            UpdateSetting(&hundreds, &ReflowTime, EEPROM_ReflowTime);
          break; 

        case 4:
          if(InSoakSettings)
            UpdateSetting(&tens, &SoakTime, EEPROM_SoakTime);
          else
            UpdateSetting(&tens, &ReflowTime, EEPROM_ReflowTime);
          break;  

        case 5:
          if(InSoakSettings)
            UpdateSetting(&ones, &SoakTime, EEPROM_SoakTime);
          else
            UpdateSetting(&ones, &ReflowTime, EEPROM_ReflowTime);
          break;

        default:
          SettingsCursor = 0;
          SetCursor(SettingsCursor/3, SettingsCursor+11-3*(SettingsCursor/3));          
      }  
    }
  }
}
//prevkeypressed = keypressed;
//}

void loop() {
sevseg.refreshDisplay();
val = analogRead(TempPin); // read the input pin
//
val = float(val);
//val = val*5.0/1023;   //4.7194558
//Serial.println(val);
//int RawTemperature = 0.9583*val-246.94;  // convert the 1024 bit ADC
int RawTemperature = 0.897*val-238.97;  // convert the 1024 bit ADC
//int RawTemperature = 1.2707*val-335.88;

int TempError = abs(PrevTemperature - RawTemperature);
if((millis() - LoopTimer) >= 200){
  if(RawTemperature != PrevTemperature && TempError <= 1 ){
    sevseg.setNumber(RawTemperature, 0);
    Serial.println(RawTemperature);
    temperature = RawTemperature;
    //Serial.println(val*1023/5.0);
  } 
  PrevTemperature = RawTemperature;
  LoopTimer = millis();
}
//Serial.println(PrevTemperature);

PID_Input = temperature;
myPID.Compute();

unsigned long now = millis();
if (now - ProgramStartTime > WindowSize)
{ //time to shift the Relay Window
  ProgramStartTime += WindowSize;
}
if (PID_Output > now - ProgramStartTime){
  if(OvenOn == 1)
    digitalWrite(RelayPin, HIGH);
}
  else 
  digitalWrite(RelayPin, LOW);

  
StartStopButton = digitalRead(StartStopButtonPin);

if( StartStopButton != PrevStartStopButton ){
  if(StartStopButton == HIGH){
    StartReflow ^= 1;
      if(StartReflow == 0){
       StartOfState = 1;
       ReflState = RefState_STOP;
       StartReflow = 1;
      }
      else{
        theLCD.boxCursorOff();
        ReflState = RefState_RtS;
        StartOfState = 1;
      }
  }  
}

PrevStartStopButton = StartStopButton;

if (StartReflow == 1)
  reflowFSM();
else
  SettingsFSM();

}




































































































































































