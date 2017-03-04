/*
EDCduino - Simple interceptor ECU for the Arduino Mega 2560 platform heavily based on Speeduino stand-alone ECU by Josh Stewart

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
*/


//**************************************************************************************************
//**************************************************************************************************

//**************************************************************************************************
// Config section
#define engineSquirtsPerCycle 2 //Would be 1 for a 2 stroke
//**************************************************************************************************

#include "globals.h"//required, needs stripping
#include "utils.h"// has pinmapping. PW,PW_SD, PW_AN, pinmapping have been modified
#include "table.h"//required
//#include "scheduler.h"// has the timers for ign and fuel
#include "comms.h"//required
#include "math.h"//required
#include "corrections.h"// needs dummy corrections assignment
//#include "timers.h"// manages dwelltime, idle uses timer 2, 250ms interrupt, not required
//#include "display.h"//not required, for adafruit oled display
#include "decoders.h"// returns rpm
//#include "idle.h"//not required
#include "auxiliaries.h"//not required, frees up timer1
#include "fastAnalog.h"
//#include "libs/PID_v1/PID_v1.h"//not required
#include "DigiPotX9Cxxx.h"

#ifdef __SAM3X8E__
//Do stuff for ARM based CPUs
#else
#include "storage.h"
#endif
//pin definitions
#define pin_crankinH 48
#define pin_crankinL 49
#define pin_caminH 21

#define pin_cranko 18
#define pin_camo 19
#define  camHteeth  1
#define  camLteeth   35

#define cromiss1 13
#define cromiss2 14
#define cromiss3 73
#define cromiss4 74


//variable definitions
//sync variables
volatile int syncstat = 0;
volatile int crinteeth = 1;
volatile int croteeth = 1;


//timevariables
volatile uint16_t per24thteethH = 0;
volatile uint16_t per24thteethL = 0;
volatile uint16_t delta_t1 = 0;
volatile uint16_t delta_t2 = 0;
volatile uint16_t deltaL_t1 = 0;
volatile uint16_t deltaL_t2 = 0;
volatile uint16_t delta_t = 0;
volatile uint16_t deltaL_t = 0;

//advance/retard variables
volatile int cur_degsH = 0;//in 0.5deg resolution, int numbers
volatile int cur_degsL = 0;
volatile int target_degs = 0;
volatile uint8_t target_pot = 0;
int target_degs1;
uint32_t y = 0;


//delta calc var
volatile int aH = 0;       volatile int aL = 0;
volatile uint8_t aH1 = 0;      volatile uint8_t aL1 = 0;
volatile uint16_t b = 0;   volatile uint16_t c = 0;
volatile uint16_t e = 0;   volatile uint16_t d = 0;
volatile uint16_t t1 = 0;  volatile uint16_t t2 = 0;
volatile uint16_t t3 = 0;  volatile uint8_t aH2 = 0;
volatile uint8_t misteeth = 0;
volatile uint8_t misteeth1 = 0;
volatile uint8_t iofH = 0; volatile uint8_t iofL = 0;
volatile uint16_t OCR3A1 = 0; volatile uint16_t OCR3B1 = 0;

struct config1 configPage1;
struct config2 configPage2;
struct config3 configPage3;
struct config4 configPage4;

int req_fuel_uS, inj_opentime_uS;
#define MAX_RPM 15000 //This is the maximum rpm that the ECU will attempt to run at. It is NOT related to the rev limiter, but is instead dictates how fast certain operations will be allowed to run. Lower number gives better performance
int (*getRPM)();
volatile byte startRevolutions = 0; //A counter for how many revolutions have been completed since sync was achieved.
volatile byte ign1LastRev;
volatile byte ign2LastRev;
volatile byte ign3LastRev;
volatile byte ign4LastRev;
bool ignitionOn = false; //The current state of the ignition system
bool fuelOn = false; //The current state of the ignition system
bool fuelPumpOn = false; //The current status of the fuel pump

struct table3D fuelTable; //16x16 fuel map
struct table3D ignitionTable; //16x16 ignition map
struct table3D afrTable; //16x16 afr target map
struct table3D boostTable; //8x8 boost map
struct table3D vvtTable; //8x8 vvt map
struct table2D taeTable; //4 bin TPS Acceleration Enrichment map (2D)
struct table2D WUETable; //10 bin Warm Up Enrichment map (2D)
struct table2D dwellVCorrectionTable; //6 bin dwell voltage correction (2D)
struct table2D injectorVCorrectionTable; //6 bin injector voltage correction (2D)
struct table2D IATDensityCorrectionTable; //9 bin inlet air temperature density correction (2D)
struct table2D IATRetardTable; //6 bin ignition adjustment based on inlet air temperature  (2D)
byte cltCalibrationTable[CALIBRATION_TABLE_SIZE];
byte iatCalibrationTable[CALIBRATION_TABLE_SIZE];
byte o2CalibrationTable[CALIBRATION_TABLE_SIZE];

//These variables are used for tracking the number of running sensors values that appear to be errors. Once a threshold is reached, the sensor reading will go to default value and assume the sensor is faulty
byte mapErrorCount = 0;
byte iatErrorCount = 0;
byte cltErrorCount = 0;


unsigned long counter;
unsigned long currentLoopTime; //The time the current loop started (uS)
unsigned long previousLoopTime; //The time the previous loop started (uS)

unsigned long MAPrunningValue; //Used for tracking either the total of all MAP readings in this cycle (Event average) or the lowest value detected in this cycle (event minimum)
unsigned int MAPcount; //Number of samples taken in the current MAP cycle
byte MAPcurRev = 0; //Tracks which revolution we're sampling on


byte coilHIGH = HIGH;
byte coilLOW = LOW;
byte fanHIGH = HIGH;             // Used to invert the cooling fan output
byte fanLOW = LOW;               // Used to invert the cooling fan output

struct statuses currentStatus;
volatile int mainLoopCount;
byte deltaToothCount = 0; //The last tooth that was used with the deltaV calc
int rpmDelta;
byte ignitionCount;
unsigned long secCounter; //The next time to incremen 'runSecs' counter.

void setup()
{
  pinMode(pin_crankinH, INPUT);
  pinMode(pin_crankinL, INPUT);
  pinMode(pin_caminH, INPUT);
  pinMode(pin_camo, OUTPUT);
  pinMode(pin_cranko, OUTPUT);
  pinMode(8, OUTPUT);
  //Setup the dummy fuel and ignition tables
  //dummyFuelTable(&fuelTable);
  //dummyIgnitionTable(&ignitionTable);
  table3D_setSize(&fuelTable, 16);
  table3D_setSize(&ignitionTable, 16);
  table3D_setSize(&afrTable, 16);
  table3D_setSize(&boostTable, 8);
  table3D_setSize(&vvtTable, 8);

  loadConfig();

  //Repoint the 2D table structs to the config pages that were just loaded
  taeTable.valueSize = SIZE_BYTE; //Set this table to use byte values
  taeTable.xSize = 4;
  taeTable.values = configPage2.taeValues;
  taeTable.axisX = configPage2.taeBins;
  WUETable.valueSize = SIZE_BYTE; //Set this table to use byte values
  WUETable.xSize = 10;
  WUETable.values = configPage1.wueValues;
  WUETable.axisX = configPage2.wueBins;
  //The WUE X axis values are hard coded (Don't ask, they just are)
  WUETable.axisX[0] = 0;
  WUETable.axisX[1] = 11;
  WUETable.axisX[2] = 22;
  WUETable.axisX[3] = 33;
  WUETable.axisX[4] = 44;
  WUETable.axisX[5] = 56;
  WUETable.axisX[6] = 67;
  WUETable.axisX[7] = 78;
  WUETable.axisX[8] = 94;
  WUETable.axisX[9] = 111;

  dwellVCorrectionTable.valueSize = SIZE_BYTE;
  dwellVCorrectionTable.xSize = 6;
  dwellVCorrectionTable.values = configPage2.dwellCorrectionValues;
  dwellVCorrectionTable.axisX = configPage3.voltageCorrectionBins;
  injectorVCorrectionTable.valueSize = SIZE_BYTE;
  injectorVCorrectionTable.xSize = 6;
  injectorVCorrectionTable.values = configPage3.injVoltageCorrectionValues;
  injectorVCorrectionTable.axisX = configPage3.voltageCorrectionBins;
  IATDensityCorrectionTable.valueSize = SIZE_BYTE;
  IATDensityCorrectionTable.xSize = 9;
  IATDensityCorrectionTable.values = configPage3.airDenRates;
  IATDensityCorrectionTable.axisX = configPage3.airDenBins;
  IATRetardTable.valueSize = SIZE_BYTE;
  IATRetardTable.xSize = 6;
  IATRetardTable.values = configPage2.iatRetValues;
  IATRetardTable.axisX = configPage2.iatRetBins;

  //Setup the calibration tables
  loadCalibration();
  //Set the pin mappings
  setPinMapping(configPage1.pinMapping);

  pot_setup(2, 3, 4);
  pot_reset();


  //Set the tacho output default state
  digitalWrite(pinTachOut, HIGH);

  initialiseFan();
  initialiseAuxPWM();

  //Once the configs have been loaded, a number of one time calculations can be completed
  req_fuel_uS = configPage1.reqFuel * 100; //Convert to uS and an int. This is the only variable to be used in calculations
  inj_opentime_uS = configPage1.injOpen * 100; //Injector open time. Comes through as ms*10 (Eg 15.5ms = 155).

  //Begin the main crank trigger interrupt pin setup
  //The interrupt numbering is a bit odd - See here for reference: http://arduino.cc/en/Reference/AttachInterrupt
  //These assignments are based on the Arduino Mega AND VARY BETWEEN BOARDS. Please confirm the board you are using and update acordingly.
  byte triggerInterrupt = 0; // By default, use the first interrupt
  byte triggerInterrupt2 = 1;
  currentStatus.RPM = 0;
  currentStatus.hasSync = false;
  currentStatus.runSecs = 0;
  currentStatus.secl = 0;
  triggerFilterTime = 0; //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be disgarded as noise. This is simply a default value, the actual values are set in the setup() functinos of each decoder


  //Set the trigger function based on the decoder in the config
  switch (configPage2.TrigPattern)
  {
    default:
      getRPM = stdGetRPM;
  }

  //End crank triger interrupt attachment


  //Initial values for loop times
  previousLoopTime = 0;
  currentLoopTime = micros();

  Serial.begin(115200);

  //This sets the ADC (Analog to Digitial Converter) to run at 1Mhz, greatly reducing analog read times (MAP/TPS)
  //1Mhz is the fastest speed permitted by the CPU without affecting accuracy
  //Please see chapter 11 of 'Practical Arduino' (http://books.google.com.au/books?id=HsTxON1L6D4C&printsec=frontcover#v=onepage&q&f=false) for more details
  //Can be disabled by removing the #include "fastAnalog.h" above
#ifdef sbi
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif
  mainLoopCount = 0;
  ignitionCount = 0;
  //Begin priming the fuel pump. This is turned off in the low resolution, 1s interrupt in timers.ino
  digitalWrite(pinFuelPump, HIGH);
  fuelPumpOn = true;
  //Perform the priming pulses. Set these to run at an arbitrary time in the future (100us). The prime pulse value is in ms*10, so need to multiple by 100 to get to uS
  //timer configs
  //timer 5 is for cam o/p
  TCCR5A = 0;  TCCR5B = 0;
  TCCR5B |= _BV(CS51); // set prescaler to 8
  TCCR5B |= _BV(ICES5);//rising edge trigger
  TIMSK5 |= _BV(ICIE5); //enable i/p capt interrupt
  TCCR3A = 0;  TCCR3B = 0;
  TCCR3B |= _BV(CS51); // set prescaler to 8
  //timer4 is for crank o/p
  TCCR4A = 0;// set timer counter to 0
  TCCR4B = 0;
  TCCR4B |= _BV(CS41); // set prescaler to 8
  TIMSK4 |= _BV(ICIE4);//enable i/p capt interrupt
  TCNT5 = 0; TCNT4 = 0; TCNT3 = 0;
  digitalWrite(pin_cranko, LOW);
  Serial.begin(115200);
  Serial3.begin(115200);

  //interrupt0
  EIMSK |= _BV(INT0);
  EICRA |= _BV(ISC00);
  EICRA |= _BV(ISC01);
  sei();
  toothOneMinusOneTime = 0;
  toothOneTime = 0;
}

void loop()
{
  mainLoopCount++;
  //Check for any requets from serial. Serial operations are checked under 2 scenarios:
  // 1) Every 64 loops (64 Is more than fast enough for TunerStudio). This function is equivalent to ((loopCount % 64) == 1) but is considerably faster due to not using the mod or division operations
  // 2) If the amount of data in the serial buffer is greater than a set threhold (See globals.h). This is to avoid serial buffer overflow when large amounts of data is being sent
  if ( ((mainLoopCount & 63) == 1) or (Serial.available() > SERIAL_BUFFER_THRESHOLD) )
  { if (Serial.available() > 0)
    {
      command();
    }
  }

  currentStatus.RPM = getRPM();
  // previousLoopTime = currentLoopTime;
  //currentLoopTime = micros();
  // unsigned long timeToLastTooth = (currentLoopTime - toothLastToothTime);
  if  ((currentStatus.RPM > 1000) && (currentStatus.RPM < 10000)) //Check how long ago the last tooth was seen compared to now. If it was more than half a second ago then the engine is probably stopped. toothLastToothTime can be greater than currentLoopTime if a pulse occurs between getting the lastest time and doing the comparison
  {
    int lastRPM = currentStatus.RPM; //Need to record this for rpmDOT calculation
    ignitionOn = true;
    if (syncstat == 0) {
      syncstat++;
    }
    else if (syncstat >= 3)    {
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK);
      BIT_SET(currentStatus.engine, BIT_ENGINE_RUN);
      currentStatus.hasSync = true;
    }

    //MAP Sampling system
    int tempReading;
    switch (configPage1.mapSample)
    {
      case 0:
        //Instantaneous MAP readings
        tempReading = analogRead(pinMAP);

        //Error checking
        if (tempReading >= VALID_MAP_MAX || tempReading <= VALID_MAP_MIN) {
          mapErrorCount += 1;
        }
        else {
          currentStatus.mapADC = tempReading;
          mapErrorCount = 0;
        }

        currentStatus.MAP = map(currentStatus.mapADC, 0, 1023, configPage1.mapMin, configPage1.mapMax); //Get the current MAP value
        break;

      case 1:
        //Average of a cycle
        if ( (MAPcurRev == startRevolutions) || (MAPcurRev == startRevolutions + 1) ) //2 revolutions are looked at for 4 stroke. 2 stroke not currently catered for.
        {
          tempReading = analogRead(pinMAP);

          //Error check
          if (tempReading < VALID_MAP_MAX && tempReading > VALID_MAP_MIN)
          {
            MAPrunningValue = MAPrunningValue + tempReading; //Add the current reading onto the total
            MAPcount++;
          }
          else {
            mapErrorCount += 1;
          }
        }
        else
        {
          //Reaching here means that the last cylce has completed and the MAP value should be calculated
          currentStatus.mapADC = ldiv(MAPrunningValue, MAPcount).quot;
          currentStatus.MAP = map(currentStatus.mapADC, 0, 1023, configPage1.mapMin, configPage1.mapMax); //Get the current MAP value
          MAPcurRev = startRevolutions; //Reset the current rev count
          MAPrunningValue = 0;
          MAPcount = 0;
        }
        break;

      case 2:
        //Minimum reading in a cycle
        if ( (MAPcurRev == startRevolutions) || (MAPcurRev == startRevolutions + 1) ) //2 revolutions are looked at for 4 stroke. 2 stroke not currently catered for.
        {
          tempReading = analogRead(pinMAP);

          //Error check
          if (tempReading < VALID_MAP_MAX && tempReading > VALID_MAP_MIN)
          { if ( tempReading < MAPrunningValue) {
              MAPrunningValue = tempReading;  //Check whether the current reading is lower than the running minimum
            }
          }
          else {
            mapErrorCount += 1;
          }
        }
        else
        {
          //Reaching here means that the last cylce has completed and the MAP value should be calculated
          currentStatus.mapADC = MAPrunningValue;
          currentStatus.MAP = map(currentStatus.mapADC, 0, 1023, configPage1.mapMin, configPage1.mapMax); //Get the current MAP value
          MAPcurRev = startRevolutions; //Reset the current rev count
          MAPrunningValue = 1023; //Reset the latest value so the next reading will always be lower
        }
        break;
    }

    //TPS setting to be performed every 32 loops (any faster and it can upset the TPSdot sampling time)
    if ((mainLoopCount & 31) == 1)
    { currentStatus.TPSlast = currentStatus.TPS;
      currentStatus.TPSlast_time = currentStatus.TPS_time;
      currentStatus.tpsADC = fastMap1023toX(analogRead(pinTPS), 0, 1023, 0, 255); //Get the current raw TPS ADC value and map it into a byte
      //Check that the ADC values fall within the min and max ranges (Should always be the case, but noise can cause these to fluctuate outside the defined range).
      if (currentStatus.tpsADC < configPage1.tpsMin) {
        currentStatus.tpsADC = configPage1.tpsMin;
      }
      else if (currentStatus.tpsADC > configPage1.tpsMax) {
        currentStatus.tpsADC = configPage1.tpsMax;
      }
      currentStatus.TPS = map(currentStatus.tpsADC, configPage1.tpsMin, configPage1.tpsMax, 0, 100); //Take the raw TPS ADC value and convert it into a TPS% based on the calibrated values
      currentStatus.TPS_time = currentLoopTime;
    }

    //The IAT and CLT readings can be done less frequently. This still runs about 4 times per second
    if ((mainLoopCount & 255) == 1)
    { currentStatus.cltADC = fastMap1023toX(analogRead(pinCLT), 0, 1023, 0, 511); //Get the current raw CLT value
      currentStatus.iatADC = map(analogRead(pinIAT), 0, 1023, 0, 511); //Get the current raw IAT value
      currentStatus.O2ADC = map(analogRead(pinO2), 0, 1023, 0, 511); //Get the current O2 value. Calibration is from AFR values 7.35 to 22.4. This is the correct calibration for an Innovate Wideband 0v - 5V unit. Proper calibration is still a WIP
      currentStatus.O2_2ADC = map(analogRead(pinO2_2), 0, 1023, 0, 511); //Get the current O2 value. Calibration is from AFR values 7.35 to 22.4. This is the correct calibration for an Innovate Wideband 0v - 5V unit. Proper calibration is still a WIP
      currentStatus.battery10 = fastMap1023toX(analogRead(pinBat), 0, 1023, 0, 245); //Get the current raw Battery value. Permissible values are from 0v to 24.5v (245)
      currentStatus.coolant = cltCalibrationTable[currentStatus.cltADC] - CALIBRATION_TEMPERATURE_OFFSET; //Temperature calibration values are stored as positive bytes. We subtract 40 from them to allow for negative temperatures
      currentStatus.IAT = iatCalibrationTable[currentStatus.iatADC] - CALIBRATION_TEMPERATURE_OFFSET;
      currentStatus.O2 = o2CalibrationTable[currentStatus.O2ADC];
      // currentStatus.O2_2 = o2CalibrationTable[currentStatus.O2_2ADC];
      //enable if required.
      vvtControl();
      boostControl(); //Most boost tends to run at about 30Hz, so placing it here ensures a new target time is fetched frequently enough
      fanControl();
    }

    //Always check for sync
    //Main loop runs within this clause


    //END SETTING STATUSES
    //-----------------------------------------------------------------------------------------------------

    //Begin the fuel calculation
    //Calculate an injector pulsewidth from the VE
    currentStatus.corrections = correctionsTotal();
    if (configPage1.algorithm == 0) //Check which fuelling algorithm is being used
    {
      //Speed Density
      currentStatus.VE = get3DTableValue(&fuelTable, currentStatus.MAP, currentStatus.RPM); //Perform lookup into fuel map for RPM vs MAP value
      currentStatus.advance = get3DTableValue(&ignitionTable, currentStatus.MAP, currentStatus.RPM); //As above, but for ignition advance
    }
    else
    {
      //Alpha-N
      currentStatus.VE = get3DTableValue(&fuelTable, currentStatus.TPS, currentStatus.RPM); //Perform lookup into fuel map for RPM vs TPS value
      currentStatus.advance = get3DTableValue(&ignitionTable, currentStatus.TPS, currentStatus.RPM); //As above, but for ignition advance
    }
    //target_degs =(uint16_t) currentStatus.advance - 5;
        if(syncstat>=3){
          target_degs = (uint16_t) currentStatus.advance - 5; 
          if(target_degs>25)//limiting to max values in case of improper inputs from table
          {target_degs=25;}
          if (target_degs < -25){target_degs= -25;}}
     target_pot = (uint8_t)(currentStatus.VE);
     pot_set(target_pot);

  }
  else 
  {
    //We reach here if the time between teeth is too great. This VERY likely means the engine has stopped
    currentStatus.RPM = 0;
    currentStatus.PW = 0;
    currentStatus.VE = 0;
    toothLastToothTime = 0;
    currentStatus.hasSync = false;
    currentStatus.runSecs = 0; //Reset the counter for number of seconds running.
    secCounter = 0; //Reset our seconds counter.
    startRevolutions = 0;
    MAPcurRev = 0;
    currentStatus.rpmDOT = 0;
    ignitionOn = false;
    fuelOn = false;
    fuelPumpOn = false;
//    syncstat = 0;
//    TIMSK3 &= ~_BV(OCIE3A);
//    TIMSK3 &= ~_BV(OCIE3B);
    target_degs = 0;

  }
  // target_degs = 1;

}



ISR(TIMER5_CAPT_vect)//rising trigger
{ crinteeth++;

  delta_t1 = (uint16_t)ICR5;

  if (syncstat >= 3)
  {
    if ((crinteeth == 12) || (crinteeth == 70))
    {      misteeth = 1;    }
    else {      misteeth = 0;     }


    if ((aL == 0) && (aL1 == 0))
    { croteeth++; OCR3A = ICR5 - 10;
      if ((croteeth != cromiss1) && (croteeth != cromiss2)) {
        if ((croteeth != cromiss3) && (croteeth != cromiss4)) // check for missing teeth on crANKO
        {          PORTD |= _BV(PD3);        }
      }

      if (misteeth == 1) {

        delta_t = delta_t1 - delta_t2;        c = delta_t1 + delta_t;
        OCR3A = (uint16_t)c;
        misteeth++;
      }
    }
    else if ((aL == 0) && (aL1 == 1))
    { croteeth++; iofH = 0;
      if ((croteeth != cromiss1) && (croteeth != cromiss2)) {
        if ((croteeth != cromiss3) && (croteeth != cromiss4)) // check for missing teeth on crANKO
        {
          PORTD |= _BV(PD3);
        }
      }
      if ((crinteeth != 13) && (crinteeth != 71)) {
        delta_t = delta_t1 - delta_t2;
      }
      per24thteethH = (((uint32_t)delta_t * (uint32_t)0xAAAB) >> 20) ; //fast div by 24
      c = (ICR5 + (20 * (per24thteethH))) ;
      if (iofH <= 0) {
        OCR3A = (uint16_t) c;
        iofH = 1;
      }
      else {
        OCR3A1 = (uint16_t) c;
        iofH = 2;
      }
    }
    else if (aL != 0)
    {
      if ((crinteeth != 13) && (crinteeth != 71)) {
        delta_t = delta_t1 - delta_t2;
      }
      per24thteethH = (((uint32_t)delta_t * (uint32_t)0xAAAB) >> 20) ; //fast div by 24
      c = (ICR5 + (aL * 4 * (per24thteethH))) ;
      if (iofH <= 0) {
        OCR3A = (uint16_t) c;
        iofH = 1;
      }
      else {
        OCR3A1 = (uint16_t) c;
        iofH = 2;
      }
    }
  }
  else {    PORTD |= _BV(PD3);  }

  delta_t2 = delta_t1;

  if (croteeth == 120) 
  {    croteeth = 0;  }
  if (croteeth == 1)
  {    PORTD |= _BV(PD2);  }
  else if (croteeth == camLteeth) //camo interrupt
  {    PORTD &= ~_BV(PD2);  }
}




ISR(TIMER4_CAPT_vect) //falling
{

  deltaL_t1 = (uint16_t)ICR4;

  if ((crinteeth == 12) || (crinteeth == 70))
  {
    misteeth1 = 1;
    //toothOneMinusOneTime = toothOneTime;
    //toothOneTime = micros();
  }
  else {    misteeth1 = 0;  }

  if (syncstat >= 3)
  {
    if ((aL == 0) && (aL1 == 0))
    { OCR3B = ICR4 - 10;
      if ((croteeth != cromiss1) && (croteeth != cromiss2)) {
        if ((croteeth != cromiss3) && (croteeth != cromiss4)) // check for missing teeth on crANKO
        {
          PORTD &= ~_BV(PD3);
        }
      }
      if (misteeth1 == 1) {

        deltaL_t = deltaL_t1 - deltaL_t2;        b = deltaL_t1 + deltaL_t;
        OCR3B = (uint16_t)b;
        misteeth1++;
      }
    }
    else if ((aL == 0) && (aL1  == 1)) {
      iofL = 0;
      if ((croteeth != cromiss1) && (croteeth != cromiss2)) {
        if ((croteeth != cromiss3) && (croteeth != cromiss4)) // check for missing teeth on crANKO
        {
          PORTD &= ~_BV(PD3);
        }
      }
      if ((crinteeth != 13) && (crinteeth != 71)) {
        deltaL_t = deltaL_t1 - deltaL_t2;
      }
      per24thteethL = (((uint32_t)deltaL_t * (uint32_t)0xAAAB) >> 20) ; //fast div by 24
      b = (ICR4 + (20 * (per24thteethL))) ; aL1 = 0;
      if (iofL <= 0)
      {
        OCR3B = (uint16_t) b;       iofL = 1;
      }
      else {
        OCR3B1 = (uint16_t) b;
        iofL = 2;
      }
    }
    else if (aL != 0 )
    { if ((crinteeth != 13) && (crinteeth != 71)) {
        deltaL_t = deltaL_t1 - deltaL_t2;
      }
      per24thteethL = (((uint32_t)deltaL_t * (uint32_t)0xAAAB) >> 20) ; //fast div by 24
      b = (ICR4 + (aL * 4 * (per24thteethL))) ;
      if (iofL <= 0)
      {
        OCR3B = (uint16_t) b;     iofL = 1;
      }
      else {
        OCR3B1 = (uint16_t) b;
        iofL = 2;
      }
    }
  }
  else
  {
    PORTD &= ~_BV(PD3);
    if ((PIND & (1 << PD0)) == 0)
    {
      PORTD &= ~_BV(PD2);
    }
  }

  // for direct high, calculates for next interrupt.
  if (cur_degsL < target_degs)
  { if (cur_degsL == -1 || cur_degsL == -7 || cur_degsL == -13 || cur_degsL == -19 || cur_degsL == -25 || cur_degsL == -31 || cur_degsL == 5 ||  cur_degsL == 11 ||  cur_degsL == 17 ||  cur_degsL == 23 ||  cur_degsL == 29 ) {
      OCR3B = ICR4 - 10;
      OCR3A = ICR5 - 10;
    }
    cur_degsL++;
    aL = (cur_degsL) % 6;
  }

  else if (cur_degsL > target_degs )
  { aL = (cur_degsL) % 6;
    if (aL != 0)     {
      cur_degsL--;
      aL = (cur_degsL) % 6;
    }
    else if (aL == 0)    {
      aL1 = 1;
      aL = 0;
      cur_degsL--;

    }
  }

  else if (cur_degsL == target_degs)
  {
    aL = (cur_degsL) % 6;
  }

  if (aL < 0) {
    aL = 6 + aL;
  }
  deltaL_t2 = deltaL_t1;
  aH = aL; aH1 = aL1; cur_degsH = cur_degsL;

}



ISR(TIMER3_COMPA_vect) //crankoH
{ croteeth++;
  if ((croteeth != cromiss1) && (croteeth != cromiss2 ))
  { if ((croteeth != cromiss3) && (croteeth != cromiss4)) // check for missing teeth
    {
      PORTD |= _BV(PD3);
    }
  }
  if ((misteeth == 1) || (misteeth == 2))  //crin missing teeth no advance interrupt
  {
    d = OCR3A + delta_t ;    OCR3A = (uint16_t) d ;    //TIMSK3 |= _BV(OCIE3A);
    misteeth++;
  }

  if (croteeth == 120)   {
    croteeth = 0;
  }
  if (croteeth == 1)  {
    PORTD |= _BV(PD2);
  }
  else if (croteeth == camLteeth)  {
    PORTD &= ~_BV(PD2);
  }
  if (iofH <= 1) {
    iofH = 0;
  }
  else if (iofH == 2) {
    OCR3A = OCR3A1;
    iofH = 1;
  }

}


ISR(TIMER3_COMPB_vect) //crankoL
{ PORTD &= ~_BV(PD3);
  if ((misteeth1 == 1) || (misteeth1 == 2)) //crin missing teeth no advance interrupt
  {
    e = OCR3B + deltaL_t ;    OCR3B = (uint16_t) e ;    
    misteeth1++;
  }
  if (iofL <= 1) {
    iofL = 0;
  }
  else if (iofL == 2) {
    OCR3B = OCR3B1;
    iofL = 1;
  }
}



ISR(INT0_vect)
{
  crinteeth = 1;
  if ((syncstat== 1) || (syncstat==2) )
  {
    syncstat++;
    PORTD |= _BV(PD2);//cranhko high
  }
  if (syncstat == 3) {
    syncstat++;
    TIFR3 |= _BV(OCF3A);
    TIFR3 |= _BV(OCF3B);
    TIMSK3 |= _BV(OCIE3A);
    TIMSK3 |= _BV(OCIE3B);
  }

}



                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
