/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
This file contains the various crank and cam wheel decoder functions.


Each decoder must have the following 4 functions (Where xxxx is the decoder name):

* triggerSetup_xxx - Called once from within setup() and configures any required variables
* triggerPri_xxxx - Called each time the primary (No. 1) crank/cam signal is triggered (Called as an interrupt, so variables must be declared volatile)
* triggerSec_xxxx - Called each time the secondary (No. 2) crank/cam signal is triggered (Called as an interrupt, so variables must be declared volatile)
* getRPM_xxxx - Returns the current RPM, as calculated by the decoder
* getCrankAngle_xxxx - Returns the current crank angle, as calculated b the decoder

And each decoder must utlise at least the following variables:
toothLastToothTime - The time (In uS) that the last primary tooth was 'seen'
* 

*/

inline void addToothLogEntry(unsigned long time)
{
  //High speed tooth logging history
  toothHistory[toothHistoryIndex] = time;
  if(toothHistoryIndex == (TOOTH_LOG_BUFFER-1))
  { toothHistoryIndex = 0; BIT_CLEAR(currentStatus.squirt, BIT_SQUIRT_TOOTHLOG1READY); } //The tooth log ready bit is cleared to ensure that we only get a set of concurrent values. 
  else
  { toothHistoryIndex++; }
}

/*
As nearly all the decoders use a common method of determining RPM (The time the last full revolution took)
A common function is simpler
*/
inline int stdGetRPM()
{
  //noInterrupts();
  revolutionTime = ((toothOneTime - toothOneMinusOneTime)); //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
  //interrupts(); 
  return (US_IN_MINUTE / revolutionTime); //Calc RPM based on last full revolution time (Faster as /)
}

/*
This is a special case of RPM measure that is based on the time between the last 2 teeth rather than the time of the last full revolution
This gives much more volatile reading, but is quite useful during cranking, particularly on low resolution patterns.
It can only be used on patterns where the teeth are evently spaced
It takes an argument of the full (COMPLETE) number of teeth per revolution. For a missing tooth wheel, this is the number if the tooth had NOT been missing (Eg 36-1 = 36)
*/
inline int crankingGetRPM(byte totalTeeth)
{
  noInterrupts();
  revolutionTime = (toothLastToothTime - toothLastMinusOneToothTime) * totalTeeth;
  interrupts(); 
  return (US_IN_MINUTE / revolutionTime);
}

/* 
Name: Missing tooth wheel
Desc: A multi-tooth wheel with one of more 'missing' teeth. The first tooth after the missing one is considered number 1 and isthe basis for the trigger angle
Note: This does not currently support dual wheel (ie missing tooth + single tooth on cam)
*/
void triggerSetup_missingTooth()
{
  triggerToothAngle = 360 / configPage2.triggerTeeth; //The number of degrees that passes from tooth to tooth
  triggerActualTeeth = configPage2.triggerTeeth - configPage2.triggerMissingTeeth; //The number of physical teeth on the wheel. Doing this here saves us a calculation each time in the interrupt
  triggerFilterTime = (int)(1000000 / (MAX_RPM / 60 * configPage2.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be disgarded as noise
  secondDerivEnabled = false;
}

void triggerPri_missingTooth()
{
   // http://www.msextra.com/forums/viewtopic.php?f=94&t=22976
   // http://www.megamanual.com/ms2/wheel.htm

  
   //else { targetGap = (10 * (toothLastToothTime - toothLastMinusOneToothTime)) >> 2; } //Multiply by 2.5 (Checks for a gap 2.5x greater than the last one)

   
   toothLastMinusOneToothTime = toothLastToothTime;
   toothLastToothTime = curTime;
}

void triggerSec_missingTooth(){ return; } //This function currently is not used

int getRPM_missingTooth()
{
   return stdGetRPM();
}




