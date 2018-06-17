/** \file
 * Calibration routine to generate the lookup table.
 */

#include <FlashStorage.h>
#include "Parameters.h"
#include "Controller.h"
#include "Utils.h"
#include "State.h"
#include "analogFastWrite.h"

// This is the encoder lookup table (created by calibration routine):
// There are 16384 entries and it will be filled by the flasher.
// the default is a linear map so that things will work ok out of the
// box, but the calibration will ensure that the mapping is better
const float __attribute__((__aligned__(256))) lookup[16384] = {};

int lookup_valid = 0;


static FlashClass flash;
static const unsigned page_size = 256; // actual size is 64?
static unsigned page_count;
static const unsigned floats_per_page = page_size / sizeof(float);
static float page[floats_per_page];
static const void * page_ptr;

static void write_page()
{
  if (0 == (0xFFF & (uintptr_t) page_ptr))
  {
    SerialUSB.println();
    SerialUSB.print("0x");
    SerialUSB.print((uintptr_t) page_ptr, HEX);
  } else {
    SerialUSB.print(".");
  }

  flash.erase((const void*) page_ptr, sizeof(page));
  flash.write((const void*) page_ptr, (const void *) page, sizeof(page));
}

static void store_lookup(float lookupAngle)
{
  page[page_count++] = lookupAngle;
  if(page_count != floats_per_page)
    return;

  // we've filled an entire page, write it to the flash
  write_page();

  // reset our counters and increment our flash page
  page_ptr += sizeof(page);
  page_count = 0;
  memset(page, 0, sizeof(page));
}


/// this is the calibration routine
void calibrate()
{

  // can't use readEncoder while in closed loop
  disableTCInterrupts();

  int encoderReading = 0;     //or float?  not sure if we can average for more res?
  int currentencoderReading = 0;
  int lastencoderReading = 0;
  int avg = 10;               //how many readings to average

  int iStart = 0;     //encoder zero position index
  int jStart = 0;
  int stepNo = 0;
  
  int fullStepReadings[spr];
    
  int fullStep = 0;
  int ticks = 0;
  float lookupAngle = 0.0;
  SerialUSB.println("Beginning calibration routine...");

  encoderReading = readEncoder();
  dir = true;
  oneStep();
  delay(500);

  if ((readEncoder() - encoderReading) < 0)   //check which way motor moves when dir = true
  {
    SerialUSB.println("Wired backwards");    // rewiring either phase should fix this.  You may get a false message if you happen to be near the point where the encoder rolls over...
    return;
  }

  while (stepNumber != 0) {       //go to step zero
    if (stepNumber > 0) {
      dir = true;
    }
    else
    {
      dir = false;
    }
    oneStep();
    delay(100);
  }
  dir = true;
  for (int x = 0; x < spr; x++) {     //step through all full step positions, recording their encoder readings

    encoderReading = 0;
    delay(20);                         //moving too fast may not give accurate readings.  Motor needs time to settle after each step.
    lastencoderReading = readEncoder();
        
    for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
      currentencoderReading = readEncoder();

      if ((currentencoderReading-lastencoderReading)<(-(cpr/2))){
        currentencoderReading += cpr;
      }
      else if ((currentencoderReading-lastencoderReading)>((cpr/2))){
        currentencoderReading -= cpr;
      }
 
      encoderReading += currentencoderReading;
      delay(10);
      lastencoderReading = currentencoderReading;
    }
    encoderReading = encoderReading / avg;
    if (encoderReading>cpr){
      encoderReading-= cpr;
    }
    else if (encoderReading<0){
      encoderReading+= cpr;
    }

    fullStepReadings[x] = encoderReading;
   // SerialUSB.println(fullStepReadings[x], DEC);      //print readings as a sanity check
    if (x % 20 == 0)
    {
      SerialUSB.println();
      SerialUSB.print(100*x/spr);
      SerialUSB.print("% ");
    } else {
      SerialUSB.print('.');
    }
    
    oneStep();
  }
      SerialUSB.println();

 // SerialUSB.println(" ");
 // SerialUSB.println("ticks:");                        //"ticks" represents the number of encoder counts between successive steps... these should be around 82 for a 1.8 degree stepper
 // SerialUSB.println(" ");
  for (int i = 0; i < spr; i++) {
    ticks = fullStepReadings[mod((i + 1), spr)] - fullStepReadings[mod((i), spr)];
    if (ticks < -15000) {
      ticks += cpr;

    }
    else if (ticks > 15000) {
      ticks -= cpr;
    }
   // SerialUSB.println(ticks);

    if (ticks > 1) {                                    //note starting point with iStart,jStart
      for (int j = 0; j < ticks; j++) {
        stepNo = (mod(fullStepReadings[i] + j, cpr));
        // SerialUSB.println(stepNo);
        if (stepNo == 0) {
          iStart = i;
          jStart = j;
        }

      }
    }

    if (ticks < 1) {                                    //note starting point with iStart,jStart
      for (int j = -ticks; j > 0; j--) {
        stepNo = (mod(fullStepReadings[spr - 1 - i] + j, cpr));
        // SerialUSB.println(stepNo);
        if (stepNo == 0) {
          iStart = i;
          jStart = j;
        }

      }
    }

  }

  // The code below generates the lookup table by intepolating between
  // full steps and mapping each encoder count to a calibrated angle
  // The lookup table is too big to store in volatile memory,
  // so we must generate and store it into the flash on the fly

  // begin the write to the calibration table
  page_count = 0;
  page_ptr = (const uint8_t*) lookup;
  SerialUSB.print("Writing to flash 0x");
  SerialUSB.print((uintptr_t) page_ptr, HEX);
  SerialUSB.print(" page size PSZ=");
  SerialUSB.print(NVMCTRL->PARAM.bit.PSZ);

  for (int i = iStart; i < (iStart + spr + 1); i++) {
    ticks = fullStepReadings[mod((i + 1), spr)] - fullStepReadings[mod((i), spr)];

    if (ticks < -15000) {           //check if current interval wraps over encoder's zero positon
      ticks += cpr;
    }
    else if (ticks > 15000) {
      ticks -= cpr;
    }
    //Here we print an interpolated angle corresponding to each encoder count (in order)
    if (ticks > 1) {              //if encoder counts were increasing during cal routine...

      if (i == iStart) { //this is an edge case
        for (int j = jStart; j < ticks; j++) {
	  store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0));
        }
      }

      else if (i == (iStart + spr)) { //this is an edge case
        for (int j = 0; j < jStart; j++) {
	  store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0));
        }
      }
      else {                        //this is the general case
        for (int j = 0; j < ticks; j++) {
	  store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0));
        }
      }
    }

    else if (ticks < 1) {             //similar to above... for case when encoder counts were decreasing during cal routine
      if (i == iStart) {
        for (int j = - ticks; j > (jStart); j--) {
          store_lookup(0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / float(ticks))), 360000.0));
        }
      }
      else if (i == iStart + spr) {
        for (int j = jStart; j > 0; j--) {
          store_lookup(0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / float(ticks))), 360000.0));
        }
      }
      else {
        for (int j = - ticks; j > 0; j--) {
          store_lookup(0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / float(ticks))), 360000.0));
        }
      }

    }


  }

  if (page_count != 0)
	write_page();

  SerialUSB.println(" ");

  // re-enable the interrupt, but in a non-functional mode
  mode = ' ';
  lookup_valid = 1;
  enableTCInterrupts();

  // wait for the interrupt to resume and populate the yw value
  // then enable hold mode
  delay(200);
  r = yw;
  mode = 'x';
}
