
/*
  -------------------------------------------------------------
  Mechaduino 0.1 & 0.2 Firmware  v0.1.4
  SAM21D18 (Arduino Zero compatible), AS5047 encoder, A4954 driver

  All Mechaduino related materials are released under the
  Creative Commons Attribution Share-Alike 4.0 License
  https://creativecommons.org/licenses/by-sa/4.0/

  Many thanks to all contributors!
  --------------------------------------------------------------
  
  Controlled via a SerialUSB terminal at 115200 baud.

  Implemented serial commands are:

 s  -  step
 d  -  dir
 p  -  print [step number] , [encoder reading]

 c  -  calibration routine
 e  -  check encoder diagnositics
 q  -  parameter query

 x  -  position mode
 v  -  velocity mode
 t  -  torque mode

 y  -  enable control loop
 n  -  disable control loop
 r  -  enter new setpoint

 j  -  step response
 k  -  edit controller gains -- note, these edits are stored in volatile memory and will be reset if power is cycled
 g  -  generate sine commutation table
 m  -  print main menu


  ...see serialCheck() in Utils for more details

*/

#include "Utils.h"
#include "Parameters.h"
#include "Controller.h"
#include "State.h"
#include "analogFastWrite.h"

//////////////////////////////////////
/////////////////SETUP////////////////
//////////////////////////////////////


void setup()        // This code runs once at startup
{                         
   
  digitalWrite(ledPin,HIGH);        // turn LED on 
  setupPins();                      // configure pins
  setupTCInterrupts();              // configure controller interrupt

  SerialUSB.begin(115200);          
  delay(3000);                      // This delay seems to make it easier to establish a connection when the Mechaduino is configured to start in closed loop mode.  
  serialMenu();                     // Prints menu to serial monitor
  setupSPI();                       // Sets up SPI for communicating with encoder
  digitalWrite(ledPin,LOW);         // turn LED off 
  
  // Uncomment the below lines as needed for your application.
  // Leave commented for initial calibration and tuning.
  
  // Configures setpoint to be controlled by step/dir interface
  configureStepDir();

  // Active low, for use wath RAMPS 1.4 or similar
  //configureEnablePin();

  // Use pin 3 as a ground so that the we have convienent ground
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);


  // start the interrupt handler so that enc is filled in
  enableTCInterrupts();
  mode = ' ';

  // spot check some of the lookup table to decide if it has been filled in
  if (lookup[0] == 0 && lookup[128] == 0 && lookup[1024] == 0)
  {
    SerialUSB.println("WARNING: Lookup table is empty!");
    SerialUSB.println("Run calibration before enabling hold mode.");
    lookup_valid = 0;
  } else {
    SerialUSB.println("Using lookup table");
    lookup_valid = 1;
  }

    delay(200);
    controller_clear();
    mode = 'x'; // hold the current position

    SerialUSB.print("Initial setpoint ");
    SerialUSB.println(r, 2);
}
  

void report_status()
{
	const unsigned report_interval = 1000;
	static unsigned last_report;
	const unsigned now = millis();
	if (now - last_report < report_interval)
		return;

	last_report = now;

	SerialUSB.print(now);
	SerialUSB.print(" a=");
	SerialUSB.print(y, 2);
	SerialUSB.print(" A=");
	SerialUSB.print(yw, 2);
	SerialUSB.print(" r=");
	SerialUSB.print(r, 2);
	SerialUSB.print(" e=");
	SerialUSB.print(enc);
	SerialUSB.print(" v=");
	SerialUSB.print(v, 2);
	SerialUSB.print(" u=");
	SerialUSB.print(u, 0);
	SerialUSB.println();
}


void loop()
{
	//report_status();

	// gcode waypoint loop
	controller_loop();

	// check for new gcode commands
	serialCheck();
}
