#include <SPI.h>
#include <Wire.h>

#include "Parameters.h"
#include "Controller.h"
#include "Utils.h"
#include "State.h"


static int gcode_g0(const char * args[], const int count)
{
	// find the X and F arguments
	float x = 0;
	float f = 0;

	//SerialUSB.println(count);

	for(int i = 0 ; i < count ; i++)
	{
		//SerialUSB.print("arg=");
		//SerialUSB.println(args[i]);

		if (args[i][0] == 'X')
			x = atof(&args[i][1]);
		else
		if (args[i][0] == 'F')
			f = atof(&args[i][1]);
		else
			return -1;
	}

	return controller_add_point(x, f);
}


static int gcode_m114(const char * args[], const int count)
{
	char buf[256];
	snprintf(buf, sizeof(buf),
		"X:%.3f F:%.3f E:%u M:%c",
		yw, // should be scaled from deg to mm
		v, // should also be scaled deg/s to mm
		enc, // raw value
		mode
	);
	SerialUSB.println(buf);
	return 0;
}


static int gcode_m0(const char ** argc, const int count)
{
	mode = ' ';
	controller_clear();
	SerialUSB.println("STOP");
	return 0;
}

static int gcode_m17(const char ** argc, const int count)
{
	controller_clear();
	mode = 'x';
	SerialUSB.println("START");
	return 0;
}


int gcode_line(char * line)
{
	char * cmd = strtok(line, " \t");
	const char * args[16];
	int count = 0;
	while(count < 16 && (args[count++] = strtok(NULL, " \t")) != NULL)
		;
	count--;

	if (strcmp(cmd, "G0") == 0
	||  strcmp(cmd, "G1") == 0)
		return gcode_g0(args, count);

	if (strcmp(cmd, "M0") == 0
	||  strcmp(cmd, "M18") == 0)
		return gcode_m0(args, count);

	if (strcmp(cmd, "M17") == 0)
		return gcode_m17(args, count);

	if (strcmp(cmd, "M114") == 0)
		return gcode_m114(args, count);

	return -1;
}


// Monitors serial for commands.
// Must be called in routinely in loop for serial interface to work.
// Processes a line at a time
void serialCheck()
{
	static char line[256];
	static unsigned len;

	if (!SerialUSB.available())
		return;

	const char c = (char) SerialUSB.read();

	// strip any line feeds
	if (c == '\r')
		return;

	if (len == sizeof(line))
	{
		SerialUSB.println("ERROR: gcode buffer overflow");
		len = 0;
		return;
	}

	if (c != '\n')
	{
		line[len++] = c;
		return;
	}

	line[len] = '\0';

	// echo the line to the serial port
	SerialUSB.println(line);

	if (gcode_line(line) < 0)
		SerialUSB.println("ERROR: gcode failed");

	len = 0;
}

#if 0

    switch (inChar) {

      case 'G': // go position and velocity
        desired_pos = SerialUSB.parseFloat();
	break;
      case 'V': // set velocity
        desired_vel = SerialUSB.parseFloat();
	if (desired_vel < 0.1)
		desired_vel = 0.1;

	break;
	
      case 'p':             //print
        print_angle();
        break;

      case 's':             //step
        oneStep();
        print_angle();
        break;

      case 'd':             //dir
        if (dir) {
          dir = false;
        }
        else {
          dir = true;
        }
        break;

      case 'w':                //old command
        calibrate();           //cal routine
        break;
        
      case 'c':
        calibrate();           //cal routine
        break;        

      case 'e':
        readEncoderDiagnostics();   //encoder error?
        break;

      case 'y':
        r = read_angle();          // hold the current position
        SerialUSB.print("New setpoint ");
        SerialUSB.println(r, 2);
        enableTCInterrupts();      //enable closed loop
        break;

      case 'n':
        disableTCInterrupts();      //disable closed loop
        break;

      case 'r':             //new setpoint
        SerialUSB.println("Enter setpoint:");
        while (SerialUSB.available() == 0)  {}
        r = SerialUSB.parseFloat();
        SerialUSB.println(r);
        break;

      case 'x':
        mode = 'x';           //position loop
        break;

      case 'v':
        mode = 'v';           //velocity loop
        break;

      case 't':
        mode = 't';           //torque loop
        break;

      case 'h':               //hybrid mode
        mode = 'h';
        break;

      case 'q':
        parameterQuery();     // prints copy-able parameters
        break;

      case 'a':             //anticogging
        antiCoggingCal();
        break;

      case 'k':
        parameterEditmain();
        break;
        
      case 'g':
        sineGen();
        break;

      case 'm':
        serialMenu();
        break;
        
      case 'j':
        stepResponse();
        break;


      default:
        break;
    }
}


void parameterQuery() {         //print current parameters in a format that can be copied directly in to Parameters.cpp
  SerialUSB.println(' ');
  SerialUSB.println("----Current Parameters-----");
  SerialUSB.println(' ');
  SerialUSB.println(' ');

  SerialUSB.print("volatile float Fs = ");
  SerialUSB.print(Fs, DEC);
  SerialUSB.println(";  //Sample frequency in Hz");
  SerialUSB.println(' ');

  SerialUSB.print("volatile float pKp = ");
  SerialUSB.print(pKp, DEC);
  SerialUSB.println(";      //position mode PID vallues.");
  
  SerialUSB.print("volatile float pKi = ");
  SerialUSB.print(pKi, DEC);
  SerialUSB.println(";");

  SerialUSB.print("volatile float pKd = ");
  SerialUSB.print(pKd, DEC);
  SerialUSB.println(";");
  
  SerialUSB.print("volatile float pLPF = ");
  SerialUSB.print(pLPF, DEC);
  SerialUSB.println(";");

  SerialUSB.println(' ');

  SerialUSB.print("volatile float vKp = ");
  SerialUSB.print(vKp, DEC);
  SerialUSB.println(";      //velocity mode PID vallues.");

  SerialUSB.print("volatile float vKi = ");
  SerialUSB.print(vKi , DEC);
  SerialUSB.println(";");
 // SerialUSB.println(vKi * Fs, DEC);
 // SerialUSB.println(" / Fs;");

  SerialUSB.print("volatile float vKd = ");
  SerialUSB.print(vKd, DEC);
  SerialUSB.println(";");
 // SerialUSB.print(vKd / Fs);
 // SerialUSB.println(" * FS;");
  SerialUSB.print("volatile float vLPF = ");
  SerialUSB.print(vLPF, DEC);
  SerialUSB.println(";");

  SerialUSB.println("");
  SerialUSB.println("//This is the encoder lookup table (created by calibration routine)");
  SerialUSB.println("");
  
  SerialUSB.println("const float lookup[] = {");
  for (int i = 0; i < 16384; i++) {
    SerialUSB.print(lookup[i]);
    SerialUSB.print(", ");
  }
  SerialUSB.println("");
  SerialUSB.println("};");

}



void oneStep() {           /////////////////////////////////   oneStep    ///////////////////////////////
  
  if (!dir) {
    stepNumber += 1;
  }
  else {
    stepNumber -= 1;
  }

  //output(1.8 * stepNumber, 64); //updata 1.8 to aps..., second number is control effort
  output(aps * stepNumber, (int)(0.33 * uMAX));
  delay(10);
}
#endif
