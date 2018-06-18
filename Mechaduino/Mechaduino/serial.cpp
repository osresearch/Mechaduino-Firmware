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


static int gcode_g92(const char * args[], const int count)
{
	// find the X, if there is one specified
	float x = 0;

	//SerialUSB.println(count);

	for(int i = 0 ; i < count ; i++)
	{
		//SerialUSB.print("arg=");
		//SerialUSB.println(args[i]);

		if (args[i][0] == 'X')
			x = atof(&args[i][1]);
		else
			return -1;
	}

	controller_set_position(x);
	return 0;
}


static int gcode_m114(const char * args[], const int count)
{
	SerialUSB.print("X:");
	SerialUSB.print(yw, 3);
	SerialUSB.print(" F:");
	SerialUSB.print(v, 3);
	SerialUSB.print(" E:");
	SerialUSB.print(enc);
	SerialUSB.print(" M:");
	SerialUSB.print(mode == ' ' ? '0' : mode);
	SerialUSB.println();
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


static int gcode_calibrate(const char ** argc, const int count)
{
	calibrate();
	controller_clear();
	return 0;
}

static int gcode_help(const char ** argc, const int count);

static const struct {
	const char * const cmd;
	int (*handler)(const char ** argc, const int count);
	const char * const help;
} gcode_handlers[] = {
	{ "G0", gcode_g0, "Move rapid" },
	{ "G1", gcode_g0, "Move linear" },
	{ "M0", gcode_m0, "Emergency stop" },
	{ "M17", gcode_m17, "Motors on" },
	{ "M18", gcode_m0, "Motors off" },
	{ "M114", gcode_m114, "Report position" },
	{ "G92", gcode_g92, "Set position" },
	{ "c", gcode_calibrate, "Calibrate encoder" },
	{ "?", gcode_help, "Help" },
};

static const unsigned num_handlers
	= sizeof(gcode_handlers)/sizeof(*gcode_handlers);


int gcode_line(char * line)
{
	char * cmd = strtok(line, " \t");
	const char * args[16];
	int count = 0;
	while(count < 16 && (args[count++] = strtok(NULL, " \t")) != NULL)
		;
	count--;

	for(unsigned i = 0 ; i < num_handlers; i++)
	{
		if (strcmp(cmd, gcode_handlers[i].cmd) != 0)
			continue;
		return gcode_handlers[i].handler(args, count);
	}

	SerialUSB.print("UKNOWN GCODE: ");
	SerialUSB.println(cmd);
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

void serialMenu() {
	SerialUSB.println("----- Mechaduino 0.X -----");

	SerialUSB.print("Firmware: ");
	SerialUSB.println(firmware_version);

	SerialUSB.print("Identifier: ");
	SerialUSB.println(identifier);

	SerialUSB.print("GCODES:");
	for(unsigned i = 0 ; i < num_handlers ; i++)
	{
		SerialUSB.print(" ");
		SerialUSB.print(gcode_handlers[i].cmd);
	}

	SerialUSB.println("");
}

static int gcode_help(const char ** argc, const int count)
{
	SerialUSB.println("Supported gcodes:");
	for(unsigned i = 0 ; i < num_handlers ; i++)
	{
		SerialUSB.print(gcode_handlers[i].cmd);
		SerialUSB.print(": ");
		SerialUSB.println(gcode_handlers[i].help);
	}
}

