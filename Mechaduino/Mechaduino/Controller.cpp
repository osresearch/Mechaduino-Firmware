//Contains TC5 Controller definition
//The main control loop is executed by the TC5 timer interrupt:

#include <SPI.h>

#include "State.h"
#include "Utils.h"
#include "Parameters.h"


// gets called with FPID frequency
void TC5_Handler()
{
	//this is used by step response
	static int print_counter = 0;

	// Only run the loop if a counter overflow caused the interrupt
	if (TC5->COUNT16.INTFLAG.bit.OVF != 1)
		return;

	// Fast Write to Digital 3 for debugging
	TEST1_HIGH();
	// digitalWrite(3, HIGH);

	// read encoder and lookup corrected angle in calibration lookup table
	enc = readEncoder();
	y = lookup[enc];

	// Check if we've rotated more than a full revolution
	// (have we "wrapped" around from 359 degrees to 0 or from 0 to 359?)
	if ((y - y_1) < -180.0)
		wrap_count += 1;
	else
	if ((y - y_1) > 180.0)
		wrap_count -= 1;

	// yw is the wrapped angle (can exceed one revolution)
	yw = (y + (360.0 * wrap_count));

	// low pass filter the velocity measurement based on the wrapped
	// y position, since it will not have discontinuities
	v = vLPFa*v + vLPFb*(yw-yw_1);


	// choose control algorithm based on mode
	switch (mode) {
	case 'h': // hybrid control is still under development...
		hybridControl();
		break;

        case 'x': // position control
		e = (r - yw);
		ITerm += (pKi * e);

		// Integral wind up limit
		if (ITerm > +150.0)
			ITerm = +150.0;
		else
		if (ITerm < -150.0)
			ITerm = -150.0;

		// multiply the velocity by the D gain
		// this is a "slowing force", so it is negative
		DTerm = pLPFa*DTerm - pLPFb*pKd*(yw-yw_1);

		// The output is Kp * e + Ki * I - Kd * v
		u = pKp*e + ITerm + DTerm;

		break;

	case 'v': // velocity controller
		// error in degrees per rpm (sample frequency in
		// Hz * (60 seconds/min) / (360 degrees/rev) )
		e = (r - v);

		ITerm += (vKi * e);

		//Integral wind up limit
		if (ITerm > +200)
			ITerm = +200;
		else
		if (ITerm < -200)
			ITerm = -200;

		u = ((vKp * e) + ITerm - (vKd * (e-e_1)));

		//SerialUSB.println(e);
		break;

	case 't': // torque control
		u = 1.0 * r;
		break;

	default:
		u = 0;
		break;
	}

	// copy current value of y to previous value (y_1)
	// for next control cycle before PA angle added
	y_1 = y;

	// Depending on direction we want to apply torque,
	// add or subtract a phase angle of PA for max effective torque.
	// PA should be equal to one full step angle: if the excitation
	// angle is the same as the current position, we would not move!
	if (u > 0)
	{
		// You can experiment with "Phase Advance" by
		// increasing PA when operating at high speeds
		// update phase excitation angle and limit max current command
		y += PA;
		if (u > uMAX)
			u = uMAX;
	} else {
		y -= PA;
		if (u < -uMAX)
			u = -uMAX;
	}

	U = abs(u);

	// turn on LED if error is less than 0.1
	if (abs(e) < 0.1)
		ledPin_HIGH();
	else
		ledPin_LOW();


	// update phase currents; hybrid mode has already done this
	if (mode != 'h')
		output(-y, round(U));

	// these past values can be useful for more complex
	// controllers/filters.  Uncomment as necessary
	// e_3 = e_2;
	e_2 = e_1;
	e_1 = e;
	// u_3 = u_2;
	u_2 = u_1;
	u_1 = u;
	yw_1 = yw;
	//y_1 = y;

	if (print_yw == true && ++print_counter >= 5)
	{
		//*1024 allows us to print ints instead of floats... may be faster
		SerialUSB.println(int(yw*1024));
		print_counter = 0;
	}

	// writing a one clears the flag ovf flag
	TC5->COUNT16.INTFLAG.bit.OVF = 1;

	// for testing the control loop timing
	TEST1_LOW();
}
