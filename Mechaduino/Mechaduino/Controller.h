//Contains the TC5 Handler declaration


#ifndef __CONTROLLER_H__
#define  __CONTROLLER_H__

#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);


void TC5_Handler();

extern int
controller_add_point(
	float x,
	float v
);

extern int
controller_loop();

extern void
controller_clear();

#endif
