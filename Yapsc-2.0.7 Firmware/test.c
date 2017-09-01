//---------------------------------------------------------------------
//	File:		test.c
//
//	Written By:	Lawrence Glaister VE7IT
//
// Purpose: This set of routines deals with various test
//          routines that were used to verify parts of the
//          hardware and software design. They are not
//			needed in the running system.
// 
//---------------------------------------------------------------------
//
// Revision History
//
// Aug 8 2006 --    first version Lawrence Glaister
// 
//---------------------------------------------------------------------- 
#include "dspicservo.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#ifdef DEMOBOARD
	#warning "H/W is set for demo board!"
#endif

extern volatile unsigned short int timer_test;
extern volatile unsigned short int do_servo;
extern unsigned short int cmd_posn;			// current posn cmd from PC
extern unsigned short int cmd_err;			// number of bogus encoder positions detected
extern unsigned short int cmd_bits;			// a 4 bit number with old and new port values
extern volatile unsigned short int do_servo;
extern struct PID pid;

extern void set_pwm(float amplitude);
extern void calc_pid( void );

volatile struct TESTSTRUCT testS;	// structure which handle variables for testing and curves recording

#ifdef TESTING_FUNC
void test_pwm_interface( void )
{
	float max_amps = 100;
	float amps;
    float amp_incr = 5;
	unsigned short holdtime = 100;  // 1ms/current level

	printf("Testing current ramps via pwm and OPA549\r\n");
	timer_test = holdtime;
	while (1)
	{
		for ( amps = -max_amps; amps <= max_amps; amps += amp_incr )
		{
			set_pwm(amps);
			// 5ms at each torque value (current)
			while( timer_test );
			timer_test = holdtime;
		}
		for ( amps = max_amps; amps >= -max_amps; amps -= amp_incr )
		{
			set_pwm(amps);
			// 5ms at each torque value (current)
			while( timer_test );
			timer_test = holdtime;
		}
	}
}

// every second, echo cmded position to serial port
// (this tests the quadrature interface via the IC1 and IC2 pins)
// use emc2 to jog axis in each direction to monitor operation
void test_pc_interface( void )
{
	printf("Testing pc command interface on IC1 and IC2\r\n");
	timer_test = 10000;
	while (1)
	{
		// 1 second betwen prints
		while( timer_test );
		timer_test = 10000;
		printf("cmd = 0x%04X, err = 0x%04X, bits= 0x%02X\r\n",cmd_posn, cmd_err, cmd_bits);

		toggleErrLed();
	}
}

// every second, echo some pid status to serial port
void test_pid_interface( void )
{
	printf("Testing pid loop operation\r\n");
	timer_test = 10000;
	while (1)
	{
		if ( do_servo )
		{
			do_servo = 0;
			pid.command = (float)((short)cmd_posn);
			pid.feedback = (float)((short)POSCNT);
			calc_pid();
			set_pwm(fabs(pid.output));
		}

		if ( timer_test == 0)
		{
			timer_test = 10000;
			printf("cmd = %6.0f, feedback = %6.0f err = %6.0f, output = %6.0f\r\n",
				(double)pid.command, (double)pid.feedback, (double)pid.error, (double)pid.output);

			toggleErrLed();
		}
	}
}
#endif

/* adds delta (positive or negative) to the input command, and records the error periodically. */
void fire_curve()
{
	testS.ticks=0;					// to handle prediv
	testS.printCurve = 0;			// all samples not taken yet => not ready to print
	testS.sampleToPrintIndex = 0;	// the sampe which has to be printed
	testS.sampleIndex = 0;			// the current sample
	if(testS.delta>= pid.maxerror)
		testS.delta = pid.maxerror - 1;
	if(testS.delta<= -pid.maxerror)
		testS.delta = 1- pid.maxerror;
	testS.testInRun = 1;			// means a test is in course
}

void test_Handle_sampling(void)
{
	static int lasttestInRun = 0, ticks=0;
	if(testS.testInRun)
	{
		if(lasttestInRun == 0)
		{
			cmd_posn += testS.delta;
			lasttestInRun = 1;
			ticks = testS.prediv;
			return;
		}	
		if(ticks++ >= testS.prediv)
		{
			testS.samples[testS.sampleIndex++] = (int)pid.error;	// add an error sample. max +-32K
			testS.outputsamples[testS.sampleIndex] = (char)pid.output;
			if(testS.sampleIndex >= TEST_MAXSAMPLES)
			{
				testS.testInRun = 0;
				lasttestInRun = 0;
				testS.printCurve = 1;
			}
			ticks = 0;
		}
	}
}

void test_handle_print(void)
{
	if(testS.printCurve)
	{
		if(testS.sampleToPrintIndex == 0)
		{
			printf("\r\nBEGIN DATA 1 1 %i\r\n", TEST_MAXSAMPLES);
			printf("%i\r\n", 100*(testS.prediv+1));	// output sample period (µs)
			printf("%i %i\r\n", testS.sampleToPrintIndex, testS.samples[testS.sampleToPrintIndex]);
		}
		else
		{
			if(	testS.sampleToPrintIndex == TEST_MAXSAMPLES)
			{
				printf("END\r\n");
				printf("prediv was %i, step was %i\r\n", testS.prediv, testS.delta);
				testS.printCurve = 0;
			}
			else
			{
				//printf("%i %i\r\n", testS.sampleToPrintIndex, testS.samples[testS.sampleToPrintIndex]);
				printf("error: %i\r\n output: %i\r\n", testS.samples[testS.sampleToPrintIndex], testS.outputsamples[testS.sampleToPrintIndex]);
			}		
		}
		testS.sampleToPrintIndex++;
	}
}
