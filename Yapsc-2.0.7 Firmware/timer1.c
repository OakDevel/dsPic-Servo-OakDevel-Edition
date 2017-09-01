//---------------------------------------------------------------------
//	File:		timer1.c
//
//	Written By:	Lawrence Glaister VE7IT
//
// Purpose: routines to setup and use timer 1
//      
// 
//---------------------------------------------------------------------
//
// Revision History
//
// Nov 5 2005 -- first version 
//---------------------------------------------------------------------- 
#include <stdio.h>
#include <math.h>
#include "dspicservo.h"
#ifdef DEMOBOARD
	#warning "H/W is set for demo board!"
#endif

volatile unsigned short int timer_test;
volatile unsigned short int timer_test2;
volatile unsigned char error_flags;
// 0bxxxxxxxx
//   |||||||`-> maxerror fault active
//   ||||||`--> overcurrent fault has been active


extern struct PID pid;

extern void test_Handle_sampling();
extern void calc_pid( void );
extern void set_pwm(float output);
extern void checkDir();

extern unsigned short new_cmd,last_cmd, new_fb,last_fb;
extern volatile unsigned short int cmd_posn;			// current posn cmd from PC

/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _T1Interrupt (void)

  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        This isr is used to periodically do certain tasks
          		   Basic rate is set to 10000intr/sec in the init function

********************************************************************/

void __attribute__((__no_auto_psv__, __interrupt__)) _T1Interrupt (void)
{
	static short gear = 0, ticks;

	IFS0bits.T1IF = 0;

	if ( pid.ticksperservo < 2) pid.ticksperservo = 2;
	
	// this block of timers is used in the software for delays
    if ( timer_test > 0 ) --timer_test;
    if ( timer_test2 > 0 ) --timer_test2;
	
	if(ticks++ > 5000)
	{
	    if(FltPin && (error_flags & ERR_OVCUR))
	    {
	    	clearErrLed();	// Current limit ended
	    	error_flags -= ERR_OVCUR;	// remove the erro flag
	    }	
    	ticks = 0;
 	}
 	
 	
	if (++gear >= pid.ticksperservo)
	{
		checkDir();
		gear = 0;
		
		if (SVO_ENABLE)
		{				
			if ( pid.enable == 0 )	// last loop, servo was off
			{
			    set_pwm( 0.0 );
				printf("servo-enabled\r\n>");
				pid.enable = 1;
				// make sure we dont move on enabling
				cmd_posn = POSCNT;		// make 16bit incr registers match
				pid.command = 0L;		// make 32 bit counter match
				pid.feedback = 0L;
				// make the 1ms loop temps match
				new_cmd = last_cmd = new_fb = last_fb = 0;
				pid.error_i = 0.0;		// reset integrator
			}

		    new_cmd = cmd_posn;		// grab current cmd from pc
		    new_fb = POSCNT;		// grab current posn from encoder

		    pid.command  += (long int)((short)(new_cmd - last_cmd));
		    pid.feedback += (long int)((short)(new_fb  - last_fb ));
		    last_cmd = new_cmd;
		    last_fb = new_fb;

		    calc_pid();

		    // check for a drive fault ( posn error > allowed )
		    if (( pid.maxerror > 0.0 ) && 
			    ( fabs(pid.error) > pid.maxerror ))
		    {
			    set_pwm( 0.0 );
			    pid.error_i = 0.0;		// reset integrator as it may have wound up
				error_flags |= 0b00000001;
				clearOUT1();
		    }
		    else
		    {
			    set_pwm(pid.output);	// update motor drive
			    error_flags &= 0b11111110;
				setOUT1();
		    }
		}
		else
		{
			pid.enable = 0;		// disable pid
			set_pwm(0.0);		// null output is disabled
			clearOUT1();		// the external enable signal is cleared => amp disabled (10V)
		}
	}
 	test_Handle_sampling();
	return;
}


/*********************************************************************
  Function:        void setupTMR1(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        Initialization of timer 1 as a periodic interrupt 
                   each 0.1 ms (10khz)

  Note:            None.
********************************************************************/

void setup_TMR1(void)
{
	T1CON = 0x0020;			// internal Tcy/64 clock
	TMR1 = 0;
#define PR1PRESET  (FCY/64/10000)
	PR1 = PR1PRESET;		// 0.1 ms interrupts (100us)
	T1CONbits.TON = 1;		// turn on timer 1 
	_T1IP = 6;
}
