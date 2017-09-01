//---------------------------------------------------------------------
//	File:		main.c
//
//	Written By:	Lawrence Glaister VE7IT
//
// Purpose: This program is used to control an single axis servo card
//			using a pic 30f2010 and a OPA549 power op am driver stage.
// 
// The following files should be included in the MPLAB project:
//
//		main.c		    -- Main source code file
//		capture.c		-- interface to pc quadrature cmd inputs using IC1 and IC2
//      timer1.c        -- timer 1 setup for 100us intervals
//      serial.c        -- interface to pc serial port for tuning - 9600n81
//      encoder.c       -- interface to quadature encoder
//		pwm.c			-- pwn ch for motor current control
//		pid.c			-- actual code for pid loop
//		save-res.c		-- routines to read/write configuration
//		p30f4012.gld	-- Linker script file
//		DataEEPROM.s	-- assembler file for read/write eeprom
//---------------------------------------------------------------------
//
// Revision History
//
// July 4 2006 -- first version 
// Aug 13 2006 -- fixed wrap around problem in servo loop
//             -- when servo is reenabled, the target posn is set to the current posn
//             -- if drive faults, it now stays faulted until pwr cycle
//				  or disable/reenable
// Aug 15 2006 -- added pulse multiplier option for high count motor encoders
// Sept 23 2006-  servo loop from 1ms to 500us (2khz rate)
// 
//---------------------------------------------------------------------- 
#include <p30f4012.h>
#include <stdio.h>
#include "dspicservo.h"
#include <math.h>

#ifdef DEMOBOARD
	#warning "H/W is set for demo board!"
#endif

//--------------------------Device Configuration------------------------       
// pll16 needs a 30mhz part
//_FOSC( FRC_PLL16 );  // 6mhz * PLL16  / 4 = 24mips
//_FWDT(WDT_OFF);                   // wdt off
/* enablebrownout @4.2v, 64ms powerup delay, MCLR pin active */
/* pwm pins in use, both active low to give 64ms delay on powerup */
//_FBORPOR(PBOR_ON & BORV_42 & PWRT_64 & MCLR_EN & RST_PWMPIN & PWMxL_ACT_LO );
//_FBORPOR(MCLR_EN & PBOR_ON & BORV_42 & RST_PWMPIN & PWMxH_ACT_HI & PWMxL_ACT_HI);
//_FBORPOR(MCLR_EN & PWMxH_ACT_HI & PWMxL_ACT_HI);
//----------------------------------------------------------------------
extern void setup_TMR1(void);
extern void setup_encoder(void);
extern void setup_uart(void);

extern volatile unsigned short int timer_test;
extern volatile unsigned short int timer_test2;
extern volatile unsigned short int cmd_posn;			// current posn cmd from PC
extern volatile short int rxrdy;

extern void setup_pwm(void);
extern void set_pwm(float amps);
extern void setup_adc10(void);
extern void setup_capture(void);
extern int restore_setup( void );
extern int calc_cksum(int sizew, int *adr);
extern void print_tuning( void );
extern volatile unsigned char error_flags;

extern void test_pc_interface( void );
extern void test_pid_interface( void );
extern void test_pwm_interface( void );
					
extern struct PID _EEDATA(32) pidEE;
extern volatile unsigned short int do_servo;
extern struct PID pid;
extern void init_pid(void);
extern void	process_serial_buffer();

extern void init_pid();
extern void clear_EE();
extern void save_setup();
extern void test_handle_print();

	// vars used for detection of incremental motion
unsigned short new_cmd,last_cmd, new_fb,last_fb;


void __attribute__((__no_auto_psv__, __interrupt__)) _StackError (void)
{
	PDC2 = PDC3 = PDC1 = 0;	// turn OFF the output (0V)
	clearOUT1();			// turn OFF (high impedance) OUT1
	T1CONbits.TON = 0;		// turn OFF timer 1 (and PID loop)
	while (1)
	{
		
		timer_test = 31;
		while ( timer_test-- )
		{
			timer_test2 = 16000;
			while(timer_test2--)setErrLed();
		}	
		
		timer_test = 98;
		while ( timer_test-- )
		{
			timer_test2 = 16000;
			while(timer_test2--)clearErrLed();
		}	
	}
}

void __attribute__((__no_auto_psv__, __interrupt__)) _AddressError (void)
{
	PDC2 = PDC3 = PDC1 = 0;
	clearOUT1();			// turn OFF (high impedance) OUT1
	T1CONbits.TON = 0;		// turn OFF timer 1 (and PID loop)
	while (1)
	{
		timer_test = 125;
		while ( timer_test-- )
		{
			timer_test2 = 16000;
			while(timer_test2--)setErrLed();
		}	
		
		timer_test = 125;
		while ( timer_test-- )
		{
			timer_test2 = 16000;
			while(timer_test2--)clearErrLed();
		}	
	}
}

void __attribute__((__no_auto_psv__, __interrupt__)) _MathError (void)
{
	PDC2 = PDC3 = PDC1 = 0;
	clearOUT1();			// turn OFF (high impedance) OUT1
	T1CONbits.TON = 0;		// turn OFF timer 1 (and PID loop)
	while (1)
	{
		
		timer_test = 63;
		while ( timer_test-- )
		{
			timer_test2 = 16000;
			while(timer_test2--)setErrLed();
		}	
		
		timer_test = 63;
		while ( timer_test-- )
		{
			timer_test2 = 16000;
			while(timer_test2--)clearErrLed();
		}	
	}
}

/*********************************************************************
  Function:        void set-io(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        Sets up io ports for in and out

  Note:            Some periperal configs may change them again
********************************************************************/
void setup_io(void)
{
	// PORT B
    ADPCFG = 0xffff;        // make sure analog doesnt grab encoder pins (all digital)
							// 0=output, 1=input
	_TRISB0 = 1;			// IN1
	_TRISB1 = 1;			// ENABLE
	_TRISB2 = 0;			// Error LED
	_TRISB3 = 1;			// used by quad encoder input ch INDX
	_CN5PUE = 1;
	_TRISB4 = 1;			// used by quad encoder input ch A
	_CN6PUE = 1;
	_TRISB5 = 1;			// used by quad encoder input ch B
	_CN7PUE = 1;
	
	// PORT C				// 0=output, 1=input
	_TRISC13 = 0;			// serial port aux  tx data
	_TRISC14 = 1;			// serial port aux  rx data
	_TRISC15 = 0;			// spare (maybe future xtal)

	// PORT D				// 0=output, 1=input
	_TRISD0 = 1;			// STEP
	_TRISD1 = 1;			// DIR

	// PORT E				// 0=output, 1=input
	_TRISE0 = 1;			// 
	_TRISE1 = 1;			// 
	_TRISE2 = 1;			// 
	_TRISE3 = 1;			// PWM out, input for start-up
	_TRISE4 = 1;			// 
	_TRISE5 = 1;			// PWM out, input for start-up
	_TRISE8 = 1;			// Current limit input pin : FLTA

	// PORT F				// 0=output, 1=input
	_TRISF2 = 1;			// PGC used by icsp
	_TRISF3 = 1;			// PGD used by icsp
}

/*********************************************************************
  Function:        int main(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        main function of the application. Peripherals are 
                   initialized.

  Note:            None.
********************************************************************/
int main(void) 
{
	int cs;
	/*int i;
	unsigned char *ptr = (char*)&pid;*/
	


	setup_io();			// make all i/o pins go the right dir
	setErrLed();		// led on

	setup_uart();		// setup the serial interface to the PC

    setup_TMR1();       // set up 1ms timer
	IEC0bits.T1IE = 1;  // Enable interrupts for timer 1
   						// needed for delays in following routines

	// 1/2 seconds startup delay 
	timer_test = 5000;		
	while ( timer_test );

	printf("\r\nPowerup\r\ni/o,uart,timer OK\r\n");	

	setup_pwm();		// start analog output
	set_pwm(0.0); 
	printf("pwm OK\r\n");

	init_pid();
	printf("pid OK\r\n");

    setup_encoder();    // 16 bit quadrature encoder module setup
	printf("encoder OK\r\n");

    setup_capture();    // 2 pins with quadrature cmd from PC
	printf("capture OK\r\n");

	// some junk for the serial channel
	printf(YAPSC_GREETING);
	printf(YAPSC_VERSION);

	clearErrLed();		// led off when init finished

	// restore config from eeprom
	// Read array named "setupEE" from DataEEPROM and place 
	// the result into array in RAM named, "setup" 
	restore_setup();
	cs = calc_cksum(sizeof(pid)/sizeof(int),(int*)&pid);
//	printf("checksums : %i : %i \?r\n", cs, pid.cksum);
/*	for(i = 0; i < sizeof(pid); i++)
		printf("[%u]", ptr[i]);
	printf("\r\n");*/
	
	if ( cs != pid.cksum)
	//if(0)
	{
		// opps, no valid setup detected
		// assume we are starting from a new box
		printf("No valid setup found in EEPROM, using defaults.\r\n");
		init_pid();
		clear_EE();
		save_setup();
	}
	else
	{
		printf("Using setup from eeprom.. ? for help\r\n");
		
		pid.error =0;	/* to avoid some bad bugs */
		pid.output = 0;
		pid.command = 0;
		pid.feedback = 0;
		print_tuning();
	}
//    printf("using %fms servo loop interval\r\n",pid.ticksperservo * 0.1);

//	BLOCK OF TEST ROUTINES FOR HARDWARE DEBUGGING		
//	test_pwm_interface();		// play with opa549 hardware
//	test_pc_interface();		// echo cmded posn to serial port
//  test_pid_interface();		// test pid loop operation

	new_cmd = last_cmd = new_fb = last_fb = 0;
	set_pwm( 0.0 );
	// make sure we dont move on enabling
	cmd_posn = POSCNT;		// make 16bit incr registers match
	pid.feedback = pid.command = 0L;		// make 32 bit counter match
	new_cmd = last_cmd = new_fb = last_fb = 0;
	pid.error_i = 0.0;		// reset integrator
	pid.enable = 0;
	clearOUT1();

	while (1)
	{

		// look for serial cmds
		// doing this while svo is enabled will cause bumps in servo loop
		// because of serial i/o time ( unless we get smart and move svo loop
		// into an isr )
		if ( rxrdy )
			process_serial_buffer();
		test_handle_print();
		
		if(error_flags & 0b00000001)	// "maxerror exceeded" error
		{
			if(!timer_test)
			{
				toggleErrLed();		
				timer_test = 1000;	// toggle the ERR led 10 times a second (5Hz)
			}
			if(!timer_test2)
			{
				printf("MAXERROR exceeded!\r\n");
				timer_test2 = 20000;	// 2s delay between messages
			}
		}
		else
			clearErrLed();
	}
	// to keep compiler happy....
	return 0;
}
