//---------------------------------------------------------------------
//	File:		capture.c
//
//	Written By:	Lawrence Glaister VE7IT
//
//  Modified by Maximilien Mousset
//
// The original code from Lawrence Glaister was made to captude 
// quadrature input from the PC. This was modified for STEP/DIR input
// with {RD0 : STEP; RD1 : DIR}
// increments/decrements command by the PC multiplier on each RD0's
// rising edge
//
//      
// 
//---------------------------------------------------------------------
//
// Revision History
//
// Aug 7 2006 --    first version Lawrence Glaister
// Aug 15 2006		added pc command pulse multiplier option
// 
//---------------------------------------------------------------------- 
#include "dspicservo.h"
#ifdef DEMOBOARD
	#warning "H/W is set for demo board!"
#endif

extern struct PID pid;

volatile unsigned short int cmd_posn;			// current posn cmd from PC
volatile unsigned short int cmd_err;			// number of bogus encoder positions detected
volatile unsigned short int cmd_bits;	// a 4 bit number with old and new port values

int stepIncr;		// how much the command pos is increased each pulse

/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _IC1Interrupt(void)

  PreCondition:    None.
 
  Input:           None

  Output:          None.

  Side Effects:    None.

  Overview:        handles changes on IC1 pin 

  Note:            None.
********************************************************************/
void __attribute__((__interrupt__, __no_auto_psv__, shadow)) _IC1Interrupt(void)
{
    IFS0bits.IC1IF = 0;                    	// Clear IF bit
 //   if(!STEP)
//    	return;
	cmd_posn = (int) cmd_posn + stepIncr;
}

/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _IC2Interrupt(void)

  PreCondition:    None.
 
  Input:           None

  Output:          None.

  Side Effects:    None.

  Overview:        handles changes on IC2 pin 

  Note:            None.
********************************************************************/
void __attribute__((__no_auto_psv__, __interrupt__)) _IC2Interrupt(void)
{
    IFS0bits.IC2IF = 0;                    /* Clear IF bit */
    checkDir();
}


/*********************************************************************
  Function:        void setup_capture(void)

  PreCondition:    None.
 
  Input:           None

  Output:          None.

  Side Effects:    None.

  Overview:        

  Note:            None.
********************************************************************/
void setup_capture(void)
{
	/* start with a clean slate in the control words */
	/* also disables IC module 1 and 2*/
	IC1CON = 0;						
	IC2CON = 0;

	/*	disable interrupts */
    IEC0bits.IC1IE = 0;
    IEC0bits.IC2IE = 0;

	/* Clean up any pending IF bits */
    IFS0bits.IC1IF = 0;             
	IFS0bits.IC2IF = 0;

	// assign Interrupt Priority to IPC Register  (7 is max, we want to be sure we get the ints)
    IPC0bits.IC1IP = 0x0007;     
    IPC1bits.IC2IP = 0x0006;

    // and Capture Mode (002=every falling edge)
    IC1CON = 0x0002;
    IC2CON = 0x0001;

	cmd_posn = 0;

	//	go live... enable interrupts
    IEC0bits.IC1IE = 1;
    IEC0bits.IC2IE = 1;
}

void checkDir()
{
    if(DIR)
	     stepIncr = pid.multiplier;
	else
	     stepIncr = -pid.multiplier;
}	     