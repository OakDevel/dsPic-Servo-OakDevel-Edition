//---------------------------------------------------------------------
//	File:		pwm.c
//
//	Written By:	Lawrence Glaister
//
// Mod by Maximilien MOUSSET alias MaX-MoD:
// Output : 2 PWM MOSFET bridge
//
// Purpose: This set of routines to run pwn output
//      
// 
//---------------------------------------------------------------------
//
// Revision History
//
// Nov 21 2005 --    first version Lawrence Glaister
// Sept 1 2009 --    Last modification for YAPSC, including step input 
//                   testing
//---------------------------------------------------------------------- 
#include "dspicservo.h"
#include <pwm.h>
#include <stdio.h>

extern struct PID pid;
extern volatile unsigned char error_flags;

/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _PWMInterrupt(void)

  PreCondition:    None.
 
  Input:           None

  Output:          None.

  Side Effects:    None.

  Overview:        handles pwm interrupts 

  Note:            None.
********************************************************************/
void __attribute__((__no_auto_psv__, __interrupt__)) _PWMInterrupt(void)
{
    IFS2bits.PWMIF =0;
}

void __attribute__((__no_auto_psv__, __interrupt__)) _FLTAInterrupt(void)
{
    IFS2bits.FLTAIF =0;
    error_flags |= ERR_OVCUR;	// overcurrent flag set
    setErrLed();
}

/*********************************************************************
  Function:        void setupPWM(void)

  PreCondition:    None.
 
  Input:           None

  Output:          None.

  Side Effects:    None.

  Overview:        port setup with deadtime, 
                   fault input disables pwm outputs on a per cycle basis

  Note:            None.
********************************************************************/
void setup_pwm(void)
{
    /* Holds the PWM interrupt configuration value*/
    unsigned int config;
    /* Configure pwm interrupt enable/disable and set interrupt priorties */
    config = (PWM_INT_DIS & PWM_FLTA_EN_INT & PWM_INT_PR1 & PWM_FLTA_INT_PR4);

    /* clear the Interrupt flags */
    IFS2bits.PWMIF = 0;	
    IFS2bits.FLTAIF = 0;	

    /* Set priority for the period match */
    IPC9bits.PWMIP      = (0x0007 & config);

    /* Set priority for the Fault A */
    IPC10bits.FLTAIP    = (0x0070 & config)>> 4;

    /* enable /disable of interrupt Period match */
    IEC2bits.PWMIE      = (0x0008 & config) >> 3;

    /* enable /disable of interrupt Fault A.*/
	IEC2bits.FLTAIE     = (0x0080 & config) >> 7;


    /* Configure PWM to generate 0 current*/
    PWMCON2bits.UDIS = 0;
    
#ifdef DEMOBOARD
	PDC1 = 0;	// PWM1 used for H bridge
#else
    PDC1 = (FCY/FPWM - 1);	// PWM1 used as sign/magnitude output
#endif
	PDC2 = PDC3 = 0;
    PTPER = ((FCY/FPWM)/2 - 1);      // set the pwm period register(/2 for cnt up/dwn)
    
    SEVTCMP = 0x00;
    /* 1 output is independant and enabled */
#ifdef DEMOBOARD
    PWMCON1 = (PWM_MOD1_COMP & PWM_MOD2_COMP & PWM_MOD3_COMP &	// complementary outputs
				PWM_PEN1L & PWM_PEN1H &
				PWM_PEN2L & PWM_PEN2H &
				PWM_PEN3L & PWM_PEN3H 
				);
#else
    PWMCON1 = (PWM_MOD1_IND & PWM_MOD2_COMP & PWM_MOD3_COMP &
				PWM_PEN1L & PWM_PDIS1H &
				PWM_PEN2L & PWM_PEN2H &					 // PWM2&3 for H bridge; PWM3 Mirrored on PWM1
				PWM_PEN3L & PWM_PEN3H 
				);
#endif
	/* set dead time options, scale = 1, 5*4TCY (about 400ns) */
	DTCON1 = PWM_DTAPS4 & PWM_DTA5;
	
	    /* set up the fault mode override bits and mode */
#ifdef DEMOBOARD
    FLTACON =	PWM_FLTA_MODE_CYCLE &
				PWM_FLTA1_EN &
				PWM_FLTA2_EN &
				PWM_FLTA3_EN &
				PWM_OVA1L_ACTIVE &
				PWM_OVA1H_INACTIVE &
				PWM_OVA2L_ACTIVE &
				PWM_OVA2H_INACTIVE &
				PWM_OVA3L_ACTIVE &
				PWM_OVA3H_INACTIVE ;
#else
    FLTACON =	PWM_FLTA_MODE_CYCLE &
				PWM_FLTA2_EN &
				PWM_FLTA3_EN &
				PWM_OVA2L_ACTIVE &
				PWM_OVA2H_INACTIVE &
				PWM_OVA3L_ACTIVE &
				PWM_OVA3H_INACTIVE ;
#endif
    /* set special event post scaler, output override sync select and pwm update enable */
    PWMCON2 = (PWM_SEVOPS1 & PWM_OSYNC_PWM & PWM_UEN);
    PTCON   = (PWM_EN & PWM_IDLE_CON & PWM_OP_SCALE1 & PWM_IPCLK_SCALE1 & PWM_MOD_UPDN);
    OVDCON = 0xFFFF;	// no output override

}



/*********************************************************************
  Function:        void set_pwm(float output)

  PreCondition:    None.
 
  Input:           current value in output
				   (calculation specific to dspic-servo with 3.3k resistor
				   on current limit pin of the OPA549)

  Output:          None.

  Side Effects:    None.

  Overview:        

  Note:            None.
********************************************************************/
void set_pwm(float output)
{
    
    // ABOUT DEAD TIME:
    // Dead Time creates a output less than expected.
    // Example: output is 200 TCY, and DT is 20 TCY, real output will be 180 TCY!
    // So we add the DT to the PDC registers value, in order to avoid an additional static error.
    // also we HAVE to stay in the 95% zone, to ensure boostsrap capacitor is recharged.
    // otherwise, the H side could un-switch and the corresponding motor pole won't be tied to V+ voltage.
	#ifdef YAPSC_10V
		const long pwm_max = (FCY/FPWM)-20;		// for the 10V version, there is no need to charge the
												// bootstrap capacitors. 100% PWM is possible.
	#else 
    	const long pwm_max = (FCY/FPWM)*0.95 - 1;      // 97% pwm count
    #endif
    short temp_dir;
    long temp;
    
	if ( output >= 0.0 )
		temp_dir = 1;
	else
	{
        temp_dir = 0;
		output = -output;		// flip minus values to an amplitude
	}
	
	if ( output > pid.maxoutput ) output = pid.maxoutput;
	if ( output > 100) output = 100;
	
	temp = pwm_max*(output/100);
	
//	SEVTCMP = temp;	// Special event trigger is on the end of active portion of PWM period

	temp += 20;	// add the dead time to avoid dead band around 0V output
	if(temp > pwm_max)
		temp = pwm_max;
	
	// H control
	if ( temp_dir == 1 )
	{
		PDC2 = temp;	// PWM to the other (plus dead time)
		
		PDC3 = 0;	// 0V to first output
       	#ifdef DEMOBOARD
			PDC1 = 0;
 		#endif
    }
	else
	{
		PDC2 = 0;	// 0V to 2nd output
		
    	PDC3 = temp;	// PWM to the other (plus dead time)
       	#ifdef DEMOBOARD
 			PDC1 = temp;
 		#endif
  		      	
	}
	
#ifndef DEMOBOARD
    PDC1 = temp+(FCY/FPWM)/15;
    #warning "WH set up for sign/mag on PWM1"
#endif
}

