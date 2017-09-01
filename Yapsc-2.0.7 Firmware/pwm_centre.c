//---------------------------------------------------------------------
//	File:		pwm.c
//
//	Written By:	Lawrence Glaister
//
// Purpose: This set of routines to run pwn output
//      
// 
//---------------------------------------------------------------------
//
// Revision History
//
// Nov 21 2005 --    first version Lawrence Glaister
// Mar 18 2006 --    stripped to single output for servo card
//---------------------------------------------------------------------- 
#include "dspicservo.h"
#include <pwm.h>
#include <stdio.h>


/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _PWMInterrupt(void)

  PreCondition:    None.
 
  Input:           None

  Output:          None.

  Side Effects:    None.

  Overview:        handles pwm interrupts 

  Note:            None.
********************************************************************/
void __attribute__((__interrupt__)) _PWMInterrupt(void)
{
    IFS2bits.PWMIF =0;
}

void __attribute__((__interrupt__)) _FLTAInterrupt(void)
{
    IFS2bits.FLTAIF =0;
    CurrentLimitLED = 1;
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
    PDC1 = (FCY/FPWM - 1);
	PDC2 = PDC3 = 0;
    PTPER = (FCY/FPWM - 1);      // set the pwm period register(/2 for cnt up/dwn)
    
    SEVTCMP = 0x00;
    /* 1 output is independant and enabled */
    PWMCON1 = (PWM_MOD1_IND & PWM_MOD2_COMP & PWM_MOD3_COMP &  /* independant and comp i/o */
				PWM_PEN1L & PWM_PDIS1H &                    /* use 1L as AOP pwm, 1H is AOP dir (normal i/o pin)*/
				PWM_PEN2L & PWM_PEN2H &					 /* PWM2&3 for H bridge */
				PWM_PEN3L & PWM_PEN3H 
				);
    /* set dead time options, scale = 1, 20*TCY (about 400ns) */
    DTCON1 = PWM_DTAPS1 & PWM_DTA20;
  
    /* set up the fault mode override bits and mode */
    FLTACON = PWM_FLTA_MODE_CYCLE &
              PWM_FLTA1_EN &
              PWM_FLTA2_EN &
              PWM_FLTA3_EN &
              PWM_OVA1L_ACTIVE &
              PWM_OVA1H_INACTIVE &
              PWM_OVA2L_ACTIVE &
              PWM_OVA2H_INACTIVE &
              PWM_OVA3L_ACTIVE &
              PWM_OVA3H_INACTIVE ;

    /* set special event post scaler, output override sync select and pwm update enable */
    PWMCON2 = (PWM_SEVOPS1 & PWM_OSYNC_PWM & PWM_UEN);
    PTCON   = (PWM_EN & PWM_IDLE_CON & PWM_OP_SCALE1 & PWM_IPCLK_SCALE1 & PWM_MOD_UPDN);
    OVDCON = 0xFFFF;

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
    long temp;
	
	if ( output >= 0.0 )
		SVO_DIR = 1;
	else
        	SVO_DIR = 0;
	
	if ( output > 100 ) output = 100;
	if ( output < -100 ) output = -100;
	
	temp = (FCY/FPWM)*((output+100)/200);
	
	SEVTCMP = temp;	/* Special event trigger is on the end of active portion of PWM period */
	
	PDC1 = temp;
}

