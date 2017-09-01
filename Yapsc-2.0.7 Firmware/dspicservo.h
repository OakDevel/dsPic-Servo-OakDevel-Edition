//---------------------------------------------------------------------
//	File:		dspicservo.h
//
//	Written By:	Lawrence Glaister VE7IT
//
// Purpose: Various header info for the project
//      
// 
//---------------------------------------------------------------------
//
// Revision History
// March 11 2006 --   formatted into multi file project
// Sept 25 2006 -  6x pwm rate
//---------------------------------------------------------------------- 
// define which chip we are using (peripherals change)
#include <p30f4012.h>

//#define DEMOBOARD 1	// test board special HW needs some adjustments
//#define YAPSC_10V	// analogue output version (YAPSC:V1 if this #define is commented!)

#ifdef YAPSC_10V
	// HW specific constants
	#define FPWM 30E3	// 30KHz PWM
	// HW specific io
	#define IN1				_RB0
	#define OUT1			_LATE0
	#define clearOUT1()		(OUT1 = 0)
	#define setOUT1()		(OUT1 = 1)
	
	// HW specific texts
	#define YAPSC_GREETING "YAPSC:10V. Version: 2.0.7 (5 Mar 2010)\r\ndspic-servo by L.Glaister\r\n10V mod by MaX-MoD\r\n"
	#define YAPSC_VERSION "\r\nYTT2F2.0.7Y10\r\n"	// YTT protocol V2; Firmware 2.0.1; 10V version
#else	// YAPSC:V1
	// HW specific constants
	#define FPWM 20E3	// 20KHz PWM
	// HW specific io
	#define clearOUT1()		asm("nop")
	#define setOUT1()		asm("nop")
	// HW specific texts
	#define YAPSC_GREETING "YAPSC:V1 Version: 2.0.7 (5 Mar 2010)\r\ndspic-servo by L.Glaister\r\nMOSFET-H mod by MaX-MoD\r\n"
	#define YAPSC_VERSION "\r\nYTT2F2.0.7Y1\r\n"	// YTT protocol V2; Firmware 2.0.1; V1 version

#endif

#define FCY  29480000       	// Core frequency (FRC+PLL)

#define ERR_MAXE	0b00000001
#define ERR_OVCUR	0b00000010

#define TEST_MAXSAMPLES 200		// step input test samples buffer length. look test.c

// define some i/o bits for the various modules
#ifdef DEMOBOARD
	#define SVO_ENABLE		_RB1
	#define STEP			!PORTDbits.RD1
#else
	#define SVO_ENABLE		!_RB1
	#define STEP			PORTDbits.RD1
#endif
#define ErrLED			_LATB2
#define FltPin			_RE8
#define DIR				_RD1

#define setErrLed()		(ErrLED = 1)
#define clearErrLed()	(ErrLED = 0)
#define toggleErrLed()	(ErrLED = !ErrLED)

// for some reason PI may not be defined in math.h on some systems
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define	TRUE	(1)
#define	FALSE	(0)	

struct PID{
    long int command;	/* commanded value */
    long int feedback;	/* feedback value */
    float error;		/* command - feedback */
    float deadband;		/* param: deadband */
    float maxerror;		/* param: limit for error */
    float maxerror_i;	/* param: limit for integrated error */
    float maxerror_d;	/* param: limit for differentiated error */
    float maxcmd_d;		/* param: limit for differentiated cmd */
    float error_i;		/* opt. param: integrated error */
    float prev_error;	/* previous error for differentiator */
    float error_d;		/* opt. param: differentiated error */
    long int prev_cmd;	/* previous command for differentiator */
    float cmd_d;		/* opt. param: differentiated command */
    float bias;			/* param: steady state offset */
    float pgain;		/* param: proportional gain */
    float igain;		/* param: integral gain */
    float dgain;		/* param: derivative gain */
    float ff0gain;		/* param: feedforward proportional */
    float ff1gain;		/* param: feedforward derivative */
    float maxoutput;	/* param: limit for PID output */
    float output;		/* the output value */
    short enable;		/* enable input */
    short limit_state;	/* 1 if in limit, else 0 */
	short multiplier;	/* pc command multiplier */
	short ticksperservo; /* number of 100us ticks/servo cycle */
    int cksum;		/* data block cksum used to verify eeprom */
};

struct TESTSTRUCT{
	int testInRun;
	int ticks;
	int prediv;				// sampleing frequency = 10KHz / (prediv+1)
	int printCurve;
	int sampleToPrintIndex;
	int sampleIndex;
	int delta;
	int samples[TEST_MAXSAMPLES];
	char outputsamples[TEST_MAXSAMPLES];
};
extern volatile struct TESTSTRUCT testS;

extern short multiplier_double;
