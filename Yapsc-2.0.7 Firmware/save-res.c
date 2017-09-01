/* interface.c  
 * dsPic30f4011 driver program for the N2PK Vector Network Analyzer
 * This file contains code for the operators interface.
 *
 *  Copyright (C) 2006 By Lawrence Glaister VE7IT
 *             (ve7it@shaw.ca)
 *
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  Compile using: mplab ide
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>		// for PI etc
#include <libpic30.h>
#include "dspicservo.h"
#include "DataEEPROM.h"		// for eeprom read and write routines

extern struct PID pid;
struct PID pidCopy;

void clear_EE( void );

// we need to pull some tricks to get it all done
// structure with all setup constants that is stored in eeprom
// aligned on 32 byte boundary
// seed eeprom with a few default values
//struct PID _EEDATA(32) pidEE2={0.0};
struct PID _EEDATA(32) pidEE={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100, 1, 2};

//=============================================================================
// Routine to calculate a checksum on a section of memory
// call with array size in 16 bit words and ptr to start.
//=============================================================================
int calc_cksum(int sizew, int *adr)
{
	int i;
	int cksum = 0;
	for (i=0; i < sizew-1; i++)
		cksum += *adr++;

//	printf("cksum of %d 16bit words is %d\r\n",sizew,cksum);
	return cksum; 

}

//=============================================================================
// Routine to save setup structure into eeprom
// 
//=============================================================================
int save_setup( void )
{
	int size = sizeof(pid);
	int *sptr = (int *)&pidCopy;
	int res;
	int offset = 0;
	_prog_addressT p;
  	/* method 1 */
  	_init_prog_address(p, pidEE);
  	
	memcpy(&pidCopy, &pid, sizeof(pid));

	// compute correct checksum for upper part of array
	// and place it in the checsum variable
	pidCopy.cksum = calc_cksum((sizeof(pid))/sizeof(int),
							 (int*)&pidCopy);

	// this routine attempts to write the entire ram setup structure
	// into the eeprom on board.
	// write 16 words of structure at a time
	
	clear_EE();
	
	while (size > 0)
	{

		res = WriteEE(sptr, __builtin_tblpage(&pidEE),
							__builtin_tbloffset(&pidEE)+offset, ROW);
							
		//_write_eedata_row(p, (int*) &pid
		if (res)
			printf("write to eeprom failed at offset %d\r\n",offset);

		offset += ROW*2;			// bump offset to destination 32 bytes up 
		sptr   += ROW;			// bump source ptr up 16 words
		size   -= ROW*2;	    // 16 words or 32 bytes/write
		_wait_eedata(); 
	}
	return res;
}

//=============================================================================
// Routine to clear eeprom
// 
//=============================================================================
void clear_EE( void )
{
	_erase_eedata_all();
	_wait_eedata(); 
}

//=============================================================================
//  routine to restore setup data 
// 
//=============================================================================
int restore_setup( void )
{
	_prog_addressT p;
	
  	_init_prog_address(p, pidEE);
	_wait_eedata();
  	(void) _memcpy_p2d16(&pid, p, sizeof(pid)); 
	
	return 0;
}
