/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 15/04/13 4:26p $
 * @brief    Show how to set GPIO pin mode and use pin data input/output control.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "stdint.h"
#include "Seven_Segment.h"
#include "NUC100Series.h"
#include "GPIO.h"
#include "SYS.h"
#include "Scankey.h"


#define PLL_CLOCK           50000000


int main(void)
{
	OpenSevenSegment();
	OpenKeyPad();
   while(1) 
	{
		if(ScanKey() == 0){
	   ShowSevenSegment(3,5);		// display i on 7-segment display
	   CLK_SysTickDelay(1000);	// delay for keeping display
		 CloseSevenSegment();
		
		 ShowSevenSegment(2,1);
		 CLK_SysTickDelay(1000);
		 CloseSevenSegment();
		
		 ShowSevenSegment(1,9);		// display i on 7-segment display
	   CLK_SysTickDelay(1000);
     CloseSevenSegment(); 
		
		 ShowSevenSegment(0,6);		// display i on 7-segment display
	   CLK_SysTickDelay(1000);	
		 CloseSevenSegment();
	   			// increment i
		}
		else{
			ShowSevenSegment(0,ScanKey());
			CLK_SysTickDelay(1000);
		  CloseSevenSegment();
		
		}
	}

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
