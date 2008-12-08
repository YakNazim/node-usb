/*
        FreeRTOS V3.2.3 - Copyright (C) 2003-2005 Richard Barry.

        This file is part of the FreeRTOS distribution.

        FreeRTOS is free software; you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation; either version 2 of the License, or
        (at your option) any later version.

        FreeRTOS is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with FreeRTOS; if not, write to the Free Software
        Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

        A special exception to the GPL can be applied should you wish to distribute
        a combined work that includes FreeRTOS, without being obliged to provide
        the source code for any proprietary components.  See the licensing section
        of http://www.FreeRTOS.org for full details of how and when the exception
        can be applied.

        ***************************************************************************
        See http://www.FreeRTOS.org for documentation, latest information, license
        and contact details.  Please ensure to read the configuration and relevant
        port sections of the online documentation.
        ***************************************************************************
*/

#include "adc.h"
#include "lpc214x.h"




/* Initialize the A/D converter*/
//void vSetupADC( void )
//{
	
	/* Intialize the ADC to hard coded values:
		port 0 channel 3 set for BURST mode
	*/

	// start A/D Port 0 Control Register (AD0CR) address 0xE003 4000
	
	// Initialize channel 3 to BURST mode for continous conversions
	
	
	/* START: bits 26:24, used in software control mode to then conversion is started, 
		not used in Burst mode					
	*/
	//AD0CR  &= (0xF9FFFFFF); //sets START to start conversion
	
	/* EDGE: bit 27, value to start the conversion on a rising edge (0) or a falling edge (1) 
	*/
	//AD0CR |= (1<<27); //sets EDGE to enable conversion on falling edge
	
	// end AD0CR
//}

/*
						pin			PINSELx bits		to_set
	    port 0:				
		channel 1	AD0.1	P0.28		x=1, 25:24			01
		channel 2	AD0.2	P0.29		x=1, 27:26			01
		channel 3	AD0.3	P0.30		x=1, 29:28			01
		channel 4	AD0.4	P0.25		x=1, 19:18			01
		channel 6	AD0.6	P0.4 			x=0, 8:9			11
		channel 7	AD0.7	P0.5			x=0, 11:10			11
	    port 1:	
		channel 0	AD1.0	P0.6			x=0, 13:12			11	
		channel 1	AD1.1		P0.8			x=0, 17:16			11
		channel 2	AD1.2	P0.10			x=0, 21:20			11
		channel 3	AD1.3	P0.12			x=0, 25:24			11
		channel 4	AD1.4	P0.13			x=0, 27:26			11
		channel 5	AD1.5	P0.15			x=0, 31:30			11
		channel 6	AD1.6	P0.21			x=1, 11:10			10
		channel 7	AD1.7	P0.22		x=1, 13:12			01
	
	
	//init adc
*/

void configureADCPort(enum ADCPort port )
{
	unsigned portBASE_TYPE uxADC_Port = getPort(port);
	unsigned portBASE_TYPE uxADC_Channel = getChannel(port);
	
	
	switch(port) {
		case AD0_1:
			SET_BIT(PINSEL1, 24);
			ZERO_BIT(PINSEL1, 25);
			break;
		case AD0_2:
			SET_BIT(PINSEL1, 26);
			ZERO_BIT(PINSEL1, 27);
			break;
		case AD0_3:
			SET_BIT(PINSEL1, 28);
			ZERO_BIT(PINSEL1, 29);
			break;
		case AD0_4:
			SET_BIT(PINSEL1, 18);
			ZERO_BIT(PINSEL1, 19);
			break;
		case AD0_6:
			SET_BIT(PINSEL0, 8);
			SET_BIT(PINSEL0, 9);
			break;
		case AD0_7:
			SET_BIT(PINSEL0, 10);
			SET_BIT(PINSEL0, 11);
			break;	
				
		case AD1_0:
			SET_BIT(PINSEL0, 12);
			SET_BIT(PINSEL0, 13);
			break;
		case AD1_1:
			SET_BIT(PINSEL0, 16);
			SET_BIT(PINSEL0, 17);
			break;
		case AD1_2:
			SET_BIT(PINSEL0, 20);
			SET_BIT(PINSEL0, 21);
			break;
		case AD1_3:
			SET_BIT(PINSEL0, 24);
			SET_BIT(PINSEL0, 25);
			break;
		case AD1_4:
			SET_BIT(PINSEL0, 26);
			SET_BIT(PINSEL0, 27);
			break;
		case AD1_5:
			SET_BIT(PINSEL0, 30);
			SET_BIT(PINSEL0, 31);
			break;
		case AD1_6:
			ZERO_BIT(PINSEL1, 10);
			SET_BIT(PINSEL1, 11);
			break;
		case AD1_7:
			SET_BIT(PINSEL1, 12);
			ZERO_BIT(PINSEL1, 13);
			break;	
	}
	
	
	
	if( uxADC_Port == 0 ) {
		/* SEL: bits 7:0 selects which pins are to be sampled, bit 0 
	 		selects pin AD0.0, bit 7 selects pin AD0.7 			
		*/
		AD0CR |= ( 1 << uxADC_Channel); //set bit X to enable pin AD{port}.{channel} for AD conversion
		
		/* BURST: bit 16, when set the AD converter is in burst mode 
	 	doing repeated conversions at a rate of CLKS		
		*/
		AD0CR |= (1<<16); //sets the AD converter to burst mode for repeated conversions
		
		
		/* CLKDIV: bits 15:8 set the clock for the conversion. 
	 	VPB clock (PCLK) is divided by this value plus 1. 
		Total should be less than 4.5MHz. Value should be greater 
		than 14 for a VPB clock of 60Mhz*/
		AD0CR |= (20<<8); //sets a value of 20 into CLKDIV for a clock of 3MHz for conversions
	
	
		/* CLKS: bits 19:17, number of clocks used for each conversion in burst mode, 
		000 11clks,10bits -- 111 4clks,3bits*/
		//value 000 into CLKS for 11 clocks for 10 bits of resolution
		AD0CR &= ~(1<<17);
		AD0CR &= ~(1<<18);
		AD0CR &= ~(1<<19);

		/* PDN: bit 21, set to enanble the ADC, clear to disable the ADC*/
		AD0CR |= (1<<21); //sets bit 21 to turn on the ADC		

	} else if( uxADC_Port == 1 ) {
		/* SEL: bits 7:0 selects which pins are to be sampled, bit 0 
	 		selects pin AD0.0, bit 7 selects pin AD0.7 			
		*/
		AD1CR |= ( 1 << uxADC_Channel); //set bit X to enable pin AD{port}.{channel} for AD conversion
		
		/* BURST: bit 16, when set the AD converter is in burst mode 
	 	doing repeated conversions at a rate of CLKS		
		*/
		AD1CR |= (1<<16); //sets the AD converter to burst mode for repeated conversions
		
		
		/* CLKDIV: bits 15:8 set the clock for the conversion. 
	 	VPB clock (PCLK) is divided by this value plus 1. 
		Total should be less than 4.5MHz. Value should be greater 
		than 14 for a VPB clock of 60Mhz*/
		AD1CR |= (20<<8); //sets a value of 20 into CLKDIV for a clock of 3MHz for conversions
		
		
		
		/* CLKS: bits 19:17, number of clocks used for each conversion in burst mode, 
		000 11clks,10bits -- 111 4clks,3bits*/
		//value 000 into CLKS for 11 clocks for 10 bits of resolution
		AD1CR &= ~(1<<17);
		AD1CR &= ~(1<<18);
		AD1CR &= ~(1<<19);
		
		/* PDN: bit 21, set to enanble the ADC, clear to disable the ADC*/
		AD1CR |= (1<<21); //sets bit 21 to turn on the ADC
	}	
}



/* Checks if the uxADC_Channel on uxADC_Port is done with a conversion */
portBASE_TYPE uxADCDone(enum ADCPort port )
{
	/* To check the status of a conversion you read bits 7:0 of the AD{port}STAT register.
		Bit 0 corresponds to channel 0 and bit 7 corresponds to channel 7 
	*/
	
	unsigned portBASE_TYPE uxADC_Port = getPort(port);
	unsigned portBASE_TYPE uxADC_Channel = getChannel(port);

	switch( uxADC_Port )
	{
		case 0:
			return (unsigned portBASE_TYPE) ( AD0STAT & (1 << (uxADC_Channel) ) );

			break;
		case 1:
			return (unsigned portBASE_TYPE) ( AD1STAT & (1 << (uxADC_Channel) ) );

			break;
		default:
			return 0; // invalid port so return false

			break;
	}
}

/* Returns the conversion result of the channel and port specified, or -1 if the conversion is not done yet. 
 * or -2 if the port specified was invalid. */
portLONG cADC_Result( enum ADCPort port )
{
	/* To read the value of the conversion you read the value of register AD{port}DR{channel}.
		The result is in bits 15:6, the OVERRUN check is bit 30, and the DONE check is in bit 31 */	

	unsigned portBASE_TYPE uxADC_Port = getPort(port);
	unsigned portBASE_TYPE uxADC_Channel = getChannel(port);

	
	unsigned portLONG ulADCValue = 0;

	switch( uxADC_Port )
	{
		case 0:
			// 0xE0034010 is AD0DR0 register, this will add the channel number to point to the correct location for that channel
			ulADCValue = (*((volatile unsigned long *) (0xE0034010 + (uxADC_Channel * 4))) );
			
			if( !( ulADCValue & (1<<31) ) )
				return(-1); // conversion not done return 0

			ulADCValue &= ~(3<<30);  //bits 30 and 31 indicate overrun and done bits, but not useful to the caller.
    		ulADCValue = (ulADCValue >> 6);
			return ulADCValue;

			break;
		case 1:
			// 0xE0060010 is AD1DR0 register, this will add the channel number to point to the correct location for that channel
			ulADCValue = (*((volatile unsigned long *) (0xE0060010 + (uxADC_Channel * 4))) );

			if( !( ulADCValue & (1<<31) ) )
				return(-1); // conversion not done return 0

			ulADCValue &= ~(3<<30);  //bits 30 and 31 indicate overrun and done bits, but not useful to the caller.
    		ulADCValue = (ulADCValue >> 6);
			return ulADCValue;

			break;
	}
	
	return(-2);

}


portBASE_TYPE getPort(enum ADCPort port) {
	switch(port) {
		case AD0_1:
		case AD0_2:
		case AD0_3:
		case AD0_4:
		case AD0_6:
		case AD0_7:
			return(0);
			break;
		case AD1_0:
		case AD1_1:
		case AD1_2:
		case AD1_3:
		case AD1_4:
		case AD1_5:
		case AD1_6:
		case AD1_7:
			return(1);
			break;
	}
	return(0);
}

portBASE_TYPE getChannel(enum ADCPort port) {
	switch(port) {
		case AD1_0:
			return(0);
		case AD0_1:
		case AD1_1:
			return(1);
		case AD0_2:
		case AD1_2:
			return(2);
		case AD0_3:
		case AD1_3:
			return(3);
		case AD0_4:
		case AD1_4:
			return(4);
		case AD1_5:
			return(5);
		case AD0_6:
		case AD1_6:
			return(6);
		case AD0_7:
		case AD1_7:
			return(7);
	}
	//ERROR
	return(0);
}


