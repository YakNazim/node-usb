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

#ifndef ADC_H
#define ADC_H

#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		short
#define portSTACK_TYPE	unsigned portLONG
#define portBASE_TYPE	portLONG




/* A/D Converter 0 (AD0) */
#define AD0CR           (*((volatile unsigned long *) 0xE0034000))
#define AD0GDR          (*((volatile unsigned long *) 0xE0034004))
#define AD0STAT         (*((volatile unsigned long *) 0xE0034030))
#define AD0INTEN        (*((volatile unsigned long *) 0xE003400C))
#define AD0DR0          (*((volatile unsigned long *) 0xE0034010))
#define AD0DR1          (*((volatile unsigned long *) 0xE0034014))
#define AD0DR2          (*((volatile unsigned long *) 0xE0034018))
#define AD0DR3          (*((volatile unsigned long *) 0xE003401C))
#define AD0DR4          (*((volatile unsigned long *) 0xE0034020))
#define AD0DR5          (*((volatile unsigned long *) 0xE0034024))
#define AD0DR6          (*((volatile unsigned long *) 0xE0034028))
#define AD0DR7          (*((volatile unsigned long *) 0xE003402C))

/* A/D Converter 1 (AD1) */
#define AD1CR           (*((volatile unsigned long *) 0xE0060000))
#define AD1GDR          (*((volatile unsigned long *) 0xE0060004))
#define AD1STAT         (*((volatile unsigned long *) 0xE0060030))
#define AD1INTEN        (*((volatile unsigned long *) 0xE006000C))
#define AD1DR0          (*((volatile unsigned long *) 0xE0060010))
#define AD1DR1          (*((volatile unsigned long *) 0xE0060014))
#define AD1DR2          (*((volatile unsigned long *) 0xE0060018))
#define AD1DR3          (*((volatile unsigned long *) 0xE006001C))
#define AD1DR4          (*((volatile unsigned long *) 0xE0060020))
#define AD1DR5          (*((volatile unsigned long *) 0xE0060024))
#define AD1DR6          (*((volatile unsigned long *) 0xE0060028))
#define AD1DR7          (*((volatile unsigned long *) 0xE006002C))



#define SET_BIT(variable, bitNumber)    (variable |= (1<<bitNumber))
#define ZERO_BIT(variable, bitNumber)   (variable &= ~(1<<bitNumber))


enum ADCPort {AD0_1, AD0_2, AD0_3, AD0_4, AD0_6, AD0_7, AD1_0, AD1_1, AD1_2, AD1_3, AD1_4, AD1_5,
		AD1_6, AD1_7};

void configureADCPort(enum ADCPort port );
portBASE_TYPE getChannel(enum ADCPort port);
portBASE_TYPE getPort(enum ADCPort port);

/* Checks if the uxADC_Channel on uxADC_Port is done with a conversion */
portBASE_TYPE uxADCDone(enum ADCPort port);

/* Returns the conversion result of the channel and port specified */
portLONG cADC_Result( enum ADCPort port );

#endif
