/*
 * Copyright (c) 2010, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the Contiki OS
 *
 */
/*---------------------------------------------------------------------------*/
/**
* \file
*					Machine dependent LM3S6965 SPI Code.
* \author
*					Calvin Coetzee
* \version
*					0.1
* \since
*					12/02/2012
*/
/*---------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
//#include <io.h>
//#include <signal.h>
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "sys/energest.h"
#include "interrupt.h"
#include "sysctl.h"
#include "dev/spi.h"
#include "lib/ringbuf.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "dev/spi.h"

/*
 * SPI DRIVER FOR CONTIKI
 * SPI_INIT
 * SPI_WRITE
 * SPI_READ
 * SPI_WRITE_DATA
 * SPI_READ_DATA
 *
 */
#define SSI_CLK             GPIO_PIN_2
#define SSI_TX              GPIO_PIN_5
#define SSI_RX              GPIO_PIN_4
#define SSI_PINS            (SSI_TX | SSI_RX | SSI_CLK)

void SPI_INIT(void)
{ unsigned long i;
	 	//
	    // The SSI0 peripheral must be enabled for use.
	    //
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	    //
	    // For this example SSI0 is used with PortA[5:2].  The actual port and pins
	    // used may be different on your part, consult the data sheet for more
	    // information.  GPIO port A needs to be enabled so these pins can be used.
	    // TODO: change this to whichever GPIO port you are using.
	    //
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	    /* Configure the appropriate pins to be SSI instead of GPIO */
	    GPIOPinTypeSSI(GPIO_PORTA_BASE, SSI_PINS);
	    GPIOPadConfigSet(GPIO_PORTA_BASE, SSI_PINS, GPIO_STRENGTH_8MA,
	                     GPIO_PIN_TYPE_STD_WPU);
	    GPIOPadConfigSet(GPIO_PORTA_BASE, SSI_CLK, GPIO_STRENGTH_8MA,
	                     GPIO_PIN_TYPE_STD_WPD);
	    /* Set the maximum speed as half the system clock, with a max of 12.5 MHz. */
	    i = SysCtlClockGet() / 4;
	    if(i > 500000)
	    {
	        i = 500000;
	    }
	    /* Configure the SSI0 port */
	    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
	                       SSI_MODE_MASTER, i, 8);
	    SSIEnable(SSI0_BASE);
}
void SPI_WRITE(unsigned long Data)
{
	SSIDataPut(SSI0_BASE, Data);
}
void SPI_READ(unsigned long * Data)
{
	//SSIDataPut(SSI0_BASE, 0x00);
	SSIDataGet(SSI0_BASE, Data);
}
void SPI_WRITE_DATA(unsigned long* Data, unsigned long Lenth)
{unsigned long ptr;
	for(ptr = 0 ; ptr < Lenth ; ptr++)
	{
		SSIDataPut(SSI0_BASE, Data);
	}
}
void SPI_READ_DATA(unsigned long* Data, unsigned long Lenth)
{unsigned long ptr;
	for(ptr = 0 ; ptr < Lenth ; ptr++)
	{
		SSIDataPut(SSI0_BASE, 0x00);
		SSIDataGet(SSI0_BASE, &Data[ptr]);
	}
}


