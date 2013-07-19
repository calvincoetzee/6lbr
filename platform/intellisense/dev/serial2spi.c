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
*			Leds.
* \author
*			Salvatore Pitrulli <salvopitru@users.sourceforge.net>
*/
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "contiki-net.h"
#include "contiki-conf.h"
/* Hardware library includes. */
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_ethernet.h"
#include "driverlib/ethernet.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "dev/serial2spi.h"
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
*					Machine dependent STM32W radio code.
* \author
*					Salvatore Pitrulli
*					Chi-Anh La la@imag.fr
*         Simon Duquennoy <simonduq@sics.se>
*/
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "net/mac/frame802154.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "sys/rtimer.h"
#include "dev/spi.h"
#include "dev/serial2spi.h"
#include "platform-conf.h"
#include "ssi.h"
#include "hw_ssi.h"
/*
 * Process and global variable declerations
 */
PROCESS(spi_poll, "SPI poll");
 static unsigned char bypass = 0;
 static unsigned char Delimit = 0;
 static unsigned char Rx_Delimit = 0;
 static unsigned char lenth = 0;
 static unsigned long RxData;
 static unsigned long Channel;
 static unsigned int BuffLenth = 0;
 static unsigned char* Data_check;
 static unsigned char Buffer1_Lock,Buffer1_Lenth,Buffer2_Lock,Buffer2_Lenth;
 static unsigned char Buffer1[BUFFER_SIZE];
 static unsigned char Buffer2[BUFFER_SIZE];
void serial2spi_init(void)
{
	// Setup the ring buffers for the SPI driver
	// Init the SPI subsystem
	 unsigned long i;
	 unsigned char dat;
	// Setup SPI system for MASTER MODE
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
    SysCtlPeripheralEnable(SDC_CS_GPIO_SYSCTL_PERIPH);
    /*
     * This doesn't really turn the power on, but initializes the
     * SSI port and pins needed to talk to the card.
     */
    /* Enable the peripherals used to drive the SDC on SSI, and the CS */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(SDC_CS_GPIO_PORT_BASE, SDC_CS);
    GPIOPadConfigSet(SDC_CS_GPIO_PORT_BASE, SDC_CS, GPIO_STRENGTH_8MA,
                     GPIO_PIN_TYPE_STD_WPU);
    /* Deassert the SSI0 chip select */
    GPIOPinWrite(SDC_CS_GPIO_PORT_BASE, SDC_CS, SDC_CS);
    /* Configure the appropriate pins to be SSI instead of GPIO */
    GPIOPinTypeSSI(GPIO_PORTA_BASE, WSN_SSI_PINS);
    //GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(WSN_CS_BASE, WSN_CS1 | WSN_CS2 | WSN_CS2);
    GPIOPadConfigSet(GPIO_PORTA_BASE, WSN_SSI_PINS, GPIO_STRENGTH_8MA,
                     GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTA_BASE, WSN_SSI_CLK, GPIO_STRENGTH_8MA,
                     GPIO_PIN_TYPE_STD_WPD);
    GPIOPadConfigSet(WSN_CS_BASE, (WSN_CS1|WSN_CS2|WSN_CS3), GPIO_STRENGTH_8MA,
    				GPIO_PIN_TYPE_STD);
	// Make sure devices are turned off
	GPIOPinWrite(WSN_CS_BASE,WSN_CS1,WSN_CS1);
	GPIOPinWrite(WSN_CS_BASE,WSN_CS2,WSN_CS2);
	GPIOPinWrite(WSN_CS_BASE,WSN_CS3,WSN_CS3);

    /* Set the maximum speed as half the system clock, with a max of 12.5 MHz. */
    i = SysCtlClockGet() / 4;
    if(i > 750000)
    {
        i = 750000;
    }
    /* Configure the SSI0 port */
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, i, 8);
    SSIEnable(SSI0_BASE);
    Buffer1_Lock = 0;
    Buffer1_Lenth = 0;
    Buffer2_Lock = 0;
    Buffer2_Lenth = 0;
}

 PROCESS_THREAD(spi_poll, ev, data)
 { 	 int i;
 	 static struct etimer spi_timer;

	  PROCESS_BEGIN();
	  serial2spi_init();

	  /* Send a packet every 60-62 seconds. */
	  etimer_set(&spi_timer, CLOCK_SECOND/5);
 	  while(1)
 	  {
 		    PROCESS_WAIT_EVENT();
 		    if(ev == PROCESS_EVENT_TIMER || ev == PROCESS_EVENT_POLL) {
					 if(WSN_Select_Channel(Channel))
							{	//WSN_Select_Channel(Channel);
								//DataIncomin = 0;
								//BuffLenth = 10;
								//if(BuffLenth == 0){BuffLenth = 10;lenth=0;}
								// Channel is allowed to be active now run the SPI routine
								// Get the Send Buffer
								if(Buffer1_Lock == 2)
								{ 	BuffLenth = Buffer1_Lenth;
									Data_check = Buffer1;
									Buffer1_Lock = 1;

								}else{ if(Buffer2_Lock == 2)
								{ BuffLenth = Buffer2_Lenth;
									Data_check = Buffer2;
									Buffer2_Lock = 1;
								}else
								{
								// Flush the slip buffer
								slip_flushbuf(0);
								}

								}
								wsn_delay();
								for(i = 0 ; i < BUFFER_SIZE ; i++)
								{
									if(BuffLenth > 0)
									{
									SSIDataPut(SSI0_BASE, SPI_Data_check());}else
									{SSIDataPut(SSI0_BASE, 0xFF);}
									if((HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_RNE))
									{
									SSIDataGet(SSI0_BASE, &RxData); /* read data frm rx fifo */

									wsn_delay();
									WSN_Deselect_Channel(Channel);
									RxData =  SPI_Rx_Data_check(RxData);
									if(!bypass){
													//Channel_Recieve(&RxData,Channel);
													i--;
													serial_input(&RxData);
												}else
												{
													wsn_delay();
												}
									}
								wsn_delay();
								WSN_Deselect_Channel(Channel);
								wsn_delay();
								WSN_Select_Channel(Channel);
								wsn_delay();
								}
								WSN_Deselect_Channel(Channel);
								//Channel = Next_Channel(Channel);

							// Check the data recieved
							if(Buffer2_Lock == 1){Buffer2_Lock = 0;}
							if(Buffer1_Lock == 1){Buffer1_Lock = 0;}
							}
					  etimer_set(&spi_timer, CLOCK_SECOND/10);
					//	etimer_reset(&spi_timer);
 		    }
 	  }
 	  PROCESS_END();
 }
static int Device_Search;
 void Channel_Recieve(unsigned long *Data,unsigned long Channel)
 {
 Device_Search++;

 }
 unsigned long Next_Channel(unsigned long Channel)
 {
 	switch(Channel)
 	{
 	case  WSN_CS1: return(WSN_CS2);
 	case  WSN_CS2: return(WSN_CS3);
 	case  WSN_CS3: return(WSN_CS1);
 	default: return(WSN_CS1);
 	}
 }
 int WSN_Select_Channel (unsigned long Channel)
 { unsigned long SD_Check = 0;
 	// First check to see of the SD CARD is not running
// 	SD_Check = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0);
// 	if(SD_Check)
// 	{
 		GPIOPinWrite(WSN_CS_BASE, WSN_CS1, 0);
 		GPIOPinWrite(WSN_CS_BASE, WSN_CS2, WSN_CS2);
 		GPIOPinWrite(WSN_CS_BASE, WSN_CS3, WSN_CS3);
 		return(1);
// 	}else
// 	{
// 		return(0);
// 	}
 }
 int WSN_Deselect_Channel (unsigned long Channel)
 {
 	GPIOPinWrite(WSN_CS_BASE, WSN_CS1, WSN_CS1);
 	GPIOPinWrite(WSN_CS_BASE, WSN_CS2, WSN_CS2);
 	GPIOPinWrite(WSN_CS_BASE, WSN_CS3, WSN_CS3);
     return(1);
 }
 //unsigned char Delimit = 0;
 unsigned long SPI_Data_check(void)
 {	unsigned char Tx_Value;

 	if(Delimit > 0)
 	{	Tx_Value = Delimit;
 		Delimit = 0;
 		Data_check++;
 		// if(lenth > BuffLenth){BuffLenth = 0; lenth = 0;}
 		return(Tx_Value);
 	}else
 	{	 //Data_check = Tx_packet_buf[lenth];

 	     //BuffLenth--;
 		switch(*Data_check)
 		{
 		case 0xFF:
 			Delimit = 0x1F;
 			return(0x1F);
 		case 0x1F:
 			Delimit = 0x2F;
 			return(0x1F);
 		case 0x2F:
 			Delimit = 0x2F;
 			return(0x2F);
 		default:
 	 		 if(BuffLenth == 0){BuffLenth = 0;return((unsigned long)(*Data_check));}
 	 		Tx_Value = *Data_check;
 	 		Data_check++;
 	 		BuffLenth = BuffLenth - 1;
 			return((unsigned long)(Tx_Value));
 		}

 	}

 }
 unsigned char SPI_Rx_Data_check(unsigned char data)
 {unsigned char c;
 		//c = SC1_DATA;
 		c = data;
 		if(Rx_Delimit > 0)
 		{	bypass = 0;
 			switch(Rx_Delimit)
 			{
 			case 0x1F:
 				Rx_Delimit = 0;
 			 if( c == 0x1F)
 				 {return(0xFF);}
 			 if( c == 0x2F)
 			 	 {return(0x1F);}
 			 	 return 0;
 			case 0x2F:
 				Rx_Delimit = 0;
 				if(c == 0x2F)
 				{return(0x2F);}
 				return(0);
 			}
 		}else
 		{
 			switch(c)
 			{
 			case 0x1F:
 				Rx_Delimit = 0x1F;
 				bypass = 1;
 				return(0);
 			case 0x2F:
 				Rx_Delimit = 0x2F;
 				bypass = 1;
 				return(0);
 			case 0xFF:
 				bypass = 1;
 				Rx_Delimit = 0;
 				return(0);
 			default:
 				bypass = 0;
 				Rx_Delimit = 0;
 				return(c);
 			}

 		}
 }
 // Delay function for the WSN selectors
 void wsn_delay(void)
 {unsigned int Delay;
 	for(Delay = 0 ; Delay < DELAY_TIME ; Delay++){	}
 }
 int WSN_Send_Command(unsigned char* Data,int lenth)
 {
 	// This function que's a command to be sent
 	// -> This function makes use of a ping pong buffer
 	// -> 2 buffers available for sending info
 	// -> Buffer State 0 empty 1 sending 2 pending
 	// Type 1 -> Char, 2 -> binary
 	if(Buffer1_Lock == 0)
 	{
 		// Setup the buffer Command
 		Buffer1_Lenth = 0;
 		memset(Buffer1, 0 ,BUFFER_SIZE);
 		memcpy(Buffer1, Data,lenth);
 		Buffer1_Lenth = lenth;
 		Buffer1_Lock = 2;
 		// TODO: if process is not currently busy awake task
 		return(lenth);
 	}
 	if(Buffer2_Lock == 0)
 	{	// Setup the Buffer Command
 		Buffer2_Lenth = 0;
 		memset(Buffer2, 0 ,BUFFER_SIZE);
 		memcpy(Buffer2, Data,lenth);
 		Buffer2_Lock = 2;
 		Buffer2_Lenth = lenth;
 		// TODO: if process is not currently busy awake task
 		return(lenth);
 	}
 	return(-1);
 }
