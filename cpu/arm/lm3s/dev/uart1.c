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
*					Machine dependent LM3S6965 Uart Code.
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
#include "dev/uart1.h"
#include "lib/ringbuf.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"

static int (*uart1_input_handler)(unsigned char c);

static volatile uint8_t transmitting;
// TEST

#ifdef UART1_CONF_TX_WITH_INTERRUPT
#define TX_WITH_INTERRUPT UART1_CONF_TX_WITH_INTERRUPT
#else /* UART1_CONF_TX_WITH_INTERRUPT */
#define TX_WITH_INTERRUPT 1
#endif /* UART1_CONF_TX_WITH_INTERRUPT */


#if TX_WITH_INTERRUPT

#ifdef UART1_CONF_TX_BUFSIZE
#define UART1_TX_BUFSIZE UART1_CONF_TX_BUFSIZE
#else /* UART1_CONF_TX_BUFSIZE */
#define UART1_TX_BUFSIZE 64
#endif /* UART1_CONF_TX_BUFSIZE */

static struct ringbuf txbuf;
static uint8_t txbuf_data[UART1_TX_BUFSIZE];
#endif /* TX_WITH_INTERRUPT */

/*---------------------------------------------------------------------------*/
//uint8_t
//uart1_active(void)
//{
//  return ((~ UTCTL1) & TXEPT) | transmitting;
//}
/*---------------------------------------------------------------------------*/
void
uart1_set_input(int (*input)(unsigned char c))
{
  uart1_input_handler = input;
}
/*---------------------------------------------------------------------------*/
void
uart1_writeb(unsigned char c)
{
//  watchdog_periodic();
#if TX_WITH_INTERRUPT

#else /* TX_WITH_INTERRUPT */
  UARTCharPut(UART1_BASE, (unsigned char)c);
#endif /* TX_WITH_INTERRUPT */
}
/*---------------------------------------------------------------------------*/
#if ! WITH_UIP /* If WITH_UIP is defined, putchar() is defined by the SLIP driver */
#endif /* ! WITH_UIP */
/*---------------------------------------------------------------------------*/
/**
 * Initalize the RS232 port.
 *
 */
void
uart1_init(unsigned long ubr)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    //IntMasterEnable();
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), ubr,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
}
/*---------------------------------------------------------------------------*/
void uart1_rx_interrupt(void);
void uart1_tx_interrupt(void);

void UART1IntHandler(void)
{

  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  unsigned long ulStatus;

      //
      // Get the interrrupt status.
      //
      ulStatus = UARTIntStatus(UART1_BASE, true);
      uart1_rx_interrupt();
      //
      // Clear the asserted interrupts.
      //
      UARTIntClear(UART1_BASE, ulStatus);
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);

}

void uart1_rx_interrupt(void)
{
  uint8_t c;  
  
  c = (unsigned char)UARTCharGet(UART1_BASE);
  if(uart1_input_handler != NULL) {
    uart1_input_handler(c);
  }  

}
/*---------------------------------------------------------------------------*/
#if TX_WITH_INTERRUPT
void uart1_tx_interrupt(void)
{

  if(ringbuf_elements(&txbuf) == 0) {
    transmitting = 0;
    INT_SC1CFG &= ~INT_SCTXFREE;
  } else {

    SC1_DATA = ringbuf_get(&txbuf);
  }
  
}
#endif /* TX_WITH_INTERRUPT */
/*---------------------------------------------------------------------------*/
void __io_putchar(char c)
{
	UARTCharPut(UART1_BASE, (unsigned char)c);
}
