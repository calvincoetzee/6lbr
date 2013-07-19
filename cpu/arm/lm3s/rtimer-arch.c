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
*			Real-timer specific implementation for STM32W.
* \author
*			Salvatore Pitrulli <salvopitru@users.sourceforge.net>
*/
/*---------------------------------------------------------------------------*/


#include "sys/energest.h"
#include "sys/rtimer.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/hw_timer.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif



static uint32_t time_msb = 0;  // Most significant bits of the current time.

// time of the next rtimer event. Initially is set to the max value.
static rtimer_clock_t next_rtimer_time = 0;


/*---------------------------------------------------------------------------*/
void Timer0IntHandler(void){
	unsigned long Int;
	Int = TimerIntStatus(TIMER0_BASE, TIMER_IMR_TATOIM|TIMER_IMR_TAMIM);
    TimerIntClear(TIMER0_BASE, Int);
  if(Int && TIMER_IMR_TATOIM){  // Overflow event.

    time_msb++;
    rtimer_clock_t now =  ((rtimer_clock_t)time_msb << 16)|TimerValueGet(TIMER0_BASE, TIMER_A);
    
    rtimer_clock_t clock_to_wait = next_rtimer_time - now;
    
    if(clock_to_wait <= 0x10000 && clock_to_wait > 0){ // We must set now the Timer Compare Register.
    	TimerMatchSet(TIMER0_BASE,TIMER_A,clock_to_wait);
  	    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT|TIMER_IMR_TAMIM);
    }    
      
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());

    
  }
  if(Int && TIMER_IMR_TAMIM)
  {


	    //PRINTF("\nCompare event %4x\r\n", TIM1_CNT);
	    //PRINTF("INT_TIM1FLAG %2x\r\n", INT_TIM1FLAG);
	    //ENERGEST_ON(ENERGEST_TYPE_IRQ);
	    rtimer_run_next();
	    //ENERGEST_OFF(ENERGEST_TYPE_IRQ);
	    TimerIntDisable(TIMER0_BASE, TIMER_IMR_TAMIM);

  }
  

  
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_init(void)
{
  // Real Time Timer Will Operate from timer 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC);
    TimerPrescaleSet(TIMER0_BASE, TIMER_A,RT_PRESCALER);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()); // -> Will Provide Ticks on for the resolution
    IntMasterEnable();
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);

}
/*---------------------------------------------------------------------------*/
void rtimer_arch_disable_irq(void)
{
    IntDisable(INT_TIMER0A);
}
/*---------------------------------------------------------------------------*/
void rtimer_arch_enable_irq(void)
{
    IntEnable(INT_TIMER0A);
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t rtimer_arch_now(void)
{
  return ((rtimer_clock_t)time_msb << 16)|(0x0000FFFF&TimerValueGet(TIMER0_BASE, TIMER_A));
}

/*---------------------------------------------------------------------------*/

void
rtimer_arch_schedule(rtimer_clock_t t)
{
  
  PRINTF("rtimer_arch_schedule time %4x\r\n", /*((uint32_t*)&t)+1,*/(uint32_t)t);
  
  next_rtimer_time = t;
  
  rtimer_clock_t now = rtimer_arch_now();
  
  rtimer_clock_t clock_to_wait = t - now;
  
  PRINTF("now %2x\r\n", TIM1_CNT);
  PRINTF("clock_to_wait %4x\r\n", clock_to_wait);
  
  if(clock_to_wait <= 0x10000){ // We must set now the Timer Compare Register.
	  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT|TIMER_IMR_TAMIM);
	  TimerLoadSet(TIMER0_BASE, TIMER_A, clock_to_wait);
    
    //PRINTF("2-INT_TIM1FLAG %2x\r\n", INT_TIM1FLAG);
    
  }
  // else compare register will be set at overflow interrupt closer to the rtimer event.
  
}
