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
*			Watchdog
* \author
*			Salvatore Pitrulli <salvopitru@users.sourceforge.net>
*/
/*---------------------------------------------------------------------------*/

#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/watchdog.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"


/*---------------------------------------------------------------------------*/
void
watchdog_init(void)
{
    //IntEnable(INT_WATCHDOG);
    //
    // Set the period of the watchdog timer.
    //
    //WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet());

    //
    // Enable reset generation from the watchdog timer.
    //
    //WatchdogResetEnable(WATCHDOG0_BASE);
}
/*---------------------------------------------------------------------------*/
void
watchdog_start(void)
{
	  //
	  // Enable the watchdog timer.
	  //
	  //WatchdogEnable(WATCHDOG0_BASE);



}
/*---------------------------------------------------------------------------*/
void
watchdog_periodic(void)
{
  /* This function is called periodically to restart the watchdog
     timer. */
	//WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet());
}
/*---------------------------------------------------------------------------*/
void
watchdog_stop(void)
{
	//HWREG(WATCHDOG0_BASE + WDT_O_CTL) != WDT_CTL_INTEN;
}
/*---------------------------------------------------------------------------*/
void
watchdog_reboot(void)
{
	SysCtlReset();
}
/*---------------------------------------------------------------------------*/
