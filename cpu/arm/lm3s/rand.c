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
*			Random number functions for STM32W.
* \author
*			Salvatore Pitrulli <salvopitru@users.sourceforge.net>
*/
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include "contiki.h"
#include "utils/syscalls.h"
#include "inc/lm3s6965.h"
#include <sys/types.h>
typedef char * caddr_t;

#include "driverlib/systick.h"
#if (RANDOM_RAND_MAX != 0xffff)
#warning "RANDOM_RAND_MAX is not defined as 65535."
#endif

int rand(void)
{
	int rand_num;
	rand_num = (int)SysTickValueGet();
	return (int)rand_num;
}

/*
 *	It does nothing, since the rand already generates true random numbers.
 */
void srand(unsigned int seed)
{
}
static unsigned char  __HEAP_START[4096];
static unsigned char*  __HEAP_END = &__HEAP_START[4095];

caddr_t _sbrk ( int incr )
{
	static unsigned char *heap = NULL;
	unsigned char *prev_heap;

	if (heap == NULL) {
		heap = (unsigned char *)&__HEAP_START;
	}
	prev_heap = heap;
	/* check removed to show basic approach */

	if((heap + incr) >= (unsigned char *)&__HEAP_END) return((void *)-1);

	heap += incr;

	return (caddr_t) prev_heap;
}
caddr_t _sbrk_r ( int incr )
{
	static unsigned char *heap = NULL;
	unsigned char *prev_heap;

	if (heap == NULL) {
		heap = (unsigned char *)&__HEAP_START;
	}
	prev_heap = heap;
	/* check removed to show basic approach */

	if((heap + incr) >= (unsigned char *)&__HEAP_END) return((void *)-1);

	heap += incr;

	return (caddr_t) prev_heap;
}
