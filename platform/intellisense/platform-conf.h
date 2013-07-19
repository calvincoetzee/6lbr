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
*     platform-conf.h for MBXXX.
* \author
*     Salvatore Pitrulli <salvopitru@users.sourceforge.net>
*     Chi-Anh La <la@imag.fr>
*     Simon Duquennoy <simonduq@sics.se>
*/
/*---------------------------------------------------------------------------*/

#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__

#include <inttypes.h>
#include <string.h>
#include "utils/uartstdio.h" // For memcpm().
/* Platform-dependent definitions */
#define CC_CONF_REGISTER_ARGS          0
#define CC_CONF_FUNCTION_POINTER_ARGS  1
#define CC_CONF_FASTCALL
#define CC_CONF_VA_ARGS                1
#define AUTOSTART_ENABLE			   1
#define CC_CONF_INLINE                 inline


#define CCIF
#define CLIF

typedef unsigned short uip_stats_t;
//#define COFFEE_ADDRESS 0x00030000
// Device UART Configs
#define UART1_SLAVE_SPI						0
#define UART1_CONF_TX_WITH_INTERRUPT		0
#define WITH_SERIAL_LINE_INPUT				1


/* rtimer_second = 11719 */
#define RT_CONF_RESOLUTION                   RES_171US
/* A trick to resolve a compilation error with IAR. */
#ifdef __ICCARM__
#define UIP_CONF_DS6_AADDR_NBU      1
#endif
typedef unsigned long clock_time_t;

#define CLOCK_CONF_SECOND 1024UL
////#ifdef PRINTF
//#undef PRINTF
//#undef PRINT6ADDR
//#undef PRINTLLADDR
////#define PRINTF(...) UARTprintf(__VA_ARGS__)
//#define PRINT6ADDR(addr) UARTprintf(" %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x ", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
//#define PRINTLLADDR(lladdr) UARTprintf(" %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x ",lladdr.u8[0], lladdr.u8[1], lladdr.u8[2], lladdr.u8[3],lladdr.u8[4], lladdr.u8[5], lladdr.u8[6], lladdr.u8[7])
////#endif
typedef unsigned long rtimer_clock_t;
#define RTIMER_CLOCK_LT(a,b)     ((signed short)((a)-(b)) < 0)

/* LEDs ports MB8xxx */
#define LEDS_CONF_GREEN         LED_D1
#define LEDS_CONF_YELLOW        LED_D3
#define LEDS_CONF_RED           LED_D3

#define UIP_ARCH_ADD32           1
#define UIP_ARCH_CHKSUM          0
//#define UIP_FALLBACK_INTERFACE   lms_eth_interface
#define UIP_CONF_BYTE_ORDER      UIP_LITTLE_ENDIAN
#define sprintf(...) 	usprintf(__VA_ARGS__)
#define printf(...) UARTprintf(__VA_ARGS__)
typedef int32_t coffee_page_t;
//#define uip_ipaddr_copy(dest, src) (memcpy(dest,src,sizeof(uip_ipaddr_t)))
/*
 * Make sure that when the WSN channels are being polled that the SD card channel has been turned off
 */
#define WSN_SSI_CLK             GPIO_PIN_2
#define WSN_SSI_TX              GPIO_PIN_5
#define WSN_SSI_RX              GPIO_PIN_4
#define WSN_SSI_PINS            (WSN_SSI_TX | WSN_SSI_RX | WSN_SSI_CLK)
// GPIO for card chip select
#define SDC_CS_GPIO_PORT_BASE      GPIO_PORTD_BASE
#define SDC_CS_GPIO_SYSCTL_PERIPH  SYSCTL_PERIPH_GPIOD
#define SDC_CS                     GPIO_PIN_0
#define WSN_CS_ENABLE 	(SYSCTL_PERIPH_GPIOC)
#define WSN_CS_BASE 	(GPIO_PORTC_BASE)
#define WSN_CS1			(GPIO_PIN_4)
#define WSN_CS2			(GPIO_PIN_5)
#define WSN_CS3			(GPIO_PIN_6)
#endif /* __PLATFORM_CONF_H__ */
