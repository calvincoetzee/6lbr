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
*			Contiki main file.
* \author
*			Salvatore Pitrulli <salvopitru@users.sourceforge.net>
*			Chi-Anh La <la@imag.fr>
*/
/*---------------------------------------------------------------------------*/




#include <stdio.h>


#include "contiki.h"
#include "dev/watchdog.h"
#include "dev/leds.h"
#include "dev/uart1.h"
#include "dev/uart2.h"
#include "dev/serial-line.h"
#include "net/netstack.h"
#include "net/rime/rimeaddr.h"
#include "net/rime.h"
#include "net/uip-ds6.h"
#include "net/uip.h"
#include "net/uip.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "utils/uartstdio.h"
#include "driverlib/ethernet.h"
#include "dev/uart2.h"
#include "dev/sw_uart_basic.h"

#include "cetic-6lbr.h"
#include "platform-init.h"
#include "packet-filter.h"
#include "eth-drv.h"
#include "nvm-config.h"
#include "rio.h"

//#include "hello-world.h"
#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF(" %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x ", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF(" %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x ",lladdr.u8[0], lladdr.u8[1], lladdr.u8[2], lladdr.u8[3],lladdr.u8[4], lladdr.u8[5], lladdr.u8[6], lladdr.u8[7])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(lladdr)
#endif


#if UIP_CONF_IPV6
PROCINIT(&tcpip_process);
//PROCINIT(&tcpip_process,&ethernet_driver_process,);
#else
//PROCINIT(&tcpip_process);
//PROCINIT(&tcpip_process);
#warning "No TCP/IP process!"
#endif
#define VPT_SENSOR_SYSTEM "Intellisense V1.01"

void IPv6addAddress(uint16_t addr0, uint16_t addr1, uint16_t addr2, uint16_t addr3, uint16_t addr4, uint16_t addr5, uint16_t addr6, uint16_t addr7);
/*---------------------------------------------------------------------------*/
static void
set_rime_addr(void)
{  uip_ipaddr_t ipaddr;
  int i;
  union {
    uint8_t u8[8];
  }eui64;


                  eui64.u8[0] = 0x1E;
                  eui64.u8[1] = 0x00;
                  eui64.u8[2] = 0x00;
                  eui64.u8[3] = 0x00;
                  eui64.u8[4] = 0x48;
                  eui64.u8[5] = 0x1D;
                  eui64.u8[6] = 0x52;
                  eui64.u8[7] = 0x83;
                  uip_ip6addr(&ipaddr, 0xbbbb, 0, 0, 0, 0, 0, 0, 1);

#if UIP_CONF_IPV6
  memcpy(&uip_lladdr.addr, &eui64, sizeof(uip_lladdr.addr));
#endif

#if UIP_CONF_IPV6
  rimeaddr_set_node_addr((rimeaddr_t *)&eui64);
#else
  rimeaddr_set_node_addr((rimeaddr_t *)&eui64.u8[8-RIMEADDR_SIZE]);
#endif
//
//  printf("\r\nRime started with address ");
//  for(i = 0; i < sizeof(rimeaddr_t) - 1; i++) {
//    printf("%d.", rimeaddr_node_addr.u8[i]);
//  }
//  printf("%d\n", rimeaddr_node_addr.u8[i]);

}
/*---------------------------------------------------------------------------*/
static unsigned long process_counter = 0;

int
main(void)
{
    int r;

  /*
   * Initialize hardware.
   */
    //
    // Set the clocking to run at 50MHz from the PLL.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_8MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    IntMasterEnable();
  clock_init();
  uart1_init(115200);
  UARTStdioInit(1);
  uart2_init(38400);
//  UARTStdioInit(2);
//  UARTStdioInit(2);
  // Led initialization
  //leds_init();

  //INTERRUPTS_ON();

  PRINTF("\r\nStarting ");
  PRINTF(CONTIKI_VERSION_STRING);
  PRINTF(" on %s\r\n",VPT_SENSOR_SYSTEM);

  /*
   * Initialize Contiki and our processes.
   */

  process_init();
  //uart1_init(115200);
// SW_UARTConfigSetExpClk(0, SysCtlClockGet(), 19200,
//                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE));

      //
      // Enable the software UART.
      //
 //    SW_UARTEnable(TIMER1_BASE);
 #if WITH_SERIAL_LINE_INPUT
  uart1_set_input(serial_line_input_byte);
  serial_line_init();
 #endif


  /* rtimer and ctimer should be initialized before radio duty cycling layers*/
  rtimer_init();
  /* etimer_process should be initialized before ctimer */
  process_start(&etimer_process, NULL);
  ctimer_init();
//  process_start(&ethernet_driver_process, NULL);

  netstack_init();
  set_rime_addr();
//  process_start(&spi_poll, NULL);
  printf("\r\n%s %s, channel check rate %lu Hz\n",
         NETSTACK_MAC.name, NETSTACK_RDC.name,
         CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0 ? 1:
                                  NETSTACK_RDC.channel_check_interval()));
  printf("\r\n802.15.4 PAN ID 0x%x, EUI-%d:",
      IEEE802154_CONF_PANID, UIP_CONF_LL_802154?64:16);
  uip_debug_lladdr_print(&rimeaddr_node_addr);
  printf(", radio channel %u\n", RF_CHANNEL);

  procinit_init();
  //energest_init();

  //ENERGEST_ON(ENERGEST_TYPE_CPU);
  // for testing under Linux: ping6 -I eth0 fe80::1234
 // IPv6addAddress(0xfe80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1234);
  // add "Global Unicast Address"
  // for testing under Linux: ping6 2a00:eb0:100:15::1234
 // IPv6addAddress(0xaaaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1234);
 // process_start(&spi_poll, NULL);
 autostart_start(autostart_processes);

  //cetic_6lbr_init();

  process_counter = 0;
  //watchdog_start();
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,(GPIO_PIN_3));
  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,(GPIO_PIN_3));
  GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
  GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
//  cetic_6lbr();
  while(1){
//
//
//	  SELECT_WSN1();
//	  SPI_WRITE((unsigned long)0x10);
//	  SPI_READ(&Dummy);
//	  if(Dummy == 0x09)
//	  {
//		  // Toggle led
//		  if(GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_3))
//		  {
//			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
//		  }else
//		  {
//			  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
//		  }
//	  }
//	  DESELECT_WSN1();
    do {
      /* Reset watchdog. */
      //watchdog_periodic();
      r = process_run();
      process_counter++;
    } while(r > 0);



   // ENERGEST_OFF(ENERGEST_TYPE_CPU);
    //watchdog_stop();
   // ENERGEST_ON(ENERGEST_TYPE_LPM);
    /* Go to idle mode. */
    //halSleepWithOptions(SLEEPMODE_IDLE,0);
    /* We are awake. */
    //watchdog_start();
   // ENERGEST_OFF(ENERGEST_TYPE_LPM);
   // ENERGEST_ON(ENERGEST_TYPE_CPU);

  }

}



/*int8u errcode __attribute__(( section(".noinit") ));

void halBaseBandIsr(){

  errcode = 1;
  leds_on(LEDS_RED);
}

void BusFault_Handler(){

  errcode = 2;
  leds_on(LEDS_RED);
}

void halDebugIsr(){

  errcode = 3;
  leds_on(LEDS_RED);
}

void DebugMon_Handler(){

  errcode = 4;
  //leds_on(LEDS_RED);
}

void HardFault_Handler(){

  errcode = 5;
  //leds_on(LEDS_RED);
  //halReboot();
}

void MemManage_Handler(){

  errcode = 6;
  //leds_on(LEDS_RED);
  //halReboot();
}

void UsageFault_Handler(){

  errcode = 7;
  //leds_on(LEDS_RED);
  //halReboot();
}*/

void Default_Handler()
{
  //errcode = 8;
  //leds_on(LEDS_RED);
  //halReboot();
    //
    // Go into an infinite loop.
    //
	SysCtlReset();
    while(1)
    {
    }
}
//void IPv6addAddress(uint16_t addr0, uint16_t addr1, uint16_t addr2, uint16_t addr3, uint16_t addr4, uint16_t addr5, uint16_t addr6, uint16_t addr7)
//{
//    uip_ipaddr_t ipv6_address;
//    uip_ip6addr(&ipv6_address, addr0, addr1, addr2, addr3, addr4, addr5, addr6, addr7);
//    uip_ds6_addr_add(&ipv6_address, 0, ADDR_MANUAL);
//}
/*---------------------------------------------------------------------------*/
//PROCESS_NAME(webserver_nogui_process);
//PROCESS_NAME(udp_server_process);

/*---------------------------------------------------------------------------*/

//static struct etimer reboot_timer;
//
//void cetic_6lbr(void)
//{
////  PROCESS_BEGIN();
//
//  cetic_6lbr_startup = clock_seconds();
//
//#if CONTIKI_TARGET_NATIVE
//  slip_config_handle_arguments(contiki_argc, contiki_argv);
//#endif
//
//  load_nvm_config();
//
//  platform_init();
//
//  process_start(&eth_drv_process, NULL);
//
// // while(!ethernet_ready) {
//  //  PROCESS_PAUSE();
// // }
//
//  process_start(&tcpip_process, NULL);
//
// // PROCESS_PAUSE();
//
//#if CETIC_NODE_INFO
//  node_info_init();
//#endif
//
//  packet_filter_init();
//  cetic_6lbr_init();
//
////#if WEBSERVER
////  process_start(&webserver_nogui_process, NULL);
////#endif
////#if UDPSERVER
////  process_start(&udp_server_process, NULL);
////#endif
//
//  printf("CETIC 6LBR Started\n");
//
//#if CONTIKI_TARGET_NATIVE
//  PROCESS_WAIT_EVENT();
//  etimer_set(&reboot_timer, CLOCK_SECOND);
//  PROCESS_WAIT_EVENT();
//  printf("Exiting...\n");
//  exit(0);
//#endif
//
////  PROCESS_END();
//}
