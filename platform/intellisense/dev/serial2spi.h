/*
 * This is an example of how to write a network device driver ("packet
 * driver") for Contiki.
 */

#ifndef __SERIAL2SPI_H__
#define __SERIAL2SPI_H__

/*
 * We include the "contiki.h" file to get the macro and typedef used below.
 */
#define Total_Elements 50
#define DELAY_TIME 253
#define BUFFER_SIZE 128

void serial2spi_init(void);
unsigned long SPI_Data_check(void);
unsigned char SPI_Rx_Data_check(unsigned char data);
int WSN_Deselect_Channel (unsigned long Channel);
int WSN_Select_Channel (unsigned long Channel);
unsigned long Next_Channel(unsigned long Channel);
void wsn_delay(void);
void Channel_Recieve(unsigned long *Data,unsigned long Channel);
int WSN_Send_Command(unsigned char* Data,int lenth);
PROCESS_NAME(spi_poll);

#endif /* __EXAMPLE_PACKET_DRV_H__ */
