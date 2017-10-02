
#ifndef __NRF24L01_H__
#define __NRF24L01_H__


#include "em_usart.h"
#include "em_device.h"
#include "em_chip.h"
//#include "spidrv.h"




	#define Nrf24l01_HI_SPI_FREQ     8000000

	#define Nrf24l01_LO_SPI_FREQ     1000000
	#define Nrf24l01_USART           USART0

	#define Nrf24l01_CMUCLOCK        cmuClock_USART0
	#define Nrf24l01_GPIOPORT        gpioPortC
	#define Nrf24l01_MOSIPIN         0
	#define Nrf24l01_MOSI_LOC        USART_ROUTE_LOCATION_LOC0
	#define Nrf24l01_MISOPIN         1
	#define Nrf24l01_MISO_LOC        USART_ROUTE_LOCATION_LOC0
	#define Nrf24l01_CSPIN           2
	#define Nrf24l01_CLKPIN          14
	#define Nrf24l01_CLK_LOC         USART_ROUTE_LOCATION_LOC3
	#define Nrf24l01_CEPIN           3
	
	

	
void Nrf24l01_init(void);
void Nrf24l01_transmit(uint8_t* data,uint8_t size);
void Nrf24l01_recv(void);
uint8_t NRF_Write_reg(uint8_t add,uint8_t* data,uint8_t size);
uint8_t NRF_Read_reg(uint8_t add);
void Config_transmitter(void);
void Config_reciever(void);
void delay_ms(uint16_t ms);	
void delay_(void);
void multi_read(uint8_t reg,uint8_t* p,uint8_t size);

		


#endif