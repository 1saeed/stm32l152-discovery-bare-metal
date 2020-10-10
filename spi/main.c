/*
 * i2c-ads1115.c
 *
 * Author: 
 * Saeed Poorjandaghi
 *
 * Pinout: 
 * TX					: PA9
 * RX					: PA10
 * LED Blue 	: PB6
 * LED Green	: PB7
 * USER button: PA0
 *
 * SPI2:
 * SS 				: PB12
 * SCK				: PB13
 * MISO				: PB14
 * MOSI				: PB15
 *
 *
 * Clock:
 * 	HSI RC 16MHz with AHB prescaler 1, cortex system timer divider 1, APB1 and APB2 prescaler 1 --> 16MHz
 *
 * Purpose:   
 * to send a byte through spi with CPHA:1 and CPOL:0
 *
 *
 * Configuration:
 * init:
 * 1. enable clock for GPIOB
 * 2. set SPI2EN bit in RCC_APB1ENR register to enable clock for SPI2
 * 3. configure GPIOB_MODER register so that PB13,14 and 15 set as alternate function
 * 3. configure GPIOB_AFR[1] register for PB13,14 and 15 as AF5 to select as SPI2 alternate function
 * 4. configure PB12 (SS) as output
 * 5. configure SPI_CR1 register as CPHA:1, MASTER:1, SSM:1, SSI:1, BR[2:0]:011
 * 6. set SPE bit in SPI_CR1 register to enable spi module
 *
 * write:
 * 1. check TXE bit in SPI_SR register and wait until it becomes 1 so Tx buffer is empty
 * 2. select slave device by reseting SS line to 0
 * 3. write data to SPI_DR register
 * 4. check BSY bit in SPI_SR register and wait until it becomes 0 so spi is not busy in communication or Tx buffer is empty
 * 5. de-select slave device by seting SS line to 1
 */
 
#include "stm32l1xx.h"                  // Device header
#include <stdio.h>

 
 /* Function declarations */
 void spi_init(void);
 void spi_write(unsigned char data);
 void usart1_Init(void);
 void usart1_write(char ch);
 void usart1_send_string(char * str);
 char usart1_read(void);
 void gpio_init(void);
 void led_blue(int status);
 void led_green(int status);
 void set_sysclk_to_hsi(void);
 void sys_tick_delay_ms(int ms);
 void delay_ms(int delay);
 
 
 int main(void)
 {
   unsigned char str[40];	 
	 set_sysclk_to_hsi();
	 gpio_init();	  
	 usart1_Init();        	 
	 spi_init();
	 
	 while(1)
	 {
		  spi_write(0xAA);
		  sys_tick_delay_ms(100);
	 } 
 } 
 
 void spi_init(void)
 {
	 RCC->AHBENR		|= 0x00000002;			// enable clock for GPIOB
	 RCC->APB1ENR		|= 0x00004000;			// enable clock for SPI2
	 
	 GPIOB->MODER		|= 0xA8000000;			// alternate function mode
	 GPIOB->AFR[1] 	|= 0x55500000;			// alternate function selection as AF5 (SPI2)
	 
	 GPIOB->MODER		|= 0x01000000;			// set PB12 (SS) as output
	 
	 SPI2->CR1			|= 0x031D;					// configure SPI communication
	 SPI2->CR1			|= 0x0040;					// enable spi module
 }
 
 void spi_write(unsigned char data)
 {
    while(!(SPI2->SR & 0x0002)){}			// wait untill transfer buffer is empty
	  GPIOB->BSRR		|= 	0x10000000;			// reset SS line to select slave device
		SPI2->DR			 = data;	
		while(SPI2->SR	& 0x0080){}				// wait until SPI is not busy in communication or Tx buffer is empty	
		GPIOB->BSRR		|= 	0x00001000;			// set SS line to de-select slave device
 }	 
 
 void USART1_IRQHandler(void)
 {
	 char c;
	 if(USART1->SR & 0x00000020)			// check if received data is ready to be read.
	 {
			c =  (char)USART1->DR;
		  usart1_write(c);							// echo
	 }
 }
 
 void usart1_Init(void)
{
	RCC->APB2ENR |= 0x00004000;		// enable clock for APB2ENR bus (bit 14) --> 	USART1
	RCC->AHBENR |= 0x00000001;		// enable clock for AHBENR bus (bit 1) --> GPIOA 
	
	GPIOA->AFR[1] |= 0x00000770;	// alternate selection for PA9 and PA10 as usart
	GPIOA->MODER |= 0x00280000; 	// alternate function mode for PA9 and PA10 
	
  USART1->BRR = 0x0683;									// baudrate 9600 @ 16Mhz	
	USART1->CR1 = 0x0000200C;							// enable UE and TE/RE 
	
	__disable_irq();											// disable all interrupts
	USART1->CR1 |= 0x00000020;						// an USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_SR register 
	NVIC_EnableIRQ(USART1_IRQn);					// enable USART1 interrup
	__enable_irq();
	
	USART1->CR3 |= 0x00000080;										// DMA mode is enabled for transmission
}
 
void usart1_write(char ch)
{
	//while(!(USART1->SR & 0x0080)){}	//wait while TX buffer is empty
	USART1->DR = ch & 0xFF;
  while(!(USART1->SR & 0x0040)){}		// wait until transmission is completed
		
}

void usart1_send_string(char * str)
{
	unsigned int i = 0;
	while(str[i] != '\0')
	{
		usart1_write(str[i]);
		i++;
	}
}

char usart1_read(void)
{
	while(!((USART1->SR & 0x00000020) == 0x00000020)){}	// wait until received data is ready to be read
		
	return (char)USART1->DR;		
}

 void gpio_init(void)
 {
	 RCC->AHBENR |= 0x00000002;			// IO port B clock enabled (bit 1) --> All GPIOs are connected to AHB bus, so the relevant register is RCC_AHBENR 
	 GPIOB->MODER |= 0x00005000;		// As long as reset value is 0, we just need to write 01 to the relavant section to configure PB6 and PB7 as output 
	 
	 RCC->AHBENR |= 0x00000001;										// IO port A clock enabled (bit 0) --> All GPIOs are connected to AHB bus, so the relevant register is RCC_AHBENR 
	 GPIOA->MODER &= (unsigned int)~0x00000003;		// set PA0 as input (0b00)	
 }
 
 void led_blue(int status)
 {
	 if(status)										// Turn on LED blue PB6
	 {
		 GPIOB->BSRR = 0x00000040;				// Atomic sets the corresponding ODRB bit 
	   //GPIOB->ODR |= 0x00000040;			// Sets ODR6 bit	
	 }
	 else													// Turn off LED blue PB6
	 {
		 GPIOB->BSRR = 0x00400000;				// Atomic resets the corresponding ODRB bit
		 //GPIOB->ODR &= ~0x00000040;			// Resets ODR6 bit
	 }
 }
 
 void led_green(int status)
 {
	 if(status)										// Turn on LED green PB7
	 {
		 GPIOB->BSRR = 0x00000080;				// Atomic sets the corresponding ODRB bit 
	   //GPIOB->ODR |= 0x00000080;			// Sets ODR7 bit	
	 }
	 else													// Turn off LED green PB7
	 {
		 GPIOB->BSRR = 0x00800000;				// Atomic resets the corresponding ODRB bit
		 //GPIOB->ODR &= ~0x00000080;			// Resets ODR7 bit
	 }
 }

 void set_sysclk_to_hsi(void)
 {
	 RCC->CR &= 0x8EFAFEFE;		// clear all rw bits
	 RCC->CFGR &= 0x8802C00C;	// clear all rw bits
	 
	 RCC->CR |= 0x00000001;		// HSI oscillator ON
	 RCC->CFGR |= 0x00000001;	// HSI oscillator used as system clock
	 
	 while(!((RCC->CR & 0x00000002) == 0x00000002)){}	// waitn until HSI oscillator is ready
	 while(!((RCC->CFGR & 0x0000000C) == 0x04)){}			// wait until HSI is used as system clock 	 
	 
	 SystemCoreClockUpdate();
 }
 
 
 void sys_tick_delay_ms(int ms)
 {
	 SysTick->LOAD = 16000 - 1;		// @16MHz to generate 1 milisecond delay
	 SysTick->VAL = 0;						// clear current value register
	 SysTick->CTRL = 0x00000005;	// enable systick and choose internal clock as clock source
	 
	 for(int i = 0 ; i < ms ; i++)
	 {
		 // wait until the count flag is set (1 milisecond)
		 while(!((SysTick->CTRL & 0x00010000) == 0x00010000)){} 
	 }
	 SysTick->CTRL = 0; 		// stop the timer
 }
 
 // that will change the speed based on the optimization settings
void delay_ms(int delay)
{
	volatile int i;
	int j;
	for(j=delay;j>0;j--)
	{
		for(i=0;i<500;i++);
	}
}

 