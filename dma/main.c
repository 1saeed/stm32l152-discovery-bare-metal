/*
 * dma.c
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
 * Clock:
 * 	HSI RC 16MHz with AHB prescaler 1, cortex system timer divider 1, APB1 and APB2 prescaler 1 --> 16MHz
 *
 * Purpose:   
 * 256 bytes memory to memory (SRAM) trasnfer using DMA
 *
 *
 * Configuration:
 * 1- set DMA1EN bit in RCC_AHBENR register to enable DMA1 clock
 * 2- set MEM2MEM bit in DMA_CCRx register to enable memory to memory transfer
 * 3- set MINC and PINC bits in DMA_CCRx register to enable address increment mode 
 * 4- set number of transfers in DMA_CNDRTx register
 * 5- set source address in DMA_CPARx register
 * 6- set destination address in DMA_CMARx register
 * 7- set EN bit in DMA_CCRx register to enable DMA
 */
 
#include "stm32l1xx.h"                  // Device header
#include <stdio.h>

 
 /* Function declarations */
 void dma_init(void);
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
 
 #define BUF_SIZE	256	
 
 // source and destination address pointer
	static unsigned char src_addr[BUF_SIZE];
	static unsigned char dst_addr[BUF_SIZE];
 
 
 int main(void)
 { 
	 char str[40];
	 set_sysclk_to_hsi();
	 gpio_init();
	 usart1_Init();
	 
	 // fill source memory with 0..255 and zero out destination memory
	 for(int i = 0; i < BUF_SIZE ; i++)
	 {
			src_addr[i] = 50;//(unsigned char)i;
		  dst_addr[i] = 1;
	 }
	 
	 dma_init();
	 //dummy write
	 usart1_write(0);
	 while(1)
	 {
		    // if transfer is completed
				if(DMA1_Channel1->CNDTR == 0 && (DMA1_Channel1->CCR & 0x00000001))				
				{
					  sprintf(str,"last byte is :%u\r\n",dst_addr[BUF_SIZE - 1]); 
					  usart1_send_string(str);
		        sys_tick_delay_ms(1000);
						//DMA1_Channel1->CCR &= (unsigned int)~0x00000001;						
				}
	 } 
 } 
 
 void dma_init(void)
 {
	  DMA1_Channel1->CCR    = 0;														// disable dma
	  while(DMA1_Channel1->CCR & 0x00000001){}							// wait for dma to be disabled
			
	  RCC->AHBENR 					|= 0x01000000;									// DMA1 clock enabled
	  
	  DMA1_Channel1->CCR  	|= 0x00004000;									// memory to memory enabled
	  DMA1_Channel1->CCR    |= 0x000000C0;									// memory/peripheral increment mode enabled
	    
	  DMA1_Channel1->CNDTR	=  BUF_SIZE;										// number of data to transfer
	  DMA1_Channel1->CPAR   =  (unsigned int)src_addr;			// set source address
	  DMA1_Channel1->CMAR   =  (unsigned int)dst_addr;			// set destination address
	  
	  DMA1_Channel1->CCR		|= 0x00000001;									// enable DMA 
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

 