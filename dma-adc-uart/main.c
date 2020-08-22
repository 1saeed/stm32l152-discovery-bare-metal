/*
 * dma-adc-uart.c
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
 * ADC				: PA4, PA1
 *
 * Clock:
 * 	HSI RC 16MHz with AHB prescaler 1, cortex system timer divider 1, APB1 and APB2 prescaler 1 --> 16MHz
 *
 * Purpose:   
 * read analog channel PA4 and PA1 in scan mode (continuous) using DMA and send voltage values to serial port using also DMA
 *
 *
 * Configuration:
 * dma for adc:
 * 1- set DMA1EN bit in RCC_AHBENR register to enable DMA1 clock
 * 2- set MINC bit in DMA_CCRx register to enable address increment mode 
 * 3- set CIRC bit in DMA_CCRx register to enable DMA circular mode
 * 4- set PSIZE[1:0] and MSIZE[1:0] to "01" as 16-bits data size transfer
 * 5- set number of transfers in DMA_CNDRTx register 
 * 6- set destination address in DMA_CPARx register for result array
 * 7- set source address in DMA_CMARx register for ADC1->DR
 * 8- set EN bit in DMA_CCRx register to enable DMA
 *
 * dma for uart:
 * 1- set DMA1EN bit in RCC_AHBENR register to enable DMA1 clock
 * 2- set DIR bit in DMA_CCRx register to enable read from memory.
 * 3- set MINC bit in DMA_CCRx register to enable address increment mode 
 * 4- set number of transfers in DMA_CNDRTx register 
 * 5- set destination address in DMA_CPARx register for USART1->DR
 * 6- set source address in DMA_CMARx register for result array
 * 7- set EN bit in DMA_CCRx register to enable DMA 
 *
 * uart:
 * 1- set up UART
 * 2- set DMAT bit in USART_CR3 register to enable DMA for TX
 *
 * adc:
 * 1- setup ADC in continuous conversion and scan mode
 * 2- set DMA bit in ADC_CR2 register to enable DMA mode
 * 3- set DDS bit in ADC_CR2 register to to let DMA requests to be issued as long as data are converted and DMA=1 

 
#include "stm32l1xx.h"                  // Device header
 
 /* Function declarations */
 void dma_uart_init(void);
 void dma_adc_init(void);
 void adc_init(void);
 unsigned int adc_conversion(void);
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
 
 volatile unsigned short adc_value[2] = {1023,1023};

 
 int main(void)
 {
	 set_sysclk_to_hsi();
	 gpio_init();
	 usart1_Init();
	 
	 dma_adc_init();
	 adc_init();
	 dma_uart_init();
	 while(1)
	 {
				// if transfer is completed
				if(DMA1_Channel4->CNDTR == 0 && (DMA1_Channel4->CCR & 0x00000001))				
				{					
					  DMA1_Channel4->CCR		&= (unsigned int)~0x00000001;		// disable dma
						while(DMA1_Channel4->CCR & 0x00000001){}							// wait for dma to be disabled
							 
						DMA1_Channel4->CNDTR	=  4;														// number of data to transfer								
						DMA1_Channel4->CCR		|= 0x00000001;									// enable DMA
							
		        sys_tick_delay_ms(1000);										
				}
				
	 } 
 }
 
 void dma_adc_init(void)
 {
		DMA1_Channel1->CCR    = 0;														// disable dma
	  while(DMA1_Channel1->CCR & 0x00000001){}							// wait for dma to be disabled
			
	  RCC->AHBENR 					|= 0x01000000;									// DMA1 clock enabled
	  
	  DMA1_Channel1->CCR    |= 0x00000080;									// memory increment mode enabled
		DMA1_Channel1->CCR    |= 0x00000020;									// circular mode enabled
		DMA1_Channel1->CCR    |= 0x00000500;									// set peripheral and memory size to 16 bits
	    
	  DMA1_Channel1->CNDTR	=  2;														// number of data to transfer
	  DMA1_Channel1->CPAR   =  (unsigned int)&(ADC1->DR);		// set source address
	  DMA1_Channel1->CMAR   =  (unsigned int)adc_value;			// set destination address
	  
	  DMA1_Channel1->CCR		|= 0x00000001;									// enable DMA 		
 }	 
 
 void dma_uart_init(void)
 {
	  DMA1_Channel4->CCR    = 0;														// disable dma
	  while(DMA1_Channel4->CCR & 0x00000001){}							// wait for dma to be disabled
			
	  RCC->AHBENR 					|= 0x01000000;									// DMA1 clock enabled
	  
	  DMA1_Channel4->CCR    |= 0x00000080;									// memory increment mode enabled
		DMA1_Channel4->CCR    |= 0x00000010;									// Read from memory	
	    
	  DMA1_Channel4->CNDTR	=  4;														// number of data to transfer
	  DMA1_Channel4->CPAR   =  (unsigned int)&(USART1->DR);	// set destination address
	  DMA1_Channel4->CMAR   =  (unsigned int)adc_value;			// set source address
	  
	  DMA1_Channel4->CCR		|= 0x00000001;									// enable DMA 
 }
 
 unsigned int adc_conversion(void)
 {
	  unsigned int value = 0;

	  while(!((ADC1->SR & 0x00000002) == 0x00000002)){}								// wait until regular channel end of conversion
			
		value = ADC1->DR;
			
		return value;	
 }
 

 void adc_init(void)
 {
		RCC->APB2ENR |= 0x00000200;									// ADC1 interface clock enabled
	  RCC->AHBENR  |= 0x00000001;									// enable clock for AHBENR bus (bit 1) --> GPIOA 
	 
	  GPIOA->MODER |= 0x00000300;									// PA4 and PA1 as analog mode
	  
	  ADC1->CR2 	 = 0x00000000;									// disable ADC 
	 
	  ADC1->CR2		|= 0x00000002;									// continuous conversion mode
	  ADC1->CR1		|= 0x00000100;									// scan mode enabled
	  ADC1->SQR5 	|= 0x00000024;									// 1st conversion in regular sequence for ADC channel #4 and 2nd conversion for channel #1
	  ADC1->SQR1   = 0x00100000;									// 2 conversions
	  ADC1->CR2   |= 0x00000070;									// 255 APB clock cycles after the end of conversion
	  ADC1->SMPR3	|= 0x00007038;									// channel 1 and 4 sample time selection : 384 cycle
	 
	  ADC1->CR2		|= 0x00000100;									// DMA mode enabled
	  ADC1->CR2		|= 0x00000200;									// DMA requests are issued as long as data are converted and DMA=1
	    	 
	  ADC1->CR2 	|= 0x00000001;									// enable ADC 
    
    while(!((ADC1->SR & 0x00000040) == 0x00000040)){}								// wait until the ADC is ready to covert
	  ADC1->CR2   |= 0x40000000;																			// starts conversion of regular channels 		
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

 