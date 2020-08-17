/*
 * adc-scan.c
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
 * read analog channel PA4 and PA1 in scan mode (not continuous) without using the DMA and send voltage values to serial port.
 *
 *
 * Configuration:
 * init:
 * 1. enable clock for ADC1 in APB2ENR register
 * 2. enable clock for PORTA in AHBENR
 * 3. set PA4 and PA1 as analog pin
 * 4. disable adc
 * 5. set EOCS bit in ADC_CR2 register so that the EOC bit is set at the end of each regular conversion (and not each sequence)
 * 6. set SCAN bit in ADC_CR1 register
 * 7. set channel sequence (channel #4 and channel #1) in ADC_SQR5 register
 * 8. set number of conversions (2) in ADC_SQR1 register
 * 9. set proper delay between sequences by setting DELS bit in ADC_CR2 register 
 * 10.set proper sample time for both channels by setting SMPx bits in ADC_SMPR3 register
 * 11.enable ADC interrupt by setting EOCIE bit ADC_CR1 register so that when EOC bit is set an interrupt will be generated 
 * 12. enable ADC
 * 
 * conversion:
 * 1. check ADONS bit in ADC_CR register to check if ADC is ready. 
 * 2. start convertion by setting SWSTART bit ADC_CR2 register.
 * 3. check if EOC bit in ADC_CR is set (conversion completed)
 * 4. read ADC_DR for conversion result
 */
 
#include "stm32l1xx.h"                  // Device header
 
 /* Function declarations */
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
 
 volatile unsigned int adc_value[10];
 volatile unsigned int count = 0;
 volatile unsigned char adc_convertion_flag = 0;
 
 int main(void)
 {
	 unsigned int adc_result = 0;
	 char str[40];
	 set_sysclk_to_hsi();
	 gpio_init();
	 usart1_Init();
	 adc_init();
	 
	 while(1)
	 {
		  while(!((ADC1->SR & 0x00000040) == 0x00000040)){}								// wait until the ADC is ready to covert
	    ADC1->CR2   |= 0x40000000;																			// starts conversion of regular channels 
			while(adc_convertion_flag < 2){};															// wait until conversion is done
				

		  sprintf(str,"adc value1 = %1.2f / adc value2 = %1.2f\r\n",(float)adc_value[0]*3.00/4092,(float)adc_value[1]*3.00/4092);
		  usart1_send_string(str);
			adc_convertion_flag = 0;	
		  sys_tick_delay_ms(1000);
				
	 } 
 }
 
 unsigned int adc_conversion(void)
 {
	  unsigned int value = 0;

	  while(!((ADC1->SR & 0x00000002) == 0x00000002)){}								// wait until regular channel end of conversion
			
		value = ADC1->DR;
			
		return value;	
 }
 
 void ADC1_IRQHandler(void)
 {
	 if((ADC1->SR & 0x00000002) == 0x00000002)			// check if EOC bit is set
	 {
		  adc_value[count] = ADC1->DR;								// read ADC conversion result
			count++;
      if(count == 2)
			{
					count = 0;	
			}	
			adc_convertion_flag++;
	 }
 }
 
 void adc_init(void)
 {
		RCC->APB2ENR |= 0x00000200;									// ADC1 interface clock enabled
	  RCC->AHBENR  |= 0x00000001;									// enable clock for AHBENR bus (bit 1) --> GPIOA 
	 
	  GPIOA->MODER |= 0x00000300;									// PA4 and PA1 as analog mode
	  
	  ADC1->CR2 	 = 0x00000000;									// disable ADC 
	 
	  ADC1->CR2   |= 0x00000400;									// the EOC bit is set at the end of each regular conversion
	  ADC1->CR1		|= 0x00000100;									// scan mode enabled
	  ADC1->SQR5 	|= 0x00000024;									// 1st conversion in regular sequence for ADC channel #4 and 2nd conversion for channel #1
	  ADC1->SQR1   = 0x00100000;									// 2 conversions
	  ADC1->CR2   |= 0x00000070;									// 255 APB clock cycles after the end of conversion
	  ADC1->SMPR3	|= 0x00007038;									// channel 1 and 4 sample time selection : 384 cycle
	    
	  __disable_irq();											// disable all interrupts
	  ADC1->CR1   |= 0x00000020;						// EOC interrupt enabled. An interrupt is generated when the EOC bit is set.
	  NVIC_EnableIRQ(ADC1_IRQn);					  // enable ADC interrupt
	  __enable_irq();
	 
	  ADC1->CR2 	|= 0x00000001;									// enable ADC  
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

 