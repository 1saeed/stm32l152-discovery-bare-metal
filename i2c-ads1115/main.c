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
 * SDA				: PB9
 * SCL				: PB8
 *
 * Clock:
 * 	HSI RC 16MHz with AHB prescaler 1, cortex system timer divider 1, APB1 and APB2 prescaler 1 --> 16MHz
 *
 * Purpose:   
 * to develop i2c driver and drive ADS1115 ic through i2c
 *
 *
 * Configuration:
 * 1. set I2C1EN bit in RCC_APB1ENR register to enable I2C1 clock
 * 2. enable clock for PORTB and set PB8 and PB9 as i2c alternate function
 * 3. set pull-up for PB8 and PB9
 * 4. set SWRST bit in I2C1_CR1 register to reset i2c
 * 5. set FREQ[5:0] bits in I2C1_CR2 register to setup peripheral clock frequency TO 16MHz
 * 6. set TRISE[5:0] bits in I2C1_TRISE register to FREQ+1 (sm mode)
 * 7. set CCR[11:0] bits in I2C1_CCR register to set Fscl = 100KHz
 * 8. set PE bit in I2C1_CR1 register to enable peripheral

 */
 
#include "stm32l1xx.h"                  // Device header
#include <stdio.h>

 
 /* Function declarations */
 void i2c_init(void);
 void i2c_start(void);
 void i2c_stop(void);
 int i2c_read(void);
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
 
 #define _SLAVE_ADD	72			// slave address for ADS1117 if ADDDR is connected to GND (0b1001000)
 
 int main(void)
 {
   unsigned char str[40];	 
	 set_sysclk_to_hsi();
	 gpio_init();	  
	 usart1_Init();        	 
	 i2c_init();
	 
	 while(1)
	 {
		  sprintf(str,"V=%.2f\r\n",(i2c_read()*2.048)/32767);
			usart1_send_string(str);
		  sys_tick_delay_ms(1000);
	 } 
 } 
 
 void i2c_init(void)
 {
		RCC->APB1ENR	|= 0x00200000;			// enable I2C1 clock
	  RCC->AHBENR		|= 0x00000002;			// enable clock for PORTB
	  
	  GPIOB->MODER	|= 0x000A0000;			// PB8 and PB9 as alternate function
	  GPIOB->AFR[1]	|= 0x00000044;			// set PB8 and PB9 as i2c alternate function (AF4)
	  
	  GPIOB->OTYPER 	|= 0x0300;					// output open drain (PB8 , PB9)
	  GPIOB->PUPDR		|= 0x00050000;			// PB8 and PB9 pull-up
	  
	  I2C1->CR1				|= 0x8000;					// i2c reset
	  I2C1->CR1				&= (unsigned int)~0x8000;					
	  
	  I2C1->CR2				|= 0x0010;					// set peripheral clock frequency to 16MHz
	  I2C1->TRISE			|= 0x0011;					// set maximum rise time in sm mode (master mode) ----> FREQ+1 = 3
	 
	  I2C1->CCR				|= 0x0050;					// CCR=0x50 (80) ----> Thigh = Tlow = 80x62.5ns -----> Fscl = 1/(Thigh+Tlow) = 100KHz
	  
	  I2C1->CR1				|= 0x0001;					// enable i2c   
 }
 
 void i2c_start(void)
 {
	  I2C1->CR1	|= 0x0100;									// repeated start generation
		while(!(I2C1->SR1	& 0x00000001)){}		// wait until start condition is generated
 }
 
 void i2c_stop(void)
 {
	  I2C1->CR1	|= 0x0200;									// stop generation
	  while(!(I2C1->SR2 & 0x00000002)){}		// wait until stop condition is detected
 }
 
 int i2c_read(void)
 {
	  unsigned char data[2];
	  int value = 0;
	 
	  while(I2C1->SR2 & 0x00000002){}				// wait until bus is free
			
		i2c_start();

    I2C1->DR	= (_SLAVE_ADD << 1);				// 	(first 7-bit I2C address followed by a low R/W bit, First byte: 0b10010000		
		while(!(I2C1->SR1	& 0x00000002)){}		// wait until end of address transmission (master mode)
		(void)I2C1->SR2;											// reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag
			
		while(!(I2C1->SR1	& 0x00000080)){}		// wait until data register is empty
		I2C1->DR	= 0x01;											// points to Config register
		
		while(!(I2C1->SR1	& 0x00000080)){}		// wait until data register is empty	
    I2C1->DR	= 0x84;											// MSB of the Config register to be written 
			
		while(!(I2C1->SR1	& 0x00000080)){}		// wait until data register is empty	
    I2C1->DR	= 0x83;											// LSB of the Config register to be written	
			
		while(!(I2C1->SR1	& 0x00000080)){}		// wait until data register is empty
						
    i2c_start();
    
    I2C1->DR	= (_SLAVE_ADD << 1);				// 	(first 7-bit I2C address followed by a low R/W bit, First byte: 0b10010000		
		while(!(I2C1->SR1	& 0x00000002)){}		// wait until end of address transmission (master mode)
		(void)I2C1->SR2;											// reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag
    
    while(!(I2C1->SR1	& 0x00000080)){}		// wait until data register is empty
		I2C1->DR	= 0x00;											// (points to Conversion register	

    while(!(I2C1->SR1	& 0x00000080)){}		// wait until data register is empty
						
    i2c_start();

    I2C1->DR	= ((_SLAVE_ADD << 1) | 1);		// first 7-bit I2C address followed by a high R/W bit	
    while(!(I2C1->SR1	& 0x00000002)){}			// wait until end of address transmission (master mode)
		(void)I2C1->SR2;												// reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag
		
		i2c_stop();	
    while(!(I2C1->SR1 & 0x0040)){}					// wait until data register not empty

    data[0] = (unsigned char)I2C1->DR;			// the ADS111x response with the MSB of the Conversion register
    data[1] = (unsigned char)I2C1->DR;			// the ADS111x response with the LSB of the Conversion register
     
		value = ((int)data[0] << 8) + data[1];	
    return value;			
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

 