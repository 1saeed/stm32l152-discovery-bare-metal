/*
 * uart-console.c
 *
 * Author: 
 * Saeed Poorjandaghi
 *
 * Pinout: 
 * TX: PA9
 * RX: PA10
 * LED Blue : PB6
 * LED Green: PB7
 * USER button: PA0
 *
 * Clock:
 * 	HSI RC 16MHz with AHB prescaler 1, cortex system timer divider 1, APB1 and APB2 prescaler 1 --> 16MHz
 *
 * Purpose:   
 * to have serial interaction console (9600, 8 bit data size, no parity and handshake off)
 *
 * Note : 
 * Compiler version 6 or higher find redifinition of struct __FILE as error.
 * Use standard serial console terminal like Tera Term.
 *
 * Configuration:
 * 1. Redefine low-level library functions to enable
 *    direct use of high-level library functions.
 *
 * 
.

 */
 
#include "stm32l1xx.h"                  // Device header
#include <stdio.h>
 
 /* Function declarations */
 void usart1_Init(void);
 int usart1_write(char ch);
 char usart1_read(void);
 void gpio_init(void);
 void led_blue(int status);
 void led_blue_toggle(void);
 void led_green(int status);
 void led_green_toggle(void);
 void set_sysclk_to_hsi(void);
 void delay_ms(int delay);
 
 int main(void)
 {
	 char c;
	 char str[100];
	 
	 set_sysclk_to_hsi();
	 gpio_init();
	 usart1_Init();
	 
	 printf(" ");		// dummy send
	 printf("Hello stranger\n");
	 fprintf(stdout,"This is stdout\r\n");
	 fprintf(stderr,"and this is stderr\r\n");
	 
	 while(1)
	 {
			printf("Which LED do you want me to toggle? (type B for blue or G for green)\r\n");
		  scanf("%c",&c);
		  printf("You choose %c \r\n",c);
		  if(c == 'B' || c == 'b')
			{
				led_blue_toggle();
			}
			else if(c == 'G' || c == 'g')
			{
				led_green_toggle();
			}
		  printf("Now enter your name: ");
		  gets(str);
		  printf("You are doing great ");
		  puts(str);
		  printf("\r\n");
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
	
}
 
int usart1_write(char ch)
{
	//while(!(USART1->SR & 0x0080)){}	//wait while TX buffer is empty
	USART1->DR = ch & 0xFF;
  while(!(USART1->SR & 0x0040)){}		// wait until transmission is completed
		
	return ch;	
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
 
 void led_blue_toggle(void)
 {
	 if((GPIOB->ODR & 0x00000040) == 0x00000040)
		 led_blue(0);
	 else
		 led_blue(1);
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
 
 void led_green_toggle(void)
 {
	 if((GPIOB->ODR & 0x00000080) == 0x00000080)
		 led_green(0);
	 else
		 led_green(1);
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

// Check if compiler version 4/5 is selected in target option
#if   defined ( __CC_ARM )		
struct __FILE
{
	int handle;
};
#endif


FILE __stdin  = {0};
FILE __stdout = {1};
FILE __stderr = {2};

int fgetc(FILE *f)
{
	char c;
	c = usart1_read();
	if(c == '\r')				// enter
	{
		usart1_write(c);
		c = '\n';
	}
	usart1_write(c);
}

int fputc(int c, FILE *f)
{
	return usart1_write((char)c);
}