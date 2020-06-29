/*
 * sysTick-delay.c
 *
 * Author: 
 * Saeed Poorjandaghi
 *
 * Pinout: 
 * LED Blue : PB6
 * LED Green: PB7
 * USER button: PA0
 *
 * Clock:
 * 	HSI RC 16MHz with AHB prescaler 1, cortex system timer divider 1, APB1 and APB2 prescaler 1 --> 16MHz
 *
 * Purpose:   
 * to use percise and user defined system delay using sysTick
 *
 *
 * Configuration:
 * 1. Set the SysTick reloud value register based on desired delay @16MHz clock.
 * 2. Clear the SysTick current value register.
 * 3. (optional) set the calibration register if needed.    
 * 4. Program control and status register. 
 * 5. Check control and status register for count flag to be set. 
.

 */
 
#include "stm32l1xx.h"                  // Device header
#include <stdio.h>
 
 /* Function declarations */
 void gpio_init(void);
 void sys_tick_delay_ms(int ms);
 void led_blue(int status);
 void led_blue_toggle(void);
 void led_green(int status);
 void led_green_toggle(void);
 void set_sysclk_to_hsi(void);
 void delay_ms(int delay);
 
 int main(void)
 {	 
	 set_sysclk_to_hsi();
	 gpio_init();
 
	 while(1)
	 {		
			led_blue_toggle();
			led_green_toggle();
		  sys_tick_delay_ms(1000);
	 } 
 }
 
 
 void gpio_init(void)
 {
	 RCC->AHBENR |= 0x00000002;			// IO port B clock enabled (bit 1) --> All GPIOs are connected to AHB bus, so the relevant register is RCC_AHBENR 
	 GPIOB->MODER |= 0x00005000;		// As long as reset value is 0, we just need to write 01 to the relavant section to configure PB6 and PB7 as output 
	 
	 RCC->AHBENR |= 0x00000001;										// IO port A clock enabled (bit 0) --> All GPIOs are connected to AHB bus, so the relevant register is RCC_AHBENR 
	 GPIOA->MODER &= (unsigned int)~0x00000003;		// set PA0 as input (0b00)	
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
	 GPIOB->ODR ^= 0x00000040;
//	 if((GPIOB->ODR & 0x00000040) == 0x00000040)
//		 led_blue(0);
//	 else
//		 led_blue(1);
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
	 GPIOB->ODR ^= 0x00000080;
//	 if((GPIOB->ODR & 0x00000080) == 0x00000080)
//		 led_green(0);
//	 else
//		 led_green(1);
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


