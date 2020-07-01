/*
 * timer.c
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
 *  to generate 1Hz timer based delay funtion to toggle LEDs with TIM2
 *
 *
 * Configuration:
 * 1. enable clock for corrsponding buses
 * 2. set timer prescaler and timer overload value 
 * 3. enable timer
 * 4. check for timer overload
 * 
.

 */
 
#include "stm32l1xx.h"                  // Device header
#include <stdio.h>
 
 /* Function declarations */
 void gpio_init(void);
 void tim2_init(void);
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
	 tim2_init();
	 
	 while(1)
	 {
			while(!((TIM2->SR & 0x00000001) == 0x00000001)){}			// wait until timer2 overflow flag is set
			TIM2->SR &= 0xFFFFFFFE;																// clear the overflow flag to enable timer2 to start counting again	
			led_blue_toggle();
			led_green_toggle();  
	 } 
 }
 
 
 void gpio_init(void)
 {
	 RCC->AHBENR |= 0x00000002;			// IO port B clock enabled (bit 1) --> All GPIOs are connected to AHB bus, so the relevant register is RCC_AHBENR 
	 GPIOB->MODER |= 0x00005000;		// As long as reset value is 0, we just need to write 01 to the relavant section to configure PB6 and PB7 as output 
	 
	 RCC->AHBENR |= 0x00000001;										// IO port A clock enabled (bit 0) --> All GPIOs are connected to AHB bus, so the relevant register is RCC_AHBENR 
	 GPIOA->MODER &= (unsigned int)~0x00000003;		// set PA0 as input (0b00)	
 }
 
 
 void tim2_init(void)
 {
	 RCC->APB1ENR |= 0x00000001;		// enable timer2 clock
	 
	 TIM2->PSC = 1600 - 1;					// divided by 1600 @16MHz that comes to 10KHz or 1us 
	 TIM2->ARR = 10000 - 1;					// to generate 10000 x 1us = 1 second
	 TIM2->CNT = 0;									// reset timer2 counter value
	 TIM2->CR1 |= 0x00000001;				// enable timer2
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


