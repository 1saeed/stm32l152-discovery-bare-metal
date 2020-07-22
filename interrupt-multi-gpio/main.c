/*
 * interrupt-multi-gpio.c
 *
 * Author: 
 * Saeed Poorjandaghi
 *
 * Pinout: 
 * LED Blue 			: PB6
 * LED Green			: PB7
 * USER button		: PA0
 * EXTERNAL input :	PC1
 *
 * Clock:
 * 	HSI RC 16MHz with AHB prescaler 1, cortex system timer divider 1, APB1 and APB2 prescaler 1 --> 16MHz
 *
 * Purpose: 
 * detect falling edge at PA0 (when user button is pushed) and toggle green led
 * detect falling edge at PC1 and toggle blue led
 *
 *
 * Configuration:
 * interrupt init:
 * 1. disable all interrupts
 * 2. enable system configuration controller clock in APB2ENR register
 * 3. select PA0 and PC1 as input source for EXTI0 and EXTI1 in SYSCFG_EXTICR1 register
 * 4. unmask EXTI0 and EXTI1 in EXTI_IMR register
 * 5. unmask event for EXTI0 and EXTI1 in EXTI_EMR register
 * 5. falling edge trigger for EXTI0 and EXTI1 in DXTI_FTSR register (set internal pull-up for pins if necessary)
 * 6. enable interrupt
 * 7. configure PA0 and PC1 as input
 *
 * interrupt service routine:
 * 1. check if EXTI0 or EXTI1 is pending?
 * 2. then clear pending interrupt
 * 3. then do routine tasks
.

 */
 
#include "stm32l1xx.h"                  // Device header
#include <stdio.h>
 
 /* Function declarations */
 void gpio_init(void);
 void gpio_extint_init(void);
 void led_blue(int status);
 void led_blue_toggle(void);
 void led_green(int status);
 void led_green_toggle(void);
 void set_sysclk_to_hsi(void);
 void delay_ms(int delay);
 


 int main(void)
 {  
	 set_sysclk_to_hsi();
	 gpio_extint_init();
	 gpio_init();
 
	 while(1)
	 { 

	 } 
 }
 
 void EXTI0_IRQHandler(void)
 {
	 if(EXTI->PR & 0x00000001)			// check if EXTI0 is pending
	 {
		 EXTI->PR = 0x00000001;				// clear pending interrupt
		 led_green_toggle();
	 } 
 }
 
 void EXTI1_IRQHandler(void)
 {
	 if(EXTI->PR & 0x00000002)			// check if EXTI1 is pending
	 {
		 EXTI->PR = 0x00000002;				// clear pending interrupt
		 led_blue_toggle();
	 } 
 }
 
 void gpio_extint_init(void)
 {
		__disable_irq();																	// disable all interrupts
	 
	  RCC->APB2ENR |= 0x00000001;												// enable clock for system configuration controller 
	  RCC->AHBENR  |= 0x00000001;												// enable clock for PORTA
	  RCC->AHBENR  |= 0x00000004;												// enable clock for PORTC
	 
	  GPIOC->PUPDR |= 0x00000004;												// internally pull-up for PC1
	 
	  SYSCFG->EXTICR[0] &= 0xFFFFFFF0; 									// select PA0 as input for the EXTI0
	  SYSCFG->EXTICR[0] &= 0xFFFFFF0F;									// select PC0 as input for the EXTI0
	  SYSCFG->EXTICR[0] |= 0x00000020;									
	 
	  EXTI->IMR 	|= 0x00000001;													// unmask EXTI0
	  EXTI->EMR		|= 0x00000001;													// unmask event
	  EXTI->FTSR	|= 0x00000001;													// falling edge trigger for EXTI0
	 
	  EXTI->IMR		|= 0x00000002;													// unmask EXTI1
	  EXTI->EMR		|= 0x00000002;													// unmask event
	  EXTI->FTSR  |= 0x00000002;													// falling edge trigger for EXTI1
	 
	  NVIC_EnableIRQ(EXTI0_IRQn);													// enable interrupt
	  NVIC_EnableIRQ(EXTI1_IRQn);													// enable interrupt
		
	  __enable_irq();
 }	 
 
 void gpio_init(void)
 {
	 RCC->AHBENR  |= 0x00000002;			// IO port B clock enabled (bit 1) --> All GPIOs are connected to AHB bus, so the relevant register is RCC_AHBENR 
	 GPIOB->MODER |= 0x00005000;		  // As long as reset value is 0, we just need to write 01 to the relavant section to configure PB6 and PB7 as output 
	 
	 RCC->AHBENR  |= 0x00000001;										// IO port A clock enabled (bit 0) --> All GPIOs are connected to AHB bus, so the relevant register is RCC_AHBENR 
	 GPIOA->MODER &= (unsigned int)~0x00000003;		  // set PA0 as input (0b00)	
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


