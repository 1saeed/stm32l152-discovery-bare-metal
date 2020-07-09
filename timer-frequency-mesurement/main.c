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
 * Output compare : PC6 (TIM3_CH1)
 * Input capture: PA15 (TIM2_CH1)
 *
 * Clock:
 * 	HSI RC 16MHz with AHB prescaler 1, cortex system timer divider 1, APB1 and APB2 prescaler 1 --> 16MHz
 *
 * Purpose:   
 *  to generate 1Hz compare output toggle on PC6 (TIM3_CH1)
 *  and then capture this signal on PA15 (TIM2_CH1) on rising edge
 *  add watch a variable to monitor the time stamp between two rising edges
 *
 *
 * Configuration:
 * compare/output mode:
 * 	1. select alternate funtion mode for PC6
 * 	2. enable clock for corrsponding buses
 * 	3. set timer prescaler and timer overload value 
 * 	4. enable toggle on compare match
 * 	5. enable timer
 *
 * input capture/input:
 *	1. select alternate function mode for PA15
 *  2. enable clock for corresponding buses
 *	3. set timer2 prescaler to watch time stamp
 *	4. set CH1 to capture rising edges
 *  5. enable capture and timer 
.

 */
 
#include "stm32l1xx.h"                  // Device header
#include <stdio.h>
 
 /* Function declarations */
 void gpio_init(void);
 void tim3_compare_mode_init(void);
 void tim2_input_capture_init(void);
 void led_blue(int status);
 void led_blue_toggle(void);
 void led_green(int status);
 void led_green_toggle(void);
 void set_sysclk_to_hsi(void);
 void delay_ms(int delay);
 
 volatile unsigned int period = 0;
 volatile float frequency = 0.00;

 int main(void)
 {  
   unsigned int lastValue = 0;
	 unsigned int currentValue = 0;	
	 set_sysclk_to_hsi();
	 gpio_init();
	 tim3_compare_mode_init();
	 tim2_input_capture_init();
	 
	 while(1)
	 {
			while(!((TIM2->SR & 0x00000002) == 0x00000002)){}			// wait until an edge has been detected on PA15 (TIM2_CH1) which matches the selected polarity
					currentValue = TIM2->CCR1;											  // CCR1 is the counter value transferred by the last input capture 1 event (IC1).
				period = currentValue - lastValue;
				lastValue = currentValue;
				frequency = 1000.0f/(float)period;
	 } 
 }
 
 
 void gpio_init(void)
 {
	 RCC->AHBENR  |= 0x00000002;			// IO port B clock enabled (bit 1) --> All GPIOs are connected to AHB bus, so the relevant register is RCC_AHBENR 
	 GPIOB->MODER |= 0x00005000;		  // As long as reset value is 0, we just need to write 01 to the relavant section to configure PB6 and PB7 as output 
	 
	 RCC->AHBENR  |= 0x00000001;										// IO port A clock enabled (bit 0) --> All GPIOs are connected to AHB bus, so the relevant register is RCC_AHBENR 
	 GPIOA->MODER &= (unsigned int)~0x00000003;		  // set PA0 as input (0b00)	
 }
 
 
 void tim3_compare_mode_init(void)
 { 
	 RCC->AHBENR   |= 0x00000004;				// enable clock for PORTC
	 GPIOC->MODER  |= 0x00002000;			  // alternate function mode for PC6
	 GPIOC->AFR[0] |= 0x02000000;			  // alternate function selection for timer3
	 
	 RCC->APB1ENR  |= 0x00000002;			  // enable timer3 clock
	 TIM3->PSC      = 1600 - 1;				  // 10KHz @16MHz
	 TIM3->ARR      = 10000 - 1;				// 1Hz 
	 
	 TIM3->CCMR1    = 0x0030;						// toggle - OC1REF toggles when TIMx_CNT=TIMx_CCR1
	 TIM3->CCR1     = 0;								// set match mode for ch1
	 TIM3->CCER		 |= 0x00000001;				// compare enable ch1
	 
	 TIM3->CNT			= 0;								// reset timer counter
	 TIM3->CR1		 |= 0x00000001;				// enable timer3	
	 
 }
 
 void tim2_input_capture_init(void)
 {
	 RCC->AHBENR   |= 0x00000001;				// enable4 clock for PORTA
	 GPIOA->MODER  |= 0x80000000; 			// set PA15 as alternate function
	 GPIOA->AFR[1] &= 0x0FFFFFFF;
	 GPIOA->AFR[1] |= 0x10000000;		    // route PA15 for timer2
	 
	 //configure timer2 ch1 for input capture mode
	 RCC->APB1ENR  |= 0x00000001;				// enable clock for timer2
	 TIM2->PSC     = 16000 - 1;					// divided by 16000 --> 1KHz @16MHz
	 TIM2->CCMR1   = 0x00000041;				// set ch1 to capture at every edge and enable filter (fSAMPLING=fDTS/2, N=6)
	 TIM2->CCER    = 0x0000000B;				// capture enabled and polarity for trigger or capture operations (noninverted/both edges)
	 TIM2->CR1     = 0x00000001;				// enable timer2	 
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


