/*
 * button.c
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
 * (default) MSI RC 2097 KHz
 *
 * Purpose:   
 *    To toggle green and blue LEDs by preesing user button.
 *
 *
 * Configuration:
 *   1. discover the relevant bus and enable GPIOx clock from RCC register
 *   2. set the direction of LED pins from MODER to output and button pin to input mode
 *   3. (optional) set the speed of the pins from OSPEEDR
 *   4. (optional) set pins to pull-up or pull-down or
 *         leave them floating from PUPDR
 *   5. (optional) set output type register to push-pull or
 *         open-drain from OTYPER
 *   6. read IDR to check if button is pressed
 *   7. write to ODR or BSRR 
 */
 
#include "stm32l1xx.h"                  // Device header
 
 /* Function declarations */
 void GPIO_init(void);
 void led_blue(int status);
 void led_green(int status);

 
 int main(void)
 {
	 GPIO_init();
	 
	 while(1)
	 {
		 if(GPIOA->IDR & 0x00000001)		// Check if bit0 (PA0) is pressed
		 {
			 led_blue(1);
			 led_green(1);
		 }
		 else
		 {
			 led_blue(0);
			 led_green(0);
		 }
			 
	 }
 }
 
 void GPIO_init(void)
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
 


 