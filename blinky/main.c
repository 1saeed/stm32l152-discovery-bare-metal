/*
 * blinky.c
 *
 * Author: 
 * Saeed Poorjandaghi
 *
 * Pinout: 
 * LED Blue : PB6
 * LED Green: PB7
 *
 * Clock:
 * (default) MSI RC 2097 KHz
 *
 * Purpose:   
 *    To blink green and blue LEDs about every 1 second.
 *
 * Note:
 *    Delay function is just a simple dummy loop so is not
 *    accurate and the delay time will change based on the
 *    optimization flags. Becareful that the arm compiler
 *    6 or higher will remove the timer delay code, unless
 *    the variable is declared as volatile.
 *
 * Configuration:
 *   1. discover the relevant bus and enable GPIOx clock from RCC register
 *   2. set the direction of the pins from MODER to output mode
 *   3. (optional) set the speed of the pins from OSPEEDR
 *   4. (optional) set pins to pull-up or pull-down or
 *         leave them floating from PUPDR
 *   5. (optional) set output type register to push-pull or
 *         open-drain from OTYPER
 *   6. write to ODR or BSRR 
 */
 
#include "stm32l1xx.h"                  // Device header
 
 /* Function declarations */
 void GPIO_init(void);
 void led_blue(int status);
 void led_green(int status);
 void delay_ms(int delay);
 
 int main(void)
 {
	 GPIO_init();
	 
	 while(1)
	 {
		 led_blue(1);
		 led_green(1);
		 
		 delay_ms(1000);
		 
		 led_blue(0);
		 led_green(0);
		 
		 delay_ms(1000);
	 }
 }
 
 void GPIO_init(void)
 {
	 RCC->AHBENR |= 0x00000002;			// IO port B clock enabled (bit 1) --> All GPIOs are connected to AHB bus, so the relevant register is RCC_AHBENR 
	 GPIOB->MODER |= 0x00005000;		// As long as reset value is 0, we just need to write 01 to the relavant section to configure PB6 and PB7 as output 
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
 
 // that will change the speed based on the optimization settings
void delay_ms(int delay)
{
	volatile int i;
	int j;
	for(j=delay;j>0;j--)
	{
		for(i=0;i<130;i++);
	}
}

 