/*
 * clock.c
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
 * on the fly
 * 	1. (default) MSI RC 2097 KHz changes to --> 4.194 MHz
 * 	2. HSI RC 16MHz with AHB prescaler 1, cortex system timer divider 1, APB1 and APB2 prescaler 1 --> 16MHz
 * 	3. PLLCLK with HSI RC 16MHz input, PLLMul 4 and PLLDiv 2 and other prescalers 1 --> 32MHz
 *
 * Purpose:   
 * to run clock at different frequencies 
 * and different sources and change it on the fly by pressing user button
 * which changes blinking frequency by dummy delay function.
 *
 * Note:
 * 1. with different optimization options, blinking speeds will
 * vary since a dummy counter is in use.
 * 2. since no interrupt is used to detect button release, system would be in halt unless the button is released.
 * 3. since there is a considerable delay in main loop, button should be kept released enogh to be detected.
 *
 * Configuration:
 *  1. change msirange[2..0] bits in RCC_ICSCR register to choose the
 *     frequency range of MSI.
 *  2. configure RCC_CR and RCC_CFGR to select HSI as clock source
 *  3. PLL configuration notes:
 *		The PLL configuration (selection of the source clock, multiplication factor and output division factor)
 *		must be performed before enabling the PLL. Once the PLL is enabled, these parameters cannot be changed.
 *		To modify the PLL configuration, proceed as follows:
 *				a.Disable the PLL by setting PLLON to 0.
 *				b. Wait until PLLRDY is cleared. PLLRDY. The PLL is now fully stopped.
 *				c. Change the desired parameter.
 *				d. Enable the PLL again by setting PLLON to 1. 
 * 
 * Important note when switching to high frequencies:
 *		• Program the 64-bit access by setting the ACC64 bit in FLASH_ACR
 *		• Check that 64-bit access is taken into account by reading FLASH_ACR
 *		• Program 1 WS to the LATENCY bit in FLASH_ACR
 *		• Check that the new number of WS is taken into account by reading FLASH_ACR
 *		• Modify the CPU clock source by writing to the SW bits in the RCC_CFGR register
 *		• If needed, modify the CPU clock prescaler by writing to the HPRE bits in RCC_CFGR
 *		• Check that the new CPU clock source or/and the new CPU clock prescaler value is/are
 *			taken into account by reading the clock source status (SWS bits) or/and the AHB
 *			prescaler value (HPRE bits), respectively, in the RCC_CFGR register
 */
 
#include "stm32l1xx.h"                  // Device header
 
 /* Function declarations */
 void set_sysclk_to_msi_2097(void);
 void set_sysclk_to_msi_4194(void);
 void set_sysclk_to_hsi(void);
 void set_sysclk_to_pll_32(void);
 int if_button_pressed(void);
 void GPIO_init(void);
 void led_blue(int status);
 void led_green(int status);
 void delay_ms(int delay);
 
 int main(void)
 {
	 unsigned int sMachine = 0;
	 GPIO_init();
	 //set_sysclk_to_msi_4194();
	 
	 while(1)
	 {
		 if(if_button_pressed())
		 {
			 switch(sMachine)
			 {
				 case 0:
					 set_sysclk_to_msi_4194();
				   sMachine++;
				 break;
				 
				 case 1:
					 set_sysclk_to_hsi();
					 sMachine++;
				 break;
				 
				 case 2:
					 set_sysclk_to_pll_32();
				   sMachine++;
				 break;
				 
				 default:
					 set_sysclk_to_msi_2097();
					 sMachine = 0;
				 break;
			 }
		 }
		 led_blue(1);
		 led_green(1);
		 
		 delay_ms(1000);
		 
		 led_blue(0);
		 led_green(0);
		 
		 delay_ms(1000);
	 }
 }
 
 void set_sysclk_to_msi_2097(void)
 {
	 SystemInit();
	 // change msirange[2..0] bits
	 RCC->ICSCR &= 0xFFFFBFFF;
	 RCC->ICSCR |= 0x0000A000;
	 
	 while(!((RCC->CR & 0x00000200) == 0x00000200)){}	// waitn until MSI oscillator is ready
	 while(!((RCC->CFGR & 0x0000000C) == 0x00)){}			// wait until MSI is used as system clock 
		
	 /* Decreasing the CPU frequency (in the same voltage range). */
	 if((FLASH->ACR & 0x00000004) == 0x00000000)				// Latency can be set only when ACC64 is set.
	 {
			FLASH->ACR &= (unsigned int)~0x00000001;						// Program 0 WS to the LATENCY bit in FLASH_ACR
			while(((FLASH->ACR & 0x00000001) == 0x00000001)){}	// Check that the new number of WS is taken into account by reading FLASH_ACR
				
		  FLASH->ACR &= (unsigned int)~0x00000004;		  			// Program the 32-bit access by clearing ACC64 in FLASH_ACR
			while(((FLASH->ACR & 0x00000004) == 0x00000004)){}	// Check that 32-bit access is taken into account by reading FLASH_ACR
	 }		 
		 
	 SystemCoreClockUpdate();
 }
 
 void set_sysclk_to_msi_4194(void)
 {
	 SystemInit();
	 // change msirange[2..0] bits
	 RCC->ICSCR &= 0xFFFFDFFF;
	 RCC->ICSCR |= 0x0000C000;
	 
	 while(!((RCC->CR & 0x00000200) == 0x00000200)){}	// waitn until MSI oscillator is ready
	 while(!((RCC->CFGR & 0x0000000C) == 0x00)){}			// wait until MSI is used as system clock 
	 
	 SystemCoreClockUpdate();
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
 
 void set_sysclk_to_pll_32(void)
 {
	  RCC->CR |= 0x00000001;								 // HSI oscillator ON
	  while(!((RCC->CR & 0x00000002) == 0x00000002)){}	// waitn until HSI oscillator is ready
				 
	 RCC->CR &= (unsigned int)~0x01000000; // disable PLL //0x8EFAFEFE;		// clear all rw bits 
	 while((RCC->CR & 0x02000000) == 0x02000000){}	// Wait until PLLRDY is cleared. PLLRDY. The PLL is now fully stopped.
	    		 
	 // these bits can be written only when the PLL is disabled 		 
	 
	 RCC->CFGR &= (uint32_t)0xFF02FFFF; // Reset PLLSRC, PLLMUL[3:0] and PLLDIV[1:0] bits   	 
	 RCC->CFGR |= 0x00400000; // PLL clock output division = PLLVCO/2
	 RCC->CFGR |= 0x00040000; // PLL multiplication facot = PLL clock entry x 4	
	 
	 RCC->CR |= 0x01000000;								 // PLL enable
   while(!((RCC->CR & 0x02000000) == 0x02000000)){}	// wait until PLL is locked		 
		
	 /* Increasing the CPU frequency (in the same voltage range). */	 
	 FLASH->ACR |= 0x00000004;															// Program the 64-bit access by setting the ACC64 bit in FLASH_ACR
	 while(!((FLASH->ACR & 0x00000004) == 0x00000004)){}		// Check that 64-bit access is taken into account by reading FLASH_ACR
   
	 FLASH->ACR |= 0x00000001;															// Program 1 WS to the LATENCY bit in FLASH_ACR
	 while(!((FLASH->ACR & 0x00000001) == 0x00000001)){}		// Check that the new number of WS is taken into account by reading FLASH_ACR
		 	 
	 RCC->CFGR |= 0x00000003;							 									// PLL used as system clock  
		
	 while(!((RCC->CFGR & 0x0000000C) == 0x0C)){}						// wait until PLL is used as system clock 
			  	 
	 SystemCoreClockUpdate();
 }
 
 int if_button_pressed(void)
 {
	 if(GPIOA->IDR & 0x00000001)					// if button is pressed
	 {
		 while(GPIOA->IDR & 0x00000001){}		// wait until button is released
			 return 1;
	 }
	 else
	 {
		 return 0;
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

 