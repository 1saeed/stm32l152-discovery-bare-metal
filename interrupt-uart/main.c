/*
 * interrupt-uart.c
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
 * to echo charachters sent from computer serial port to the microcontroller (9600, 8 bit data size, no parity and handshake off)
 * using receive interrupt service routine
 *
 *
 * Configuration:
 * uart:
 * 1. enable uart clock from RCC
 * 2. enalble gpioa clock
 * 3. set PA9 and PA10 as af7 
 * 4. (optional) set uart word lenght and parity or leave it by default
 * 5. enable transmit and receive (TE and RE bits)
 * 6. calculate baud rate and set BRR
 * 7. enable usart
 *
 * interrupt:
 * 1. disable all interrupts;
 * 2. config USART1->CR1 register for an interrupt whenever ORE=1 or RXNE=1 in the USART_SR register 
 * 3. enable USART1 interrupt.
 * 4. enable all interrupts.
.

 */
 
#include "stm32l1xx.h"                  // Device header
 
 /* Function declarations */
 void usart1_Init(void);
 void usart1_write(char ch);
 char usart1_read(void);
 void gpio_init(void);
 void led_blue(int status);
 void led_green(int status);
 void set_sysclk_to_hsi(void);
 void delay_ms(int delay);
 
 int main(void)
 {
	 set_sysclk_to_hsi();
	 gpio_init();
	 usart1_Init();
	 
	 while(1)
	 {
			//usart1_write(usart1_read());		// echo	
	 } 
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

 