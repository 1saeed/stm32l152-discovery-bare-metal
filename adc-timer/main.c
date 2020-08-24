/*
 * adc-timer.c
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
 * ADC				: ADC_IN16 (Temperature sensor)
 *
 * Clock:
 * 	HSI RC 16MHz with AHB prescaler 1, cortex system timer divider 1, APB1 and APB2 prescaler 1 --> 16MHz
 *
 * Purpose:   
 * read internal temperature sensor by 1Hz timer trigger and send voltage value to serial port.
 *
 *
 * Configuration:
 * timer:
 * 1. set TIM3EN bit in RCC_APB1ENR register to enable clock for TIM3
 * 2. set TIM3_PSC and TIM3_ARR register to generate 1Hz timing
 * 3. set OC1M bits in TIM3_CCMR1 register to 011 to toggle
 * 4. set TIM3_CCR1 to zero
 * 5. set CC1E bit in TIM3_CCER register to enable compare 1 output
 * 6. set TIM3_CNT to zero
 * 7. enable TIM3
 *
 * adc:
 * 1. enable clock for ADC1 in APB2ENR register
 * 2. disable adc
 * 3. set regular sequence registers to channel 16th (temperature sensor)
 * 4. set sampling time for channel 16th more than Ts_temp
 * 5. set TSVREFE bit in ADC_CCR register to enable temperature sensor
 * 6. set EXTEN bits in ADC_CR2 register to set trigger detection on the rising edge
 * 7. set the EXTSEL[3:0] bits in ADC_CR2 register to select TIM3_CC1 event 
 * 8. enable ADC
 * 
 */
 
#include "stm32l1xx.h"                  // Device header
 
 /* Function declarations */
 void tim3_compare_mode_init(void);
 void tim9_init(void);
 void adc_init(void);
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
 
 
 char * ts_cal2_add = 0x1FF800FE;
 char * ts_cal1_add = 0x1FF800FA;
 
 #define TS_CAL2	(((unsigned int)ts_cal2_add[1] << 8) + (unsigned int)ts_cal2_add[0])
 #define TS_CAL1  (((unsigned int)ts_cal1_add[1] << 8) + (unsigned int)ts_cal1_add[0])
 
 int main(void)
 {
	 unsigned int adc_result = 0;
	 char str[40];
	 float temperature = 0;
	 set_sysclk_to_hsi();
	 gpio_init();
	 usart1_Init();
	 adc_init();
	 tim3_compare_mode_init();
	 
	 usart1_write(0);			// dummy send
	 while(1)
	 {  				
		  while(!((ADC1->SR & 0x00000002) == 0x00000002)){}								// wait until regular channel end of conversion
				adc_result = ADC1->DR;
				temperature = ((110.0-30.0)/(TS_CAL2-TS_CAL1))*(float)(adc_result-TS_CAL1)+30;
				sprintf(str,"TEMP = %.2f\370c\r\n",temperature);
				usart1_send_string(str);
	 } 
 }
 	
 void tim3_compare_mode_init(void)
 { 	 
	 RCC->APB1ENR  |= 0x00000002;			  // enable timer3 clock
	 TIM3->PSC      = 1600 - 1;				  // 10KHz @16MHz
	 TIM3->ARR      = 10000 - 1;				// 1Hz 
	 
	 TIM3->CCMR1    = 0x0030;						// toggle - OC1REF toggles when TIMx_CNT=TIMx_CCR1
	 TIM3->CCR1     = 0;								// set match mode for ch1
	 TIM3->CCER		 |= 0x00000001;				// compare enable ch1
	 
	 TIM3->CNT			= 0;								// reset timer counter
	 TIM3->CR1		 |= 0x00000001;				// enable timer3		 
 }
 
 void adc_init(void)
 {
		RCC->APB2ENR |= 0x00000200;									// ADC1 interface clock enabled
	 
	  ADC1->CR2 	 = 0x00000000;									// disable ADC 
	  ADC1->SQR5 	|= 0x00000010;									// 1st conversion in regular sequence for ADC channel 16
	 
	  ADC1->SMPR2 |= 0x001C0000;									// sampling time 384 cycles > Ts_temp
	  ADC->CCR   	|= 0x00800000;									// temperature sensor and VREFINT channel enabled
	 
    ADC1->CR2		|= 0x10000000; 									// trigger detection on the rising edge
	  ADC1->CR2		|= 0x07000000;									// TIM3_CC1 event
	 
	  sys_tick_delay_ms(1);												// wait for its stabilization time (tSTART).
	  ADC1->CR2 	|= 0x00000001;									// enable ADC  
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

 