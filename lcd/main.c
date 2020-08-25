/*
 * lcd.c
 *
 * Author: 
 * Saeed Poorjandaghi
 *
 * Pinout: 
 * LED Blue 	: PB6
 * LED Green	: PB7
 * USER button: PA0
 * LCD				: below
 *
 * COM1:PA9/COM0:PA8/SEG12:PA15/SEG13:PB8/SEG14:PC0/SEG15:PC1/SEG16:PC2/SEG17:PC3/SEG18:PC6/SEG19:PC7/SEG20:PC8/SEG21:PC9/SEG22:PC10/SEG23:PC11
 * COM2:PA10/CPM3:PB9/SEG11:PB15/SEG10:PB14/SEG9:PB13/SEG8:PB12/SEG7:PB11/SEG6:PB10/SEG5:PB5/SEG4:PB4/SEG3:PB3/SEG2:PA3/SEG1:PA2/SEG0:PA1
 *
 * Clock:
 * 	HSI RC 16MHz with AHB prescaler 1, cortex system timer divider 1, APB1 and APB2 prescaler 1 --> 16MHz
 *
 * Purpose:   
 * write standard characters on LCD (24 segments, 4 commons)
 *
 * LCD Mapping: 
 
			    A
     _  ----------
COL |_| |\   |J  /|
       F| H  |  K |B
     _  |  \ | /  |
COL |_| --G-- --M--
        |   /| \  |
       E|  Q |  N |C
     _  | /  |P  \|   
DP  |_| -----------  
	    D         

 An LCD character coding is based on the following matrix:
      { E , D , P , N   }
      { M , C , COL , DP}
      { B , A , K , J   }
      { G , F , Q , H   }

 The character 'A' for example is:
  -------------------------------
LSB   { 1 , 0 , 0 , 0   }
      { 1 , 1 , 0 , 0   }
      { 1 , 1 , 0 , 0   }
MSB   { 1 , 1 , 0 , 0   }
      -------------------
  'A' =  F    E   0   0 hexa


 *
 * Configuration:
 * (Drive method: multiplexed 1/4 duty, 1/3 bias)
 * 1. set RTCSEL[1:0] bits in RCC_CSR register to select LSI as clock source
 * 2. set LCDEN bit in RCC_APB1ENR to enable LCD clock
 * 3. enable clock for PORTA/B/C
 * 4. set GPIO pin for LCD as alternate function
 * 5. set alternate function as LCD (AF11) for LCD GPIOs
 * 6. set PS[3:0] and DIV[3:0] so that ck_ps = LCDCLK and ck_div = ck_ps/31 
 * 7. check FCRSF bit in LCD_SR register to be set (LCD Frame Control Register synchronized)
 * 8. set VSEL, DUTY[2:0] and BIAS[1:0] bits in LCD_CR register so that voltage source = internal, duty = 1/4 and bias = 1/3
 * 9. set MUX_SEG bit in LCD_CR register so SEG[31:28] are multiplexed with SEG[43:40]
 * 10.set CC[2:0] bits in LCD_FCR register so that Vlcd maximum voltage = Vlcd4 (3.12V)
 * 11.set PON[2:0] bits in LCD_FCR register so that pulse on durattion = 4/ck_ps 	
 * 12.check FCRSF bit in LCD_SR register to be set (LCD Frame Control Register synchronized)
 * 13.set LCDEN bit in LCD_CR register to enable LCD controller 
 * 14.check ENS bit in LCD_SR register and wait to be set
 * 15.check RDY bit in LCD_SR register and wait to be set
 * 16.set BLINKF[2:0] bits in LCD_FCR register to fLCD/32110
 * 
 */
 
#include "stm32l1xx.h"                  // Device header

/* Constant table for cap characters 'A' --> 'Z' */
const unsigned short CapLetterMap[26]=
    {
        /* A      B      C      D      E      F      G      H      I  */
        0xFE00,0x6714,0x1d00,0x4714,0x9d00,0x9c00,0x3f00,0xfa00,0x0014,
        /* J      K      L      M      N      O      P      Q      R  */
        0x5300,0x9841,0x1900,0x5a48,0x5a09,0x5f00,0xFC00,0x5F01,0xFC01,
        /* S      T      U      V      W      X      Y      Z  */
        0xAF00,0x0414,0x5b00,0x18c0,0x5a81,0x00c9,0x0058,0x05c0
    };

/* Constant table for number '0' --> '9' */
const unsigned short NumberMap[10]=
    {
        /* 0      1      2      3      4      5      6      7      8      9  */
        0x5F00,0x4200,0xF500,0x6700,0xEa00,0xAF00,0xBF00,0x04600,0xFF00,0xEF00
    };
	
/* LCD BAR status: We don't write directly in LCD RAM for save the bar setting */
unsigned char t_bar[2]={0x0,0X0};
		
 /* Function declarations */
 static void LCD_Conv_Char_Seg(unsigned char* c,unsigned char point,unsigned char column, unsigned char* digit);	
 void LCD_GLASS_WriteChar(unsigned char* ch, unsigned char point, unsigned char column, unsigned char position);
 void LCD_GLASS_DisplayString(unsigned char* ptr);
 void lcd_init(void);	
 void lcd_clear(void);		
 void gpio_init(void);
 void led_blue(int status);
 void led_green(int status);
 void set_sysclk_to_hsi(void);
 void sys_tick_delay_ms(int ms);
 void delay_ms(int delay);
 
 
 int main(void)
 {
	 lcd_init();
	 lcd_clear();
	 
   gpio_init();	
   led_blue(1);	
   LCD_GLASS_DisplayString("STM32");	 
	 while(1)
	 {  				
		  
	 } 
 }
 	
 
 void lcd_init(void)
 {
	  RCC->CSR     	|= 0x00020000;										// LSI oscillator clock used as RTC/LCD clock
	  RCC->APB1ENR 	|= 0x00000200;										// LCD clock enabled
	 
	  RCC->AHBENR  	|= 0x00000007;										// enable clock for PORTA/B/C
	  GPIOA->MODER 	|= 0x802A00A8;										// LCD GOPIO as alternate function
	  GPIOB->MODER 	|= 0xAAAA0A80;										// LCD GOPIO as alternate function
	  GPIOC->MODER 	|= 0x00AAA0AA;										// LCD GOPIO as alternate function
	  
	  GPIOA->AFR[0]	|= 0x0000BBB0;										// alternate function selection for LCD (AF11)
	  GPIOA->AFR[1] |= 0xB0000BBB;										// alternate function selection for LCD (AF11)
	 
	  GPIOB->AFR[0] |= 0x00BBB000;										// alternate function selection for LCD (AF11)
	  GPIOB->AFR[1] |= 0xBBBBBBBB;										// alternate function selection for LCD (AF11)
	 
	  GPIOC->AFR[0] |= 0xBB00BBBB;										// alternate function selection for LCD (AF11)
	  GPIOC->AFR[1] |= 0x0000BBBB;										// alternate function selection for LCD (AF11)
	 
	  LCD->FCR			|= 0x003C0000;										// set ck_ps = LCDCLK and ck_div = ck_ps/31 
	  while((LCD->SR & 0x00000020) == 0x00000000){}		// wait until LCD Frame Control Register synchronized
		
		LCD->CR				|= 0x0000004C;										// voltage source = internal, duty = 1/4 and bias = 1/3
		LCD->CR				|= 0x00000080;										// when MUX_SEG is set, output pins SEG[43:40] have function SEG[31:28]
    
		LCD->FCR			|= 0x00001000;										// set contrast to Vlcd4 (3.12V)	
    LCD->FCR			|= 0x00000040;										// set pulse on durattion to 4/ck_ps 			
		while((LCD->SR & 0x00000020) == 0x00000000){}		// wait until LCD Frame Control Register synchronized
		
		LCD->CR				|= 0x00000001;										// LCD Controller enabled
    while((LCD->SR & 0x00000001) == 0x00000000){}		// wait for LCD enables status to be set
    while((LCD->SR & 0x00000010) == 0x00000000){}		// wait for LCD ready flag to be set 
    
    LCD->FCR			|= 0x00004000;										// blink frequency = fLCD/32110			
 }
 
 void lcd_clear(void)
 {
	  unsigned char count = 0;
    while((LCD->SR & 0x00000004) == 0x00000004){}			// wait for UDR flag to reset
		
    for(count = 0; count <= 15 ; count++)
    {
			LCD->RAM[count] = 0;
		}		
		
		LCD->SR		|= 0x00000004;		// Update Display request
 }	 
 
 
 static void LCD_Conv_Char_Seg(unsigned char* c,unsigned char point,unsigned char column, unsigned char* digit)
{
  unsigned short ch = 0 ;
  unsigned char i,j;
  
  switch (*c)
    {
    case ' ' : 
      ch = 0x00;
      break;
    
    case '*':
      ch = 0xA0DD;
      break;
                  
    case 'µ' :
      ch = 0x6084;
      break;
    
    case 'm' :
      ch = 0xb210;
      break;
                  
    case 'n' :
      ch = 0x2210;
      break;					
                  
    case '-' :
      ch = 0xA000;
      break;
      
    case '/' :
      ch = 0x00c0;
      break;  
      
    case '°' :
      ch = 0xec00;
      break;  
    case '%' :
      ch = 0xb300; 
      break;
    case 255 :
      ch = 0xffdd;
      break ;
    
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':			
      ch = NumberMap[*c-0x30];		
      break;
          
    default:
      /* The character c is one letter in upper case*/
      if ( (*c < 0x5b) && (*c > 0x40) )
      {
        ch = CapLetterMap[*c-'A'];
      }
      /* The character c is one letter in lower case*/
      if ( (*c <0x7b) && ( *c> 0x60) )
      {
        ch = CapLetterMap[*c-'a'];
      }
      break;
  }
       
  /* Set the digital point can be displayed if the point is on */
  if (point)
  {
    ch |= 0x0002;
  }

  /* Set the "COL" segment in the character that can be displayed if the column is on */
  if (column)
  {
    ch |= 0x0020;
  }		

  for (i = 12,j=0 ;j<4; i-=4,j++)
  {
    digit[j] = (ch >> i) & 0x0f; //To isolate the less signifiant dibit
  }
}
 
void LCD_GLASS_WriteChar(unsigned char* ch, unsigned char point, unsigned char column, unsigned char position)
{
  unsigned char digit[4];     /* Digit frame buffer */
   
/* To convert displayed character in segment in array digit */
  LCD_Conv_Char_Seg(ch,point,column,digit);

/* TO wait LCD Ready */ 
  while((LCD->SR & 0x00000004) == 0x00000004){}			// wait for UDR flag to reset	
  
  switch (position)
  {
    /* Position 1 on LCD (Digit1)*/
    case 1:
      LCD->RAM[0] &= 0xcffffffc;
      LCD->RAM[2] &= 0xcffffffc;
      LCD->RAM[4] &= 0xcffffffc;
      LCD->RAM[6] &= 0xcffffffc;

      LCD->RAM[0] |= ((digit[0]& 0x0c) << 26 ) | (digit[0]& 0x03) ; // 1G 1B 1M 1E	    
      LCD->RAM[2] |= ((digit[1]& 0x0c) << 26 ) | (digit[1]& 0x03) ; // 1F 1A 1C 1D 
      LCD->RAM[4] |= ((digit[2]& 0x0c) << 26 ) | (digit[2]& 0x03) ; // 1Q 1K 1Col 1P                                                                                                                                    
      LCD->RAM[6] |= ((digit[3]& 0x0c) << 26 ) | (digit[3]& 0x03) ; // 1H 1J 1DP 1N

      break;
    
    /* Position 2 on LCD (Digit2)*/
    case 2:
      LCD->RAM[0] &= 0xf3ffff03;
      LCD->RAM[2] &= 0xf3ffff03;      
      LCD->RAM[4] &= 0xf3ffff03;
      LCD->RAM[6] &= 0xf3ffff03;
      
      LCD->RAM[0] |= ((digit[0]& 0x0c) << 24 )|((digit[0]& 0x02) << 6 )|((digit[0]& 0x01) << 2 ) ; // 2G 2B 2M 2E	  
      LCD->RAM[2] |= ((digit[1]& 0x0c) << 24 )|((digit[1]& 0x02) << 6 )|((digit[1]& 0x01) << 2 ) ; // 2F 2A 2C 2D
      LCD->RAM[4] |= ((digit[2]& 0x0c) << 24 )|((digit[2]& 0x02) << 6 )|((digit[2]& 0x01) << 2 ) ; // 2Q 2K 2Col 2P
      LCD->RAM[6] |= ((digit[3]& 0x0c) << 24 )|((digit[3]& 0x02) << 6 )|((digit[3]& 0x01) << 2 ) ; // 2H 2J 2DP 2N
      
      break;
    
    /* Position 3 on LCD (Digit3)*/
    case 3:
      LCD->RAM[0] &= 0xfcfffcff;
      LCD->RAM[2] &= 0xfcfffcff;
      LCD->RAM[4] &= 0xfcfffcff;
      LCD->RAM[6] &= 0xfcfffcff;

      LCD->RAM[0] |= ((digit[0]& 0x0c) << 22 ) | ((digit[0]& 0x03) << 8 ) ; // 3G 3B 3M 3E	
      LCD->RAM[2] |= ((digit[1]& 0x0c) << 22 ) | ((digit[1]& 0x03) << 8 ) ; // 3F 3A 3C 3D
      LCD->RAM[4] |= ((digit[2]& 0x0c) << 22 ) | ((digit[2]& 0x03) << 8 ) ; // 3Q 3K 3Col 3P
      LCD->RAM[6] |= ((digit[3]& 0x0c) << 22 ) | ((digit[3]& 0x03) << 8 ) ; // 3H 3J 3DP 3N
      
      break;
    
    /* Position 4 on LCD (Digit4)*/
    case 4:
      LCD->RAM[0] &= 0xffcff3ff;
      LCD->RAM[2] &= 0xffcff3ff;
      LCD->RAM[4] &= 0xffcff3ff;
      LCD->RAM[6] &= 0xffcff3ff;
      
      LCD->RAM[0] |= ((digit[0]& 0x0c) << 18 ) | ((digit[0]& 0x03) << 10 ) ; // 4G 4B 4M 4E	
      LCD->RAM[2] |= ((digit[1]& 0x0c) << 18 ) | ((digit[1]& 0x03) << 10 ) ; // 4F 4A 4C 4D
      LCD->RAM[4] |= ((digit[2]& 0x0c) << 18 ) | ((digit[2]& 0x03) << 10 ) ; // 4Q 4K 4Col 4P
      LCD->RAM[6] |= ((digit[3]& 0x0c) << 18 ) | ((digit[3]& 0x03) << 10 ) ; // 4H 4J 4DP 4N
      
      break;
    
    /* Position 5 on LCD (Digit5)*/
    case 5:
      LCD->RAM[0] &= 0xfff3cfff;
      LCD->RAM[2] &= 0xfff3cfff;
      LCD->RAM[4] &= 0xfff3efff;
      LCD->RAM[6] &= 0xfff3efff;

      LCD->RAM[0] |= ((digit[0]& 0x0c) << 16 ) | ((digit[0]& 0x03) << 12 ) ; // 5G 5B 5M 5E	
      LCD->RAM[2] |= ((digit[1]& 0x0c) << 16 ) | ((digit[1]& 0x03) << 12 ) ; // 5F 5A 5C 5D
      LCD->RAM[4] |= ((digit[2]& 0x0c) << 16 ) | ((digit[2]& 0x01) << 12 ) ; // 5Q 5K   5P 
      LCD->RAM[6] |= ((digit[3]& 0x0c) << 16 ) | ((digit[3]& 0x01) << 12 ) ; // 5H 5J   5N
      
      break;
    
    /* Position 6 on LCD (Digit6)*/
    case 6:
      LCD->RAM[0] &= 0xfffc3fff;
      LCD->RAM[2] &= 0xfffc3fff;
      LCD->RAM[4] &= 0xfffc3fff;
      LCD->RAM[6] &= 0xfffc3fff;

      LCD->RAM[0] |= ((digit[0]& 0x04) << 15 ) | ((digit[0]& 0x08) << 13 ) | ((digit[0]& 0x03) << 14 ) ; // 6B 6G 6M 6E	
      LCD->RAM[2] |= ((digit[1]& 0x04) << 15 ) | ((digit[1]& 0x08) << 13 ) | ((digit[1]& 0x03) << 14 ) ; // 6A 6F 6C 6D
      LCD->RAM[4] |= ((digit[2]& 0x04) << 15 ) | ((digit[2]& 0x08) << 13 ) | ((digit[2]& 0x01) << 14 ) ; // 6K 6Q    6P 
      LCD->RAM[6] |= ((digit[3]& 0x04) << 15 ) | ((digit[3]& 0x08) << 13 ) | ((digit[3]& 0x01) << 14 ) ; // 6J 6H   6N
      
      break;
    
     default:
      break;
  }

/* Refresh LCD  bar */
  LCD->RAM[4] &= 0xffff5fff;
  LCD->RAM[6] &= 0xffff5fff;
/* bar1 bar3 */
  LCD->RAM[4] |= (unsigned int)(t_bar[0]<<12);
  
/*bar0 bar2 */
  LCD->RAM[6] |= (unsigned int)(t_bar[1]<<12);

/* Update the LCD display */
  LCD->SR		|= 0x00000004;		// Update Display request
  
}

 void LCD_GLASS_DisplayString(unsigned char* ptr)
{
  unsigned char i = 0x01;

  /* Send the string character by character on lCD */
  while ((*ptr != 0) & (i < 8))
  {
    /* Display one character on LCD */
    LCD_GLASS_WriteChar(ptr, 0, 0, i);

    /* Point on the next character */
    ptr++;

    /* Increment the character counter */
    i++;
  }
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

 