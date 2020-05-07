// MCU STM32F042
// WS2812IN PIN - PA7- MOSI
// I2C LCD Pinout  - PB6 - SCL, PB7 - SDA

#include "stm32f0xx.h"        // Device header
#include "SSD1306.h"
#include "WS2812B.h"

#define HSI_48

	#define I2C_GPIOB			1	//		i2C PB6,PB7
//#define I2C_GPIOF			1	//		i2C PF0,PF1

uint8_t test=0,sec_tic,t_count=0;
uint16_t TimingDelay,led_count,ms1000;
/*
uint8_t testGreen[]={	0x40,0x40,0x40,0x40,0x40,0x40,0x70,0x70, 		// Green	
											0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,		// Red
											0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40};	// Blue

uint8_t testRed[]=	{	0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,	//Green
											0x40,0x40,0x40,0x40,0x40,0x40,0x70,0x70,	//Red
											0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40};	//Blue
											
uint8_t testBlue[]=	{	0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,	//Green
											0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,	//Red
											0x40,0x40,0x40,0x40,0x40,0x40,0x70,0x70};	//Blue

*/
											
void TimingDelayDec(void) 																												{
 if (TimingDelay			!=0x00) TimingDelay--;
 if (!led_count) {led_count=500; GPIOB->ODR ^=1;}
 if (!ms1000) {ms1000=300;sec_tic=1;}
 led_count--;ms1000--;
 }

void TIM17_IRQHandler(void)																												{
		if (TIM17->SR & TIM_SR_UIF) {
					TimingDelayDec();
  				TIM17->SR &=(~TIM_SR_UIF);
		}
}	
void delay_ms (uint16_t DelTime) 																									{
    TimingDelay=DelTime;
  while(TimingDelay!= 0x00);
}

void initial (void)																																{
	RCC->CR2 |=RCC_CR2_HSI48ON;
	while (!(RCC->CR2 & RCC_CR2_HSI48RDY));
	RCC->CFGR |=RCC_CFGR_SW;                                          //Switch for HSI48
	while (!(RCC->CFGR & RCC_CFGR_SWS_HSI48));
	
	RCC->CFGR3 |=RCC_CFGR3_I2C1SW_SYSCLK;   // switch I2C to HSI48
	RCC->CR &=~RCC_CR_HSION;
	
	//---------------TIM17------------------
  RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;    																			//HSI 48 MHz - 1 msek
	TIM17->PSC = 48000-1;
  TIM17->ARR = 1;
  TIM17->CR1 |= TIM_CR1_ARPE | TIM_CR1_DIR | TIM_CR1_CEN; 											// 
	TIM17->DIER |=TIM_DIER_UIE;
	NVIC_EnableIRQ (TIM17_IRQn);
	NVIC_SetPriority(TIM17_IRQn,0x05);	

//-------------------GPIOB-Blinking Led		
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN; 																					//
	GPIOB->MODER |= GPIO_MODER_MODER0_0;																					//Pb0-Out 
//-------------------GPIOA-CONFIG RX MODE----------
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN; 																					//
	GPIOA->MODER &=(~GPIO_MODER_MODER2);																					//switch	
	GPIOA->PUPDR |=GPIO_PUPDR_PUPDR2_0;																						//Pull-Up
//------------I2C1---------------------	


#ifdef I2C_GPIOB
//------------I2C1 GPIOB_SETTING ---------------------	
	RCC->AHBENR 		|=RCC_AHBENR_GPIOBEN;
	GPIOB->MODER 		|=GPIO_MODER_MODER6_1 		| GPIO_MODER_MODER7_1; 							// Alt -mode /Pb6 -SCL , Pb7- SDA
	GPIOB->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR6 	| GPIO_OSPEEDER_OSPEEDR7;
	GPIOB->OTYPER		|=GPIO_OTYPER_OT_6 				| GPIO_OTYPER_OT_7;
	GPIOB->AFR[0] 	|=(1<<GPIO_AFRL_AFRL6_Pos) |(1<<GPIO_AFRL_AFRL7_Pos);  				// I2C - Alternative PB7, PB6
#endif	
#ifdef I2C_GPIOF	
	//------------I2C1 GPIOF_SETTING---------------------	
	RCC->AHBENR 		|=RCC_AHBENR_GPIOFEN;
	GPIOF->MODER 		|=GPIO_MODER_MODER0_1 		| GPIO_MODER_MODER1_1; 							// Alt -mode /Pf0 - SDA, Pf1- SCL
	GPIOF->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR0 	| GPIO_OSPEEDER_OSPEEDR1;
	GPIOF->OTYPER		|=GPIO_OTYPER_OT_0 				| GPIO_OTYPER_OT_1;
	GPIOF->AFR[0] 	|=(1<<GPIO_AFRL_AFRL0_Pos) |(1<<GPIO_AFRL_AFRL1_Pos);  				// I2C - Alternative
#endif	

RCC->APB1ENR |=RCC_APB1ENR_I2C1EN;
I2C1->TIMINGR =0x50330309; 	//for HSI 48MHz 

I2C1->CR2 &=(~I2C_CR2_HEAD10R) & (~I2C_CR2_ADD10);
I2C1->CR1 |=I2C_CR1_PE;
	
	
//------------------------SPI-----------------------------------
	RCC->AHBENR 		|=RCC_AHBENR_GPIOAEN;
	GPIOA->OSPEEDR 	|= GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR4 
										|GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
	GPIOA->MODER 		|=GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; 	//Pa2, Pa4 - out,Pa5..7 - Alt_mode 
	GPIOA->AFR[0] 	|=(0<<GPIO_AFRL_AFRL7_Pos) |(0<<GPIO_AFRL_AFRL6_Pos) | (0<<GPIO_AFRL_AFRL5_Pos);  // SPI - Alternative
	GPIOA->MODER 		|=GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0; //Pa3, Pa4 - out, Pa0 Input inerrupt
	GPIOA->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR6 	| GPIO_OSPEEDER_OSPEEDR7;
	
	RCC->APB2ENR |=RCC_APB2ENR_SPI1EN;
	SPI1->CR1 |=SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | (3<<SPI_CR1_BR_Pos);  // if HSI8 - SpiSpeed (BR=2) - 1MHz
	SPI1->CR2 |=SPI_CR2_FRXTH;//
	SPI1->CR1 |=SPI_CR1_SPE;
		__enable_irq ();	
} 

int main(void)
{
initial();
delay_ms (100);	
LCD_Init();LCD_Clear();
LCD_Gotoxy (10,0);LCD_PrintStr(" TEST WS2812B ",1);
 
//-----------------------------initial data----------------------------------

while (1)  /* Main loop */
{
	
	if (sec_tic) {	sec_tic=0;
	LCD_Gotoxy (1,7); LCD_PrintStr("LCD = ",0);LCD_PrintDec(test++,0);LCD_PrintStr("   ",0);

	for (uint8_t ri = 0;ri<255;ri++)	{
		Write_RGB(ri,0,0);
		LCD_Gotoxy (1,3); LCD_PrintStr("R = ",0);LCD_PrintDec(ri,0);LCD_PrintStr("   ",0);
			delay_ms(2);
	}
		for (uint8_t gi = 0;gi<255;gi++)	{
			Write_RGB(0,gi,0);
			LCD_Gotoxy (1,3); LCD_PrintStr("G = ",0);LCD_PrintDec(gi,0);LCD_PrintStr("   ",0);
				delay_ms(2);
		}
			for (uint8_t bi = 0;bi<255;bi++)	{
				Write_RGB(0,0,bi);
				LCD_Gotoxy (1,3); LCD_PrintStr("B = ",0);LCD_PrintDec(bi,0);LCD_PrintStr("   ",0);
				delay_ms(2);
			}

		}


} // end - main loop 
} // end - Main  
	
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ while (1)  {  } }
#endif
