#include "stm32f0xx.h"  
#include "WS2812B.h"
#include "main.h"

uint8_t led_buf[24];

void SPI_Write_Buf(unsigned char *pBuf, unsigned char bytes)	{
  unsigned char i;

for( i = 0; i < bytes; i ++ )
 {
  while (!(SPI1->SR & SPI_SR_TXE)){};	
	SPI1_DR_8bit = (pBuf[i]); 
	while (SPI1->SR & SPI_SR_BSY){};
	while (!(SPI1->SR & SPI_SR_RXNE)){};
	SPI1->DR;	
 }
}

void Write_RGB(uint8_t r,uint8_t g, uint8_t b) {
uint8_t i;
 for (i=0;i<8;i++){
	if((g<<i)&0x80) {led_buf[i]=BIT1;}
		else 					{led_buf[i]=BIT0;}
	if((r<<i)&0x80) {led_buf[8+i]=BIT1;}
		else 					{led_buf[8+i]=BIT0;}
	if((b<<i)&0x80) {led_buf[16+i]=BIT1;}
		else 					{led_buf[16+i]=BIT0;}		
	}
SPI_Write_Buf(led_buf,24);

}
