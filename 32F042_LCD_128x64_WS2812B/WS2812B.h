//Library has been modified for STM32F0xx by Aleksey Sedyshev
//https://github.com/AlekseySedyshev

#ifndef WS2812B_h

#define WS2812B_h
#include "stm32f0xx.h"  
#include "stdint.h"  

#define BIT0 0x40
#define BIT1 0x70

#define SPI1_DR_8bit 			*((__IO uint8_t *)&SPI1->DR)		// Limit for spi bus 8 bit

void SPI_Write_Buf(unsigned char *pBuf, unsigned char bytes);
void Write_RGB(uint8_t r,uint8_t g, uint8_t b);
#endif
