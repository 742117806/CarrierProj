#ifndef __MATH__H
#define __MATH__H
#ifdef __cplusplus
 extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include "stm32f0xx_hal.h"

unsigned char coder(unsigned char codes);
unsigned char encoder(unsigned char encodes,unsigned char * codes);
uint8_t CRC8_Tab(uint8_t *ucPtr,uint8_t ucLen);  
uint16_t CRC16_check(uint16_t *Data,uint16_t Data_length);
typedef struct {
	unsigned bit0:1;
	unsigned bit1:1;
	unsigned bit2:1;
	unsigned bit3:1;
	unsigned bit4:1;
	unsigned bit5:1;
	unsigned bit6:1;
	unsigned bit7:1;
}CharBits;

typedef union {
	CharBits bits;
	unsigned char byte;
}bytes;

#ifdef __cplusplus
}
#endif

#endif