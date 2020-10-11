/*
 * Bluetooth.h
 *
 *  Created on: 2018Äê5ÔÂ18ÈÕ
 *      Author: Xiluna Tech
 */

#ifndef SENSOR_OLED_H_
#define SENSOR_OLED_H_

#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "C28x_BSP.h"

#define Max_Column  128
#define SIZE        16

#define OLED_CMD()    GPIO_WritePin(66, 0);
#define OLED_DATA()   GPIO_WritePin(66, 1);

#define OLED_RES_Set()  GPIO_WritePin(64, 1);
#define OLED_RES_Clr()  GPIO_WritePin(64, 0);

void Delay(unsigned int us);

extern const unsigned char F6x8[][6];
extern const unsigned char F8X16[];
extern const unsigned char xiyuekeji[][32];
int float2String(char* outBuffer, unsigned int bufferSize, float in, unsigned int decCount);
void recurParse(int n, char** str, unsigned int* restSize) ;
void C28x_BSP_OLED_init(void);
extern void spi_b_xmit(Uint16 a);
void SPI_B_ReadWrite_Byte(unsigned char byte);
void OLED_writeCMD(unsigned char data);
void OLED_writeDATA(unsigned char data);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_Clear(void);
void OLED_ShowChar(unsigned char x,unsigned char y, char chr);
void OLED_ShowString(unsigned char x,unsigned char y, char *chr);
void OLED_ShowCHinese(unsigned char x,unsigned char y,unsigned char no);
int oled_pow(unsigned char m,unsigned char n);
void OLED_ShowNum(unsigned char x,unsigned char y,int num,unsigned char len);
void Display_Attitude(void);
void Display_Position(void);
void Display_Mode(void);
#endif /* SENSOR_OLED_H_ */
