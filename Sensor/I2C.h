/*
 * I2C.h
 *
 *  Created on: 2019Äê3ÔÂ28ÈÕ
 *      Author: SXJ
 */

#ifndef SENSOR_I2C_H_
#define SENSOR_I2C_H_


#include "F28x_Project.h"

#define I2CDelay 10

//MS5611 I2CB  34SDA  35SCL
////#define SDA_High()  {GpioDataRegs.GPBSET.bit.GPIO34 = 1;EALLOW;GpioCtrlRegs.GPBDIR.bit.GPIO34=1;EDIS;}
////#define SDA_Low() {GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;EALLOW;GpioCtrlRegs.GPBDIR.bit.GPIO34=1;EDIS;}//To clear the SDA line. Disable protection for writing register
#define SDA_High()  {GPIO_WritePin(91,1);}
#define SDA_Low() {GPIO_WritePin(91,0);}//To clear the SDA line. Disable protection for writing register
#define SDA_Input() { GPIO_SetupPinOptions(91, GPIO_INPUT,GPIO_PUSHPULL);}  // SDA DIR=Input
#define SDA_Output(){GpioCtrlRegs.GPCMUX2.bit.GPIO91=0;GpioCtrlRegs.GPCPUD.bit.GPIO91 = 0;EALLOW;GpioCtrlRegs.GPCDIR.bit.GPIO91=1;EDIS;}  // SDA DIR=Output
#define SDA_Data_Register GpioDataRegs.GPCDAT.bit.GPIO91
////#define SCL_High() {GpioDataRegs.GPBSET.bit.GPIO35 = 1;EALLOW;GpioCtrlRegs.GPBDIR.bit.GPIO35=1;EDIS;}     //Set the SCL line
////#define SCL_Low() {GpioDataRegs.GPBCLEAR.bit.GPIO35 = 1;EALLOW;GpioCtrlRegs.GPBDIR.bit.GPIO35=1;EDIS;}  //Clear the SCL line
#define SCL_High() {GPIO_WritePin(92,1);}     //Set the SCL line
#define SCL_Low() {GPIO_WritePin(92,0);}  //Clear the SCL line


//#define SDA_High()  {GpioDataRegs.GPBSET.bit.GPIO34 = 1;EALLOW;GpioCtrlRegs.GPBDIR.bit.GPIO34=1;EDIS;}
//#define SDA_Low() {GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;EALLOW;GpioCtrlRegs.GPBDIR.bit.GPIO34=1;EDIS;}//To clear the SDA line. Disable protection for writing register
//#define SDA_High()  {GPIO_WritePin(34,1);}
//#define SDA_Low() {GPIO_WritePin(34,0);}//To clear the SDA line. Disable protection for writing register
//#define SDA_Input() { GPIO_SetupPinOptions(34, GPIO_INPUT,GPIO_PUSHPULL);}  // SDA DIR=Input
//#define SDA_Output(){GpioCtrlRegs.GPBMUX1.bit.GPIO34=0;GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0;EALLOW;GpioCtrlRegs.GPBDIR.bit.GPIO34=1;EDIS;}  // SDA DIR=Output
//#define SDA_Data_Register GpioDataRegs.GPBDAT.bit.GPIO34
//#define SCL_High() {GpioDataRegs.GPBSET.bit.GPIO35 = 1;EALLOW;GpioCtrlRegs.GPBDIR.bit.GPIO35=1;EDIS;}     //Set the SCL line
//#define SCL_Low() {GpioDataRegs.GPBCLEAR.bit.GPIO35 = 1;EALLOW;GpioCtrlRegs.GPBDIR.bit.GPIO35=1;EDIS;}  //Clear the SCL line
//#define SCL_High() {GPIO_WritePin(35,1);}     //Set the SCL line
//#define SCL_Low() {GPIO_WritePin(35,0);}  //Clear the SCL line

void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
unsigned char  IIC_Wait_Ack(void);
unsigned char IIC_Read_Byte(unsigned char ack);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(unsigned char txd);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
unsigned char IICreadByte(unsigned char dev, unsigned char reg, unsigned char *data);
unsigned char IICwriteBytes(unsigned char dev, unsigned char reg, unsigned char length, unsigned char* data);
unsigned char IICreadBytes(unsigned char dev, unsigned char reg, unsigned char length, unsigned char *data);
#endif /* SENSOR_I2C_H_ */
