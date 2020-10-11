/*
 * IIC.c
 *
 *  Created on: 2019年3月28日
 *      Author: SXJ
 */

#include "I2C.h"

void IIC_Init(void)
{
//    IICbRegs.IICSAR.all = 0xEE;     // Slave address - EEPROM control code
//
//    IICbRegs.IICPSC.all = 16;         // Prescaler - need 7-12 Mhz on module clk//预分频器-在模块CLK上需要7-12兆赫
//    IICbRegs.IICCLKL = 10;            // NOTE: must be non zero
//    IICbRegs.IICCLKH = 5;             // NOTE: must be non zero
//    IICbRegs.IICIER.all = 0x24;       // Enable SCD & ARDY __interrupts
//
//    IICbRegs.IICMDR.all = 0x0020;     // Take IIC out of reset
//                                      // Stop IIC when suspended
//
//    IICbRegs.IICFFTX.all = 0x6000;    // Enable FIFO mode and TXFIFO
//    IICbRegs.IICFFRX.all = 0x2040;    // Enable RXFIFO, clear RXFFINT,

//    EALLOW;
//    GpioCtrlRegs.GPBMUX1.bit.GPIO34=0;      //选择数字I/O模式
//    GpioCtrlRegs.GPBPUD.bit.GPIO34=0;       //使能内部上拉电阻

//    EDIS;
    SDA_Output();
    GPIO_SetupPinMux(91, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(91, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(92, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(92, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_WritePin(91,1);
    GPIO_WritePin(92,1);

//    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
//    GPIO_SetupPinMux(35, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(35, GPIO_OUTPUT, GPIO_PUSHPULL);
//    GPIO_WritePin(34,1);
//    GPIO_WritePin(35,1);
//    SDA_High();
//    SCL_High();
}


void IIC_Start(void)
{
    SDA_Output();
    SCL_High();   // Set the SCL
    SDA_High();   // Set the SDA
    DELAY_US(I2CDelay);
    SDA_Low();    // Clear the SDA while SCL is high indicates the start signal
    DELAY_US(I2CDelay);
    SCL_Low();    // Clear the SCL to get ready to transmit
}

void IIC_Stop(void)
{
    SDA_Output();
    SCL_Low();    // Clear the SCL
    SDA_Low();    // Clear the SDA
    DELAY_US(I2CDelay);
    SCL_High();   // Set the SCL
    SDA_High();   // Set the SDA while SCL is high indicates the finish signal
    DELAY_US(I2CDelay);
}

unsigned char IIC_Wait_Ack(void)
{
    int ErrTime=0;
    int ReadAck=0;
    SDA_Input();  // Config SDA GPIO as Input
    SDA_High();
    DELAY_US(I2CDelay);
    SCL_High();   // Set the SCL and wait for ACK
    DELAY_US(I2CDelay);

    ReadAck = GPIO_ReadPin(91); ;  // Read the input
//    ReadAck = GPIO_ReadPin(34); ;  // Read the input
    while(ReadAck)
    {
        ErrTime++;
        if(ErrTime>250)
        {
                 //Error handler:Set error flag, retry or stop.
                //Define by users
             IIC_Stop();
             return 1;
         }
     }

      SCL_Low();   // Clear the SCL for Next Transmit
      return 0;

}

void IIC_Ack(void)          //IIC_MSGSTAT_SEND_WITHSTOP
{
    SCL_Low();
    SDA_Output();
    SDA_Low();
    DELAY_US(10);
    SCL_High();
    DELAY_US(10);
    SCL_Low();
}

void IIC_NAck(void)         //IIC_MSGSTAT_SEND_NOSTOP
{
    SCL_Low();     // Clear the SCL to get ready to transmit
    SDA_Output();
    SDA_High();    // Clear the SDA
    DELAY_US(10);
    SCL_High();    // Set the SCL
    DELAY_US(10);
    SCL_Low();      // Clear the SCL
}

void IIC_Send_Byte(unsigned char txd)
{
    int t;
    SDA_Output();   // Config SDA GPIO as output
    SCL_Low();      // Clear the SCL to get ready to transmit
    for(t=0;t<8;t++)
        {
           SDA_Data_Register = (txd & 0x80)>>7;  // Send the LSB
           txd<<=1;
           DELAY_US(10);
           SCL_High();   // Set the SCL
           DELAY_US(10);
           SCL_Low();    // Clear the SCL
           DELAY_US(10);
        }

}

unsigned char IIC_Read_Byte(unsigned char ack)
{
    int t;
    unsigned char receive=0;
    SDA_Input();
    for(t=0;t<8;t++)
    {
        SCL_Low();   // Clear the SCL
        DELAY_US(10);
        SCL_High();   // Set the SCL
        receive<<=1;
        if(GPIO_ReadPin(91))
        {
          receive++;
        }
        DELAY_US(10);
     }
    if (!ack)
        IIC_NAck();
    else
        IIC_Ack();
     return receive;
}

/**************************实现函数********************************************
*函数原型:      unsigned char IIC_ReadOneByte(unsigned char IIC_Addr,unsigned char addr)
*功　　能:      读取指定设备 指定寄存器的一个值
输入  IIC_Addr  目标设备地址
        addr       寄存器地址
返回   读出来的值
*******************************************************************************/
unsigned char IIC_ReadOneByte(unsigned char IIC_Addr,unsigned char addr)
{
    unsigned char res=0;

    IIC_Start();
    IIC_Send_Byte(IIC_Addr);       //发送写命令
    res++;
    if( IIC_Wait_Ack())
  return 0;
    IIC_Send_Byte(addr); res++;  //发送地址
    if( IIC_Wait_Ack())
  return 0;
    //IIC_Stop();//产生一个停止条件
    IIC_Start();
    IIC_Send_Byte(IIC_Addr+1); res++;          //进入接收模式
    if( IIC_Wait_Ack())
  return 0;
    res=IIC_Read_Byte(0);
    IIC_Stop();//产生一个停止条件

    return res;
}


/**************************实现函数********************************************
*函数原型:      unsigned char IICreadBytes(unsigned char dev, unsigned char reg, unsigned char length, unsigned char *data)
*功　　能:      读取指定设备 指定寄存器的 length个值
输入  dev  目标设备地址
        reg   寄存器地址
        length 要读的字节数
        *data  读出的数据将要存放的指针
返回   读出来的字节数量
*******************************************************************************/
unsigned char IICreadBytes(unsigned char dev, unsigned char reg, unsigned char length, unsigned char *data){
    unsigned char count = 0;

    IIC_Start();
    IIC_Send_Byte(dev);    //发送写命令
    if( IIC_Wait_Ack())
  return 0;
    IIC_Send_Byte(reg);   //发送地址
    if( IIC_Wait_Ack())
  return 0;
    IIC_Start();
    IIC_Send_Byte(dev+1);  //进入接收模式
    if( IIC_Wait_Ack())
  return 0;

    for(count=0;count<length;count++){

         if(count!=length-1)data[count]=IIC_Read_Byte(1);  //带ACK的读数据
            else  data[count]=IIC_Read_Byte(0);  //最后一个字节NACK
    }
    IIC_Stop();//产生一个停止条件
    return count;
}

/**************************实现函数********************************************
*函数原型:      unsigned char IICwriteBytes(unsigned char dev, unsigned char reg, unsigned char length, unsigned char* data)
*功　　能:      将多个字节写入指定设备 指定寄存器
输入  dev  目标设备地址
        reg   寄存器地址
        length 要写的字节数
        *data  将要写的数据的首地址
返回   返回是否成功
*******************************************************************************/
unsigned char IICwriteBytes(unsigned char dev, unsigned char reg, unsigned char length, unsigned char* data){

    unsigned char count = 0;
    IIC_Start();
    IIC_Send_Byte(dev);    //发送写命令
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);   //发送地址
    IIC_Wait_Ack();
    for(count=0;count<length;count++){
        IIC_Send_Byte(data[count]);
        IIC_Wait_Ack();
     }
    IIC_Stop();//产生一个停止条件

    return 1; //status == 0;
}


/**************************实现函数********************************************
*函数原型:      unsigned char IICreadByte(unsigned char dev, unsigned char reg, unsigned char *data)
*功　　能:      读取指定设备 指定寄存器的一个值
输入  dev  目标设备地址
        reg    寄存器地址
        *data  读出的数据将要存放的地址
返回   1
*******************************************************************************/
unsigned char IICreadByte(unsigned char dev, unsigned char reg, unsigned char *data){
    *data=IIC_ReadOneByte(dev, reg);
    return 1;
}

/**************************实现函数********************************************
*函数原型:      unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*功　　能:      写入指定设备 指定寄存器一个字节
输入  dev  目标设备地址
        reg    寄存器地址
        data  将要写入的字节
返回   1
*******************************************************************************/
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

