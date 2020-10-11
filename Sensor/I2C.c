/*
 * IIC.c
 *
 *  Created on: 2019��3��28��
 *      Author: SXJ
 */

#include "I2C.h"

void IIC_Init(void)
{
//    IICbRegs.IICSAR.all = 0xEE;     // Slave address - EEPROM control code
//
//    IICbRegs.IICPSC.all = 16;         // Prescaler - need 7-12 Mhz on module clk//Ԥ��Ƶ��-��ģ��CLK����Ҫ7-12�׺�
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
//    GpioCtrlRegs.GPBMUX1.bit.GPIO34=0;      //ѡ������I/Oģʽ
//    GpioCtrlRegs.GPBPUD.bit.GPIO34=0;       //ʹ���ڲ���������

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

/**************************ʵ�ֺ���********************************************
*����ԭ��:      unsigned char IIC_ReadOneByte(unsigned char IIC_Addr,unsigned char addr)
*��������:      ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����  IIC_Addr  Ŀ���豸��ַ
        addr       �Ĵ�����ַ
����   ��������ֵ
*******************************************************************************/
unsigned char IIC_ReadOneByte(unsigned char IIC_Addr,unsigned char addr)
{
    unsigned char res=0;

    IIC_Start();
    IIC_Send_Byte(IIC_Addr);       //����д����
    res++;
    if( IIC_Wait_Ack())
  return 0;
    IIC_Send_Byte(addr); res++;  //���͵�ַ
    if( IIC_Wait_Ack())
  return 0;
    //IIC_Stop();//����һ��ֹͣ����
    IIC_Start();
    IIC_Send_Byte(IIC_Addr+1); res++;          //�������ģʽ
    if( IIC_Wait_Ack())
  return 0;
    res=IIC_Read_Byte(0);
    IIC_Stop();//����һ��ֹͣ����

    return res;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:      unsigned char IICreadBytes(unsigned char dev, unsigned char reg, unsigned char length, unsigned char *data)
*��������:      ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����  dev  Ŀ���豸��ַ
        reg   �Ĵ�����ַ
        length Ҫ�����ֽ���
        *data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/
unsigned char IICreadBytes(unsigned char dev, unsigned char reg, unsigned char length, unsigned char *data){
    unsigned char count = 0;

    IIC_Start();
    IIC_Send_Byte(dev);    //����д����
    if( IIC_Wait_Ack())
  return 0;
    IIC_Send_Byte(reg);   //���͵�ַ
    if( IIC_Wait_Ack())
  return 0;
    IIC_Start();
    IIC_Send_Byte(dev+1);  //�������ģʽ
    if( IIC_Wait_Ack())
  return 0;

    for(count=0;count<length;count++){

         if(count!=length-1)data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
            else  data[count]=IIC_Read_Byte(0);  //���һ���ֽ�NACK
    }
    IIC_Stop();//����һ��ֹͣ����
    return count;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:      unsigned char IICwriteBytes(unsigned char dev, unsigned char reg, unsigned char length, unsigned char* data)
*��������:      ������ֽ�д��ָ���豸 ָ���Ĵ���
����  dev  Ŀ���豸��ַ
        reg   �Ĵ�����ַ
        length Ҫд���ֽ���
        *data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/
unsigned char IICwriteBytes(unsigned char dev, unsigned char reg, unsigned char length, unsigned char* data){

    unsigned char count = 0;
    IIC_Start();
    IIC_Send_Byte(dev);    //����д����
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);   //���͵�ַ
    IIC_Wait_Ack();
    for(count=0;count<length;count++){
        IIC_Send_Byte(data[count]);
        IIC_Wait_Ack();
     }
    IIC_Stop();//����һ��ֹͣ����

    return 1; //status == 0;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:      unsigned char IICreadByte(unsigned char dev, unsigned char reg, unsigned char *data)
*��������:      ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����  dev  Ŀ���豸��ַ
        reg    �Ĵ�����ַ
        *data  ���������ݽ�Ҫ��ŵĵ�ַ
����   1
*******************************************************************************/
unsigned char IICreadByte(unsigned char dev, unsigned char reg, unsigned char *data){
    *data=IIC_ReadOneByte(dev, reg);
    return 1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:      unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*��������:      д��ָ���豸 ָ���Ĵ���һ���ֽ�
����  dev  Ŀ���豸��ַ
        reg    �Ĵ�����ַ
        data  ��Ҫд����ֽ�
����   1
*******************************************************************************/
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

