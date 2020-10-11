/*
 * MPU6500.c
 *
 *  Created on: 2018骞�4鏈�17鏃�
 *      Author: Xiluna Tech
 */
#include "MPU6500.h"
#include <ucos_ii.h>
#define Buf_SIZE  10


int16_t MPU6500_FIFO[7][Buf_SIZE];
static int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;


unsigned char SPI_ReadWrite_Byte(unsigned char byte){
    spi_xmit(((Uint16)byte)<<8);
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
    return SpiaRegs.SPIRXBUF;
}



void MPU6500_readRegs(unsigned char reg, unsigned char length, unsigned char *data){
    unsigned char count = 0;
    SPI_ReadWrite_Byte(reg |0x80);
    for(count=0;count<length;count++){
            data[count] = SPI_ReadWrite_Byte(0xff);
    }
}

static void MPU6500_writeReg(unsigned char reg, unsigned char data){
    MPU6500_CSL();
    SPI_ReadWrite_Byte(reg);
    SPI_ReadWrite_Byte(data);
    MPU6500_CSH();
}

void MPU6500_initialize(void){
    MPU6500_writeReg(MPU6500_RA_PWR_MGMT_1,0x80);           // 澶嶄綅
    DELAY_US(500*100);
    MPU6500_writeReg(MPU6500_RA_SIGNAL_PATH_RESET,0x07);
    DELAY_US(500*100);
    MPU6500_writeReg(MPU6500_RA_PWR_MGMT_1,0x03);           // 浣跨敤鍐呴儴20M鏃堕挓
    DELAY_US(500*100);
    MPU6500_writeReg(MPU6500_RA_SMPLRT_DIV,0x00);           // sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz
    DELAY_US(500*100);
    MPU6500_writeReg(MPU6500_RA_ACCEL_CONFIG,0x10);         // +-8G accel
    DELAY_US(500*100);
    MPU6500_writeReg(MPU6500_RA_FF_THR,0x04);               // 鍔犺浣庨�氭护娉㈠櫒  20Hz
    DELAY_US(500*100);
    MPU6500_writeReg(MPU6500_RA_GYRO_CONFIG,0x18);          // +-2000 gyro
    DELAY_US(500*100);
    MPU6500_writeReg(MPU6500_RA_CONFIG, 0x03);              // 鍐呴儴浣庨�氬反鐗规矁鏂护娉㈠櫒  鎴棰戠巼 42Hz
    DELAY_US(500*100);
    MPU6500_writeReg(MPU6500_RA_PWR_MGMT_2,0x00);
    DELAY_US(500*100);
}


void MPU6500_readGyro_Acc(int16_t *gyro,int16_t *acc){
    static unsigned char buf[14];
    static int16_t gx,gy,gz;
    static int16_t ax,ay,az;
    MPU6500_CSL();
    MPU6500_readRegs(MPU6500_RA_ACCEL_XOUT_H,14,buf);
    MPU6500_CSH();
    //acc
    ax =    (int16_t)(((int16_t)buf[0]) << 8 | buf[1]);
    ay =    (int16_t)(((int16_t)buf[2]) << 8 | buf[3]);
    az =    (int16_t)(((int16_t)buf[4]) << 8 | buf[5]);

    acc[0] = ax;     //acc
    acc[1] = ay;
    acc[2] = az;

    //temp
  //  MPU6500_NewVal(&MPU6500_FIFO[3][0],(int16_t)(((int16_t)buf[6]) << 8 | buf[7]));
    //gyro
//    MPU6500_NewVal(&MPU6500_FIFO[4][0],(int16_t)(((int16_t)buf[8]) << 8 | buf[9]));
//    MPU6500_NewVal(&MPU6500_FIFO[5][0],(int16_t)(((int16_t)buf[10]) << 8 | buf[11]));
//    MPU6500_NewVal(&MPU6500_FIFO[6][0],(int16_t)(((int16_t)buf[12]) << 8 | buf[13]));

    gx =  (int16_t)(((int16_t)buf[8]) << 8 | buf[9]);
    gy =  (int16_t)(((int16_t)buf[10]) << 8 | buf[11]);
    gz =  (int16_t)(((int16_t)buf[12]) << 8 | buf[13]);

    gyro[0] = gx - Gx_offset;   //gyro
    gyro[1] = gy - Gy_offset;
    gyro[2] = gz - Gz_offset;

}


void MPU6500_Init_Offset(void)
{
    unsigned int i;
    int16_t temp[3],temp2[3];
    int32_t tempgx=0,tempgy=0,tempgz=0;
    Gx_offset=0;
    Gy_offset=0;
    Gz_offset=0;

    for(i=0;i<500;i++){
        DELAY_US(300);
        MPU6500_readGyro_Acc(temp,temp2);
        tempgx += temp[0];
        tempgy += temp[1];
        tempgz += temp[2];
    }
    Gx_offset=tempgx/500;
    Gy_offset=tempgy/500;
    Gz_offset=tempgz/500;
}

