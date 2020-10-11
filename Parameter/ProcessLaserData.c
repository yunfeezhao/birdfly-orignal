/*
 * ProcessLaserData.c
 *
 *  Created on: 2019年8月2日
 *      Author: Jccao
 */

#include "ProcessLaserData.h"

void Process_LaserData(unsigned char *LaserData){
    unsigned char cntRx =0;
    Uint16 sumVerify =0;  //和校验
    unsigned char sumCnt=0;  //和校验计数
    for(cntRx=0 ; cntRx<18 ; cntRx++)
    {
        if(LaserData[cntRx]== 0x59 && LaserData[cntRx+1]== 0x59 && cntRx<9)
        {
            for(sumCnt=0;sumCnt<8;sumCnt++)
                sumVerify += LaserData[cntRx+sumCnt];
            if( (sumVerify & 0x00ff) == LaserData[cntRx+8])
            {
                Uint16 Distance = (((Uint16)LaserData[cntRx+3]) << 8) + LaserData[cntRx+2];
                Uint16 Strength = (((Uint16)LaserData[cntRx+5]) << 8) + LaserData[cntRx+4];
                //根据数据手册强度在这个范围内有效
                if(Strength>400 && Strength<60000 && Distance > 0 && Distance < 900){
                    Sensor_Info.Laser = ((float)Distance) * 0.01f;   // 米制单位
                }
            }
            cntRx = 18;
        }
    }
}
