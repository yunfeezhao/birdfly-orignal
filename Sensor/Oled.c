/*
 * Bluetooth.c
 *
 *  Created on: 2018年5月18日
 *      Author: Xiluna Tech
 */
#include "Oled.h"
#include <stdio.h>

#include <stdlib.h>
#include <string.h>

extern const unsigned char F6x8[][6];
extern const unsigned char F8X16[];
extern const unsigned char xiyuekeji[][32];

void Delay(unsigned int us)
{
    int k1 = 0,k2 = 0;
    if(k2 == 0){k1 = OSTime;k2 = 1;}
    while((OSTime - k1) <= us)
    {
        k2 = 0;
    }
}

void spi_b_xmit(Uint16 a)
{
    SpibRegs.SPITXBUF = a;
}

void SPI_B_ReadWrite_Byte(unsigned char byte)
{
    spi_b_xmit(((Uint16)byte)<<8);

}

void OLED_writeCMD(unsigned char data)
{
    OLED_CMD();
    SPI_B_ReadWrite_Byte(data);
    DELAY_US(20);
   //OSTimeDly(5000);
    OLED_DATA();
}

void OLED_writeDATA(unsigned char data)
{
    OLED_DATA();
    SPI_B_ReadWrite_Byte(data);
    DELAY_US(20);
   //OSTimeDly(5000);
    OLED_DATA();
}

void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_writeCMD(0xb0+y);
    OLED_writeCMD(((x&0xf0)>>4)|0x10);
    OLED_writeCMD((x&0x0f)|0x01);
}

void OLED_Clear(void)
{
    unsigned char i,n;
    for(i=0;i<8;i++)
    {
        OLED_writeCMD(0xb0+i);    //设置页地址（0~7）
        OLED_writeCMD(0x00);      //设置显示位置—列低地址
        OLED_writeCMD(0x10);      //设置显示位置—列高地址
        for(n=0;n<128;n++)
            OLED_writeDATA(0);
    } //更新显示
}

void OLED_ShowChar(unsigned char x,unsigned char y, char chr)
{
    unsigned char c=0,i=0;
        c=chr-' ';
        if(x>Max_Column-1){x=0;y=y+2;}
        if(SIZE ==16)
            {
            OLED_Set_Pos(x,y);
            for(i=0;i<8;i++)
                OLED_writeDATA(F8X16[c*16+i]);
            OLED_Set_Pos(x,y+1);
            for(i=0;i<8;i++)
                OLED_writeDATA(F8X16[c*16+i+8]);
            }
            else {
                OLED_Set_Pos(x,y+1);
                for(i=0;i<6;i++)
                    OLED_writeDATA(F6x8[c][i]);
            }
}
int oled_pow(unsigned char m,unsigned char n)
{
    int result=1;
    while(n--)result*=m;
    return result;
}
void OLED_ShowNum(unsigned char x,unsigned char y,int num,unsigned char len)
{
    unsigned char t,temp;
    unsigned char enshow=0;
    for(t=0;t<len;t++)
    {
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                OLED_ShowChar(x+t,y,' ');
                continue;
            }else enshow=1;

        }
        OLED_ShowChar(x+t,y,temp+'0');
    }
}
void OLED_ShowString(unsigned char x,unsigned char y, char *chr)
{
    unsigned char j=0;
    while (chr[j]!='\0')
    {       OLED_ShowChar(x,y,chr[j]);
            x+=8;
        if(x>120){x=0;y+=2;}
            j++;
    }
}

void OLED_ShowCHinese(unsigned char x,unsigned char y,unsigned char no)
{
    unsigned char t,adder=0;
    OLED_Set_Pos(x,y);
    for(t=0;t<16;t++)
    {
        OLED_writeDATA(xiyuekeji[2*no][t]);
       adder+=1;
    }
    OLED_Set_Pos(x,y+1);
    for(t=0;t<16;t++)
    {
        OLED_writeDATA(xiyuekeji[2*no+1][t]);
       adder+=1;
     }
}

void C28x_BSP_OLED_init(void)
{
    unsigned char i =0;

    OLED_RES_Set();
    for(i = 0;i<=100;i++)
        DELAY_US(1000);
    OLED_RES_Clr();
    for(i = 0;i<=200;i++)
        DELAY_US(1000);
    OLED_RES_Set();

    OLED_writeCMD(0xAE);//--turn off oled panel
    OLED_writeCMD(0x00);//---set low column address
    OLED_writeCMD(0x10);//---set high column address
    OLED_writeCMD(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_writeCMD(0x81);//--set contrast control register
    OLED_writeCMD(0xCF);// Set SEG Output Current Brightness
    OLED_writeCMD(0xA1);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    OLED_writeCMD(0xC8);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    OLED_writeCMD(0xA6);//--set normal display
    OLED_writeCMD(0xA8);//--set multiplex ratio(1 to 64)
    OLED_writeCMD(0x3f);//--1/64 duty
    OLED_writeCMD(0xD3);//-set display offset   Shift Mapping RAM Counter (0x00~0x3F)
    OLED_writeCMD(0x00);//-not offset
    OLED_writeCMD(0xd5);//--set display clock divide ratio/oscillator frequency
    OLED_writeCMD(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_writeCMD(0xD9);//--set pre-charge period
    OLED_writeCMD(0xF1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_writeCMD(0xDA);//--set com pins hardware configuration
    OLED_writeCMD(0x12);
    OLED_writeCMD(0xDB);//--set vcomh
    OLED_writeCMD(0x40);//Set VCOM Deselect Level
    OLED_writeCMD(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_writeCMD(0x02);//
    OLED_writeCMD(0x8D);//--set Charge Pump enable/disable
    OLED_writeCMD(0x14);//--set(0x10) disable
    OLED_writeCMD(0xA4);// Disable Entire Display On (0xa4/0xa5)
    OLED_writeCMD(0xA6);// Disable Inverse Display On (0xa6/a7)
    OLED_writeCMD(0xAF);//--turn on oled panel

    OLED_writeCMD(0xAF); /*display ON*/
    OLED_Clear();
    OLED_Set_Pos(0,0);

}
char arr[20];
void recurParse(int n, char** str, unsigned int* restSize) {
        if (n < 10) {
                if (*restSize > 1) {
                        **str = n + '0';
                        (*str)++;
                        *restSize -= 1;
                }

        }
        else {
                recurParse(n / 10, str, restSize);
                if (*restSize > 1) {
                        **str = n % 10 + '0';
                        (*str)++;
                        *restSize -= 1;
                }
        }
}
/*
*outBuffer: 输出字符串的缓冲区
*bufferSize: 输出字符串缓冲区的大小
*in: 要转换的浮点数
*decCount: 输出字符串保留的小数位数
*返回值: 写入到缓冲区中的字符个数（不包含\0）
*/
int float2String(char* outBuffer, unsigned int bufferSize, float in, unsigned int decCount) {
        int n;
        float dec;
        unsigned int restSize = bufferSize;
        if (restSize == 0) return 0;
        n = (int)in;
        dec = in - n;
        if (in < 0) {
                if (restSize > 1 && !(n == 0 && decCount == 0)) {
                        *outBuffer++ = '-';
                        --restSize;
                }
                n = -n;
        }
        if (dec < 0) {
                dec = -dec;
        }
        recurParse(n, &outBuffer, &restSize);
        if (decCount != 0) {
                if (restSize > 1) {
                        *outBuffer++ = '.';
                        --restSize;
                }
        }
        while (decCount) {
                dec *= 10;
                n = (int)dec;
                dec -= n;
                if (restSize > 1) {
                        *outBuffer++ = n + '0';
                        --restSize;
                }
                else {
                        break;
                }
                decCount -= 1;
        }
        *outBuffer++ = NULL;
        return bufferSize - restSize;
}
void Display_Attitude(void)
{
    //OLED_Clear();
    OLED_ShowString(0,0,"Pitch:");
    float2String(arr,20,RT_Info.Pitch,3);
    OLED_ShowString(60,0,arr);

    OLED_ShowString(0,2,"Roll:");
    float2String(arr,20,RT_Info.Roll,3);
    OLED_ShowString(60,2,arr);

    OLED_ShowString(0,4,"Yaw:");
    float2String(arr,20,RT_Info.Yaw,3);
    OLED_ShowString(60,4,arr);

    OLED_ShowString(0,6,"Height:");
    float2String(arr,20,RT_Info.Height,3);
    OLED_ShowString(60,6,arr);
}
void Display_Position(void)
{
    //OLED_Clear();
    OLED_ShowString(0,0,"FlowX:");
    float2String(arr,20,RT_Info.FlowX,3);
    OLED_ShowString(60,0,arr);

    OLED_ShowString(0,2,"FlowY:");
    float2String(arr,20,RT_Info.FlowY,3);
    OLED_ShowString(60,2,arr);

    OLED_ShowString(0,4,"PointX:");
    float2String(arr,20,RT_Info.PointX,3);
    OLED_ShowString(60,4,arr);

    OLED_ShowString(0,6,"PointY:");
    float2String(arr,20,RT_Info.PointY,3);
    OLED_ShowString(60,6,arr);
}

void Display_Mode(void)
{
    //OLED_Clear();
    OLED_ShowString(0,0,"Mode:");
    switch(FlightControl.DroneMode){
        case Drone_Mode_None:
            OLED_ShowString(60,0,"None");
            break;
        case Drone_Mode_RatePitch:
            OLED_ShowString(60,0,"RatePitch");
            break;
        case Drone_Mode_RateRoll:
            OLED_ShowString(60,0,"RateRoll");
            break;
        case Drone_Mode_Pitch:
            OLED_ShowString(60,0,"Pitch");
            break;
        case Drone_Mode_Roll:
            OLED_ShowString(60,0,"Roll");
            break;
        case Drone_Mode_4Axis:
            OLED_ShowString(60,0,"4Axis");
            break;
        default:
            break;
    }


    OLED_ShowString(0,2,"OnOff:");
    switch(FlightControl.OnOff){
    case Drone_Off:
        OLED_ShowString(60,2,"Off");
        break;
    case Drone_On:
        OLED_ShowString(60,2,"On");
        break;
    case Drone_Land:
        OLED_ShowString(60,2,"Land");
        break;
    default:break;
    }

    OLED_ShowString(0,4,"Fly_Mode:");
    switch(Fly_Mode){
    case Data_Headmode:
        OLED_ShowString(70,4,"mode");
        break;
    case Data_Headfree:
        OLED_ShowString(70,4,"free");
        break;
    case Data_Point:
        OLED_ShowString(70,4,"Point");
        break;
    case Data_Flow:
        OLED_ShowString(70,4,"Flow");
        break;
    case Data_Follow:
        OLED_ShowString(70,4,"Follow");
        break;
    default:break;
    }
}


