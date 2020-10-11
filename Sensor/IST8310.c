#include "IST8310.h"

unsigned char add;
void IST8310_Init(void)
{
    IICwriteByte(IST8310_SLAVE_ADDRESS,0x41,0x24);
    IICwriteByte(IST8310_SLAVE_ADDRESS,0x42,0xC0);
}


IST8310 Mag_IST8310;
void Get_Mag_IST8310(void)
{
//  static uint16_t IST8310_Sample_Cnt=0;
//  IST8310_Sample_Cnt++;
//  if(IST8310_Sample_Cnt==1)
//  {
//      IICwriteByte(IST8310_SLAVE_ADDRESS,IST8310_REG_CNTRL1,0x01);//Single Measurement Mode
//  }
//  else if(IST8310_Sample_Cnt==4)
//  {
//    IICreadBytes(IST8310_SLAVE_ADDRESS,0x03,6,&Mag_IST8310.Buf[0]);
//    Mag_IST8310.Mag_Data[0]=(Mag_IST8310.Buf[1]<<8)|Mag_IST8310.Buf[0];
//    Mag_IST8310.Mag_Data[1]=(Mag_IST8310.Buf[3]<<8)|Mag_IST8310.Buf[2];
//    Mag_IST8310.Mag_Data[2]=(Mag_IST8310.Buf[5]<<8)|Mag_IST8310.Buf[4];
//    IST8310_Sample_Cnt=0;
//  }
//
//  Mag_IST8310.x = Mag_IST8310.Mag_Data[0];
//  Mag_IST8310.y = -Mag_IST8310.Mag_Data[1];
//  Mag_IST8310.z = Mag_IST8310.Mag_Data[2];
}

