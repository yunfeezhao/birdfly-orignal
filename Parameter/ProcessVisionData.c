/*
 * ProcessVisionData.c
 *
 *  Created on: 2018骞�5鏈�2鏃�
 *      Author: Xiluna Tech
 */

#include "ProcessVisionData.h"

void Process_VisionData(Uint16 *VisionData){
    unsigned char cntRx =0;
    Uint16 CorrectionData[16];
    unsigned char i =0;
    float LastPosX =0 ;
    float LastPosY= 0 ;
    float LastPosZ =0;


    float DisMesPara = 1060;  // f/pixel_len
    float ColWidth = 3.5; //柱子直径 单位：cm


//    unsigned char CheckSum=0;
    for(cntRx=0 ; cntRx<32 ; cntRx++)
    {
        if(VisionData[cntRx]== 0x55 && VisionData[cntRx+1]== 0xAA)
        {
            for(i=0 ; i<16 ; i++)
            {
                CorrectionData[i] = VisionData[cntRx+i];
            }
            cntRx = 32;
        }
    }
    if(CorrectionData[0]== 0x55 && CorrectionData[1]== 0xAA
            && CorrectionData[15]== 0xAA )
    {
        int tmp = 0;
        int tmp1 = 0;
        if(CorrectionData[2]==0x00)
        {
            Fly_Mode = Data_Headmode;             //榛樿鏈夊ご妯″紡
        }
        else if(CorrectionData[2]==0x10)
        {
            tmp = ( (int)CorrectionData[3]<<8 ) + CorrectionData[4];
            Sensor_Info.Raspberry_Xaxis = ( ((float)tmp - 80) * RT_Info.Height * 0.225f ) / 0.304f +  RT_Info.Height* 100 * tan(RT_Info.Pitch * 0.0174f );
            tmp = ( (int)CorrectionData[5]<<8 ) + CorrectionData[6];
            Sensor_Info.Raspberry_Yaxis = ( ((float)tmp - 60) * RT_Info.Height * 0.225f ) / 0.304f +  RT_Info.Height* 100 * tan(-RT_Info.Roll * 0.0174f ) ;
            Fly_Mode = Data_Point;               //瀹氱偣璺熻釜妯″紡
        }
        else if(CorrectionData[2]==0x20)
        {
            tmp =  ( (int)CorrectionData[3]<<8 ) + CorrectionData[4];
            //Sensor_Info.FlowVelX = ( (float)tmp  *0.1f  * RT_Info.Height * 0.225f ) / 0.304f ;
            tmp = ( (int)CorrectionData[5]<<8 ) + CorrectionData[6];
            //Sensor_Info.FlowVelY = ( (float)tmp  *0.1f  * RT_Info.Height * 0.225f ) / 0.304f ;
            Fly_Mode = Data_Flow;               //鍏夋祦瀹氱偣妯″紡
        }
//        else if(CorrectionData[2]==0x30)
//        {
//            tmp = ( (int)CorrectionData[3]<<8 ) + CorrectionData[4];
//            Sensor_Info.Raspberry_Yaxis = ( ((float)tmp - 60) * RT_Info.Height * 0.225f ) / 0.304f +  RT_Info.Height* 100 * tan(-RT_Info.Roll * 0.0174f ) ;
//            tmp = ( (int)CorrectionData[5]<<8 ) + CorrectionData[6];
//            if(tmp >= -45 && tmp <= 45)
//            {
//                Fly_Mode = Data_Line;
//                Sensor_Info.LineAngel = tmp;
//                //Sensor_Info.LineAngel = LPButterworth((float)tmp,&LineAngel_filter_buf[0],&RotateFlowX_Parameter) ;
//            }
//            //else if()
//                //Sensor_Info.LineAngel = LPButterworth((float)(tmp+90),&LineAngel_filter_buf[0],&RotateFlowX_Parameter) ;
//            else
//            {
//                Sensor_Info.LineAngel = 0;
//                Fly_Mode = Data_Flow;
//            }
//            //RT_Info.VioYaw   = - UnsignedcharToFloat(CorrectionData,7) *180 /PI;
//           // Fly_Mode = Data_Line;               //巡线模式
//        }
        else if(CorrectionData[2]==0x40)
        {
            tmp = ( (int)CorrectionData[3]<<8 ) + CorrectionData[4];  // 柱子像素宽度
            Sensor_Info.Distance = DisMesPara * ColWidth / (float)tmp;
            tmp1 = ( (int)CorrectionData[5]<<8 ) + CorrectionData[6]; //Y轴坐标
            Sensor_Info.Vertical_Yaxis = ( ((float)tmp1 - 240) * ColWidth) / (float)tmp;
            Sensor_Info.isCentre = CorrectionData[7]; //C1N0
            Sensor_Info.isHealth = CorrectionData[8]; //R1L0
            Fly_Mode = Data_ToCentre;

//              if (FlightControl.LaunchFlag == true ||  tmp >= 70 )
//              {
//                  Fly_Mode = Data_Flow;
//              }
//              else
//              {
//                  Fly_Mode = Data_ToCentre;
//              }

        }
//        else if(CorrectionData[2]==0x50) //绕飞
//        {
//            tmp = ( (int)CorrectionData[3]<<8 ) + CorrectionData[4];
//
//            tmp = ( (int)CorrectionData[5]<<8 ) + CorrectionData[6];
//        }
    }
    else if(CorrectionData[0]== 0x55 && CorrectionData[1]== 0xAA)
    {
//        for( i=0;i<13;i++)
//        {
//           CheckSum += (unsigned  char)(CorrectionData[i]);
//        }
        if(CorrectionData[2]==0x88  && CorrectionData[15]== 0xFF)
        {
            //RT_Info.VioYaw   = - (( (int)CorrectionData[9]<<8 ) + CorrectionData[10]) *180 /PI;
            RT_Info.VioPosition_X = UnsignedcharToFloat(CorrectionData,3);
            RT_Info.VioPosition_Y = UnsignedcharToFloat(CorrectionData,7);
            RT_Info.VioPosition_Z = (float)((( (int)CorrectionData[11]<<8 ) + CorrectionData[12])) /1000 -10;

            if(LastPosX!= RT_Info.VioPosition_X  && LastPosY!= RT_Info.VioPosition_Y  && LastPosZ!= RT_Info.VioPosition_Z )
                    RT_Info.VioHeartbeat =0;

            LastPosX = RT_Info.VioPosition_X;
            LastPosY = RT_Info.VioPosition_Y;
            LastPosZ = RT_Info.VioPosition_Z;

            if(CorrectionData[13]==0xA5)
            {
                 FlightControl.landFlag=1;
            }
            FlightControl.ArmPower =    ArmPowerOFF;
        }
        else if(CorrectionData[2]==0x99)
        {
            RT_Info.VioYaw   = - UnsignedcharToFloat(CorrectionData,3) *180 /PI;
            FlightControl.ArmPower =    ArmPowerOFF;
        }
        else if(CorrectionData[2]==0xEE && RT_Info.VioAbnormal == false  &&  FlightControl.landFlag==0)
        {
            Target_Info.RoutePlanX =UnsignedcharToFloat(CorrectionData,3);
            Target_Info.RoutePlanY =UnsignedcharToFloat(CorrectionData,7);
            Target_Info.RoutePlanZ =UnsignedcharToFloat(CorrectionData,11);
            FlightControl.ArmPower =    ArmPowerOFF;
        }
        else if(CorrectionData[2]==0xFF)
        {
           FlightControl.ArmPower =   ArmPowerON;
        }
    }
}

void SendTakeOffFlag(void){
    unsigned char dataToARM[11] = "Departures\n";
    scic_msg(dataToARM);
}

void SendT265StartFlag(void){
    unsigned char dataToARM[9] = "Start265\n";
    scic_msg(dataToARM);
}

void RestartT265(void){
    unsigned char dataToARM[11] = "Refresh265\n";
    scic_msg(dataToARM);
}





