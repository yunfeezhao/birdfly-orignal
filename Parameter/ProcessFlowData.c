/*
 * ProcessFlowData.c
 *
 *  Created on: 2019年7月17日
 *      Author: Jccao
 */

#include "ProcessFlowData.h"

Butter_Parameter FlowX_Parameter;
Butter_BufferData  flowX_filter_buf[3];
Butter_Parameter FlowY_Parameter;
Butter_BufferData  flowY_filter_buf[3];
Butter_Parameter RotateFlowX_Parameter;
Butter_BufferData  RotateFlowX_filter_buf[3];
Butter_Parameter RotateFlowY_Parameter;
Butter_BufferData  RotateFlowY_filter_buf[3];

void Process_FlowData(Uint16 *FlowData){
//    uint8_t Check_sum = 0;
    unsigned char cntRx =0;
    int16_t tmpX,tmpY =0;
    float ZaixsScale = 1.0f;
    float RotateScale = 230.0f;
    float RotateX,RotateY =0;
    float FlowTime =0;

    for(cntRx=0 ; cntRx<28 ; cntRx++)
    {
        if(FlowData[cntRx]==0xfe  && FlowData[cntRx+1]==0x0A && FlowData[cntRx+13]==0x55 )
        {
            if(FlowData[cntRx+10]==0xf5)
            {
                tmpX = ( (FlowData[cntRx+3]<<8) + FlowData[cntRx+2] ) ;
                tmpY = ( (FlowData[cntRx+5]<<8) + FlowData[cntRx+4] ) ;
                FlowTime = 0.02;//(float)( (FlowData[cntRx+7]<<8) + ( FlowData[cntRx+6] && 0x00ff))/1000000;

                ZaixsScale = RT_Info.Height;

                Sensor_Info.FlowVelX = LPButterworth((float)tmpY,&flowX_filter_buf[0],&FlowX_Parameter) ;
                Sensor_Info.FlowVelY = -LPButterworth((float)tmpX,&flowY_filter_buf[0],&FlowY_Parameter) ;


                RotateX = RotateScale *  Limits_data(  ((LPButterworth(RT_Info.GyroX,&RotateFlowX_filter_buf[0],&RotateFlowX_Parameter) )* PI/180) ,3.0f,-3.0f);
                RotateY = RotateScale *  Limits_data(  ((LPButterworth(RT_Info.GyroY,&RotateFlowY_filter_buf[0],&RotateFlowY_Parameter)) * PI/180) ,3.0f,-3.0f);

                RT_Info.LPFTest1 = RotateX;
                RT_Info.LPFTest2 = RotateY;

                Sensor_Info.FixFlowX = ( Sensor_Info.FlowVelX - RotateX ) /10000 /FlowTime *ZaixsScale ;
                Sensor_Info.FixFlowY = ( Sensor_Info.FlowVelY - RotateY ) /10000 /FlowTime *ZaixsScale ;

                Sensor_Info.FlowX += Sensor_Info.FixFlowX * FlowTime;
                Sensor_Info.FlowY += Sensor_Info.FixFlowY * FlowTime;
            }
            else
            {
                Sensor_Info.FixFlowX =0;
                Sensor_Info.FixFlowY =0;
            }
            cntRx = 28;
        }
    }
}




