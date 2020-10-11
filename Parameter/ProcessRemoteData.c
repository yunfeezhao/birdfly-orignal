/*
 * ProcessRemoteData.c
 *
 *  Created on: 2019年7月27日
 *      Author: Jccao
 */
#include "ProcessRemoteData.h"

void Process_RemoteData(Uint16 *RemoteData)
{
    float  Target_Info_Roll ,Target_Info_Pitch=0;

    Control_Info.CH1 = RemoteData[0];
    Control_Info.CH2 = RemoteData[1];
    Control_Info.CH3 = RemoteData[2];
    Control_Info.CH4 = RemoteData[3];
    Control_Info.CH5 = RemoteData[4];
    Control_Info.CH6 = RemoteData[5];
    Control_Info.CH7 = RemoteData[6];

    if( Control_Info.CH1>=(Control_Info.CH1_Min -10)  && (Control_Info.CH1<=Control_Info.CH1_Max + 10)
           && Control_Info.CH2>=(Control_Info.CH2_Min -10)  && (Control_Info.CH2<=Control_Info.CH2_Max + 10)
           && Control_Info.CH3>=(Control_Info.CH3_Min -10)  && (Control_Info.CH3<=Control_Info.CH3_Max + 10)
           && Control_Info.CH4>=(Control_Info.CH4_Min -10)  && (Control_Info.CH4<=Control_Info.CH4_Max + 10)
           && Control_Info.CH5>=(Control_Info.CH5_Min -10)  && (Control_Info.CH5<=Control_Info.CH5_Max + 10)
           && Control_Info.CH6>=(Control_Info.CH6_Min -10)  && (Control_Info.CH6<=Control_Info.CH6_Max + 10)
           && Control_Info.CH7>=(Control_Info.CH7_Min -10)  && (Control_Info.CH7<=Control_Info.CH7_Max + 10))
    {

           // 通道1控制Roll
           /* 中间两个档位默认目标角度为0° */
           if(Control_Info.CH1 <= Control_Info.CH1_Mid + 80 &&  Control_Info.CH1 >= Control_Info.CH1_Mid - 80)
           {
               Target_Info_Roll = 0;
           }
           else
           {
               /* Roll控制范围为 -26°~ 26°，修改数值16可改变范围 */
               Target_Info_Roll = (Control_Info.CH1 -Control_Info.CH1_Mid)/15;
           }
           // 通道2控制pitch
           /* 中间两个档位默认目标角度为0° */
           if(Control_Info.CH2 <= Control_Info.CH2_Mid + 80 &&  Control_Info.CH2 >= Control_Info.CH2_Mid - 80)
           {
               Target_Info_Pitch = 0;
           }
           else
           {
               /* pitch控制范围为 -26°~ 26°，修改数值16可改变范围 */
               Target_Info_Pitch = (Control_Info.CH2 - Control_Info.CH2_Mid)/15 ;
           }


           if(Control_Info.CH3 <=  Control_Info.CH3_Mid + 80 &&  Control_Info.CH3 >=  Control_Info.CH3_Mid - 80)
           {
               Target_Info.RemoteSpeedZ = 0.0f;
           }
           else
           {
               /*高度的速度控制，上升0.7m/s最大，下降 0.35m/s最大*/
               float temp = (float )Control_Info.CH3 ;
               if(temp >= Control_Info.CH3_Mid + 80)
                   Target_Info.RemoteSpeedZ = (temp- Control_Info.CH3_Mid)* 0.15f;
               else if(temp <= Control_Info.CH3_Mid - 80  )
                   Target_Info.RemoteSpeedZ = (temp- Control_Info.CH3_Mid)* 0.08f;
           }


           if(Control_Info.CH4 <=  Control_Info.CH4_Mid + 80 &&  Control_Info.CH4 >=  Control_Info.CH4_Mid - 80)
           {
               Target_Info.RemoteYaw = 0.0f;
           }
           else
           {
               /*Yaw控制范围为 -1rad/s~ 1rad/s，修改数值0.15可改变范围*/
               float temp = (float )Control_Info.CH4 ;
               Target_Info.RemoteYaw = -(temp- Control_Info.CH4_Mid)* 0.15f;
           }

           Target_Info.RemotePitch = Target_Info_Pitch;
           Target_Info.RemoteRoll = Target_Info_Roll;

            // 一键急停
            if( Control_Info.CH5 <= Control_Info.CH5_Min + 80  &&  Control_Info.CH5 >= Control_Info.CH5_Min-80)
            {
               FlightControl.OnOff = Drone_Off;
            }
            // 飞行模式选择 一件起飞
            else if( Control_Info.CH5 <= Control_Info.CH5_Mid + 80  &&  Control_Info.CH5 >= Control_Info.CH5_Mid-80)
            {
               FlightControl.DroneMode=Drone_Mode_4Axis;
               FlightControl.OnOff = Drone_On;
            }
            // 一键降落
            else if( Control_Info.CH5 <= Control_Info.CH5_Max + 80  &&  Control_Info.CH5 >= Control_Info.CH5_Max-80)
            {
               FlightControl.landFlag=1;
            }


          // 飞行模式选择
          if( Control_Info.CH6 <= Control_Info.CH6_Min + 80  &&  Control_Info.CH6 >= Control_Info.CH6_Min-80)
          {
              if(Flight_Remote_Control.FlightControlMode !=   Flight_Remote_Control.CH6_Mode1 )
              {
                  //Beep_RingsOnce();
                  Flight_Remote_Control.FlightControlMode =   Flight_Remote_Control.CH6_Mode1 ;
              }
          }
          else if( Control_Info.CH6 <= Control_Info.CH6_Mid + 80  &&  Control_Info.CH6 >= Control_Info.CH6_Mid-80)
          {
              if(Flight_Remote_Control.FlightControlMode !=   Flight_Remote_Control.CH6_Mode2 )
              {
                  //Beep_RingsOnce();
                  Flight_Remote_Control.FlightControlMode =   Flight_Remote_Control.CH6_Mode2 ;
              }
          }
          else if( Control_Info.CH6 <= Control_Info.CH6_Max + 80  &&  Control_Info.CH6 >= Control_Info.CH6_Max-80)
          {
              if(Flight_Remote_Control.FlightControlMode !=   Flight_Remote_Control.CH6_Mode3 )
              {
                  //Beep_RingsOnce();
                  Flight_Remote_Control.FlightControlMode =   Flight_Remote_Control.CH6_Mode3 ;
              }
          }


    }


}


