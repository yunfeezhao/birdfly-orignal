/*
 * ProcessRemoteData.c
 *
 *  Created on: 2019��7��27��
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

           // ͨ��1����Roll
           /* �м�������λĬ��Ŀ��Ƕ�Ϊ0�� */
           if(Control_Info.CH1 <= Control_Info.CH1_Mid + 80 &&  Control_Info.CH1 >= Control_Info.CH1_Mid - 80)
           {
               Target_Info_Roll = 0;
           }
           else
           {
               /* Roll���Ʒ�ΧΪ -26��~ 26�㣬�޸���ֵ16�ɸı䷶Χ */
               Target_Info_Roll = (Control_Info.CH1 -Control_Info.CH1_Mid)/15;
           }
           // ͨ��2����pitch
           /* �м�������λĬ��Ŀ��Ƕ�Ϊ0�� */
           if(Control_Info.CH2 <= Control_Info.CH2_Mid + 80 &&  Control_Info.CH2 >= Control_Info.CH2_Mid - 80)
           {
               Target_Info_Pitch = 0;
           }
           else
           {
               /* pitch���Ʒ�ΧΪ -26��~ 26�㣬�޸���ֵ16�ɸı䷶Χ */
               Target_Info_Pitch = (Control_Info.CH2 - Control_Info.CH2_Mid)/15 ;
           }


           if(Control_Info.CH3 <=  Control_Info.CH3_Mid + 80 &&  Control_Info.CH3 >=  Control_Info.CH3_Mid - 80)
           {
               Target_Info.RemoteSpeedZ = 0.0f;
           }
           else
           {
               /*�߶ȵ��ٶȿ��ƣ�����0.7m/s����½� 0.35m/s���*/
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
               /*Yaw���Ʒ�ΧΪ -1rad/s~ 1rad/s���޸���ֵ0.15�ɸı䷶Χ*/
               float temp = (float )Control_Info.CH4 ;
               Target_Info.RemoteYaw = -(temp- Control_Info.CH4_Mid)* 0.15f;
           }

           Target_Info.RemotePitch = Target_Info_Pitch;
           Target_Info.RemoteRoll = Target_Info_Roll;

            // һ����ͣ
            if( Control_Info.CH5 <= Control_Info.CH5_Min + 80  &&  Control_Info.CH5 >= Control_Info.CH5_Min-80)
            {
               FlightControl.OnOff = Drone_Off;
            }
            // ����ģʽѡ�� һ�����
            else if( Control_Info.CH5 <= Control_Info.CH5_Mid + 80  &&  Control_Info.CH5 >= Control_Info.CH5_Mid-80)
            {
               FlightControl.DroneMode=Drone_Mode_4Axis;
               FlightControl.OnOff = Drone_On;
            }
            // һ������
            else if( Control_Info.CH5 <= Control_Info.CH5_Max + 80  &&  Control_Info.CH5 >= Control_Info.CH5_Max-80)
            {
               FlightControl.landFlag=1;
            }


          // ����ģʽѡ��
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


