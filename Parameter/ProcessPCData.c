/*
 * ProcessPCData.c
 *
 *  Created on: 2018骞�4鏈�24鏃�
 *      Author: Xiluna Tech
 */

#include "ProcessPCData.h"

float_union HexToFloat;

Acce_Unit acce_sample[6]={0};//涓夎6鍒楋紝淇濆瓨6闈㈠緟鐭鏁版嵁

Acce_Unit new_offset={
  0,0,0,
};
Acce_Unit new_scales={
  1.0,1.0,1.0,
};

float UnsignedcharToFloat(Uint16 *DataofPC,unsigned char sequence){
    HexToFloat.sv[0] = (((Uint16)DataofPC[sequence+1]) << 8 | DataofPC[sequence]);
    HexToFloat.sv[1] = (((Uint16)DataofPC[sequence+3]) << 8 | DataofPC[sequence+2]);
    return HexToFloat.fv;
}

void Process_PCData(Uint16 *PCData){
    unsigned char cntRx =0;
    for(cntRx=0 ; cntRx<32 ; cntRx++)
    {

    /* 椋炶鎺у埗鎸囦护  */
    if(PCData[cntRx]==0x55 && PCData[cntRx+1]==0xAA){
        /*鍚姩鍏抽棴闄嶈惤鏃犱汉鏈�*/
        if(PCData[cntRx+2]==0xff  && RT_Info.lowPowerFlag==0){
            if(PCData[cntRx+3]==0){
            FlightControl.OnOff = Drone_Off;
            OriginalPitchRate.iOut = 0;
            OriginalRollRate.iOut = 0;
            OriginalYaw.iOut = 0;
            OriginalVelZ.iOut = 0;
            OriginalAccZ.iOut = 0;
            OriginalVelX.iOut = 0;
            OriginalVelY.iOut = 0;
            OriginalFlowVelX.iOut =0;
            OriginalFlowVelY.iOut =0;
            Target_Info.Height = 1.0f;
            Target_Info.Pitch = 0.0f;
            Target_Info.Roll = 0.0f;
            Target_Info.Yaw = 0.0f;
            Target_Info.RoutePlanX = 0.0f;
            Target_Info.RoutePlanY = 0.0f;
            FlightControl.ArmPower =    ArmPowerON;
            Flight_Remote_Control.FlightControlMode =  VIOCruise;
            RestartT265();
            RT_Info.CpuTick=0;
            DELAY_US(500*10);
            }
            else if(PCData[cntRx+3]==1){
                FlightControl.OnOff = Drone_On;
                FlightControl.TakeOff = Complete_TakingOff;
                Target_Info.Yaw = RT_Info.Yaw;
            }
            else if(PCData[cntRx+3]==2){
                FlightControl.landFlag=1;
            }
        }
        /*骞宠　鏈ㄥ疄楠屽鐜弬鏁�*/
        else if(PCData[cntRx+2]==0x01 && FlightControl.DroneMode!=Drone_Mode_4Axis )
        {
            /*  Target_Pitch */
            Target_Info.Pitch = UnsignedcharToFloat(PCData,cntRx+3);
            /*  Target_Roll */
            Target_Info.Roll = UnsignedcharToFloat(PCData,cntRx+7);
            /*  Target_Yaw */
            Target_Info.Yaw = UnsignedcharToFloat(PCData,cntRx+11);
        }
        /*閫夋嫨瀹為獙椋炶妯″紡鍚屾椂閰嶇疆鐩稿簲鐨勫熀纭�閲�*/
        else if(PCData[cntRx+2]==0x02)
        {
            switch(PCData[cntRx+3])
            {
                case 0:
                    FlightControl.DroneMode=Drone_Mode_None;
                    break;
                case 1:
                    FlightControl.DroneMode=Drone_Mode_RatePitch;
                    break;
                case 2:
                    FlightControl.DroneMode=Drone_Mode_RateRoll;
                    break;
                case 3:
                    FlightControl.DroneMode=Drone_Mode_Pitch;
                    break;
                case 4:
                    FlightControl.DroneMode=Drone_Mode_Roll;
                    break;
                case 5:
                    FlightControl.DroneMode=Drone_Mode_4Axis;
                    break;
                default:
                    break;
            }
        }
        /*璇诲彇PID*/
        else if(PCData[cntRx+2]==0x03)
        {
            FlightControl.ReportSW = Report_SET;
        }
        /*璁剧疆鐩爣Rate*/
        else if(PCData[cntRx+2]==0x04){
            Target_Info.RatePitch = UnsignedcharToFloat(PCData,cntRx+3)/2;
            Target_Info.RateRoll = UnsignedcharToFloat(PCData,cntRx+7)/2;
        }
        /*椋炶鍣ㄩ琛岃繃绋嬩腑鎵嬫焺鐨勫��(pitch roll height yaw)*/
        else if(PCData[cntRx+2]==0x05){
            RockerControl.XaxisPos =  (float)( (int16)( ( (int16)PCData[cntRx+4]<<8 ) + PCData[cntRx+3] ) );
            RockerControl.YaxisPos = (float)( (int16)(( (int16)PCData[cntRx+6]<<8 ) + PCData[cntRx+5]) );
            RockerControl.Navigation = (float)( (int16)(( (int16)PCData[cntRx+8]<<8 ) + PCData[cntRx+7]) );
            RockerControl.ZaxisPos = (float)( (int16)(( (int16)PCData[cntRx+10]<<8 ) + PCData[cntRx+9]) );
        }
//        /*椋炶鍣ㄩ琛岃繃绋嬩腑鎵嬫焺鐨勫��(height yaw)*/
//        else if(PCData[2]==0x06){
//            RockerControl.Navigation = UnsignedcharToFloat(PCData,3);
//            RockerControl.ZaxisPos = UnsignedcharToFloat(PCData,7);
//        }
        /* 璁剧疆Pitch鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x07){
            PID_ParaInfo.Pitch.Kp= UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.Pitch.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.Pitch.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆ROll鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x08){
            PID_ParaInfo.Roll.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.Roll.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.Roll.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆Yaw鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x09){
            PID_ParaInfo.Yaw.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.Yaw.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.Yaw.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆Height鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x0A){
            PID_ParaInfo.PosZ.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.PosZ.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.PosZ.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆accPitch鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x0B){
            PID_ParaInfo.PitchRate.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.PitchRate.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.PitchRate.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆accRoll鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x0C){
            PID_ParaInfo.RollRate.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.RollRate.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.RollRate.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆accYaw鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x0D){
            PID_ParaInfo.YawRate.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.YawRate.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.YawRate.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆accHeight鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x0E){
            PID_ParaInfo.VelZ.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.VelZ.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.VelZ.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆AccZ鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x0F){
            PID_ParaInfo.AccZ.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.AccZ.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.AccZ.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆PositionX鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x12){
            PID_ParaInfo.PosX.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.PosX.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.PosX.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆PositionY鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x13){
            PID_ParaInfo.PosY.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.PosY.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.PosY.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆PositionVX鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x14){
            PID_ParaInfo.VelX.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.VelX.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.VelX.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆PositionVY鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x15){
            PID_ParaInfo.VelY.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.VelY.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.VelY.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆FlowX鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x16){
            PID_ParaInfo.FlowX.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.FlowX.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.FlowX.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆FlowVelX鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x17){
            PID_ParaInfo.FlowVelX.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.FlowVelX.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.FlowVelX.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆FlowY鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x18){
            PID_ParaInfo.FlowY.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.FlowY.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.FlowY.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            FlightControl.ReportSW = Report_SET;
            Write_Config();
        }
        /* 璁剧疆FlowVelY鐨凱ID鍙傛暟 */
        else if(PCData[cntRx+2]==0x19){
            PID_ParaInfo.FlowVelY.Kp = UnsignedcharToFloat(PCData,cntRx+3);
            PID_ParaInfo.FlowVelY.Ki = UnsignedcharToFloat(PCData,cntRx+7);
            PID_ParaInfo.FlowVelY.Kd = UnsignedcharToFloat(PCData,cntRx+11);
            Write_Config();
            FlightControl.ReportSW = Report_SET;
        }
        /*闄�铻轰华鏍″噯*/
        else if(PCData[cntRx+2]==0x30)
        {
            OffsetData.GyroX  = RT_Info.GyroX ;
            OffsetData.GyroY  = RT_Info.GyroY ;
            OffsetData.GyroZ  = RT_Info.GyroZ ;
            Write_Config();
        }

        /*鍋忕疆瑙掑害*/
        else if(PCData[cntRx+2] == 0x32)
        {
            Errangle_Info.fixedErroPitch = RT_Info.Pitch;
            Errangle_Info.fixedErroRoll = RT_Info.Roll;
            Write_Config();
            FlightControl.ReportSW=Report_SET;
        }

        /*鍔犻�熷害鏍″噯*/
                /*绗竴闈㈤鎺у钩鏀撅紝Z杞存鍚戞湞鐫�姝ｄ笂鏂癸紝Z axis is about 1g,X銆乊 is about 0g*/
                /*绗簩闈㈤鎺у钩鏀撅紝X杞存鍚戞湞鐫�姝ｄ笂鏂癸紝X axis is about 1g,Y銆乑 is about 0g*/
                /*绗笁闈㈤鎺у钩鏀撅紝X杞存鍚戞湞鐫�姝ｄ笅鏂癸紝X axis is about -1g,Y銆乑 is about 0g*/
                /*绗洓闈㈤鎺у钩鏀撅紝Y杞存鍚戞湞鐫�姝ｄ笅鏂癸紝Y axis is about -1g,X銆乑 is about 0g*/
                /*绗簲闈㈤鎺у钩鏀撅紝Y杞存鍚戞湞鐫�姝ｄ笂鏂癸紝Y axis is about 1g,X銆乑 is about 0g*/
                /*绗叚闈㈤鎺у钩鏀撅紝Z杞存鍚戞湞鐫�姝ｄ笅鏂癸紝Z axis is about -1g,X銆乊 is about 0g*/
                else if(PCData[cntRx+2]==0x31)
                {
                        unsigned char Pretime =0 ;
                        static int16_t AccData[6] ={0};
                        static float PreparationAccX,PreparationAccY,PreparationAccZ =0;
                        Pretime = 0 ;
                        if(PCData[cntRx+3]==0x01)
                        {
                            while( Pretime < 100)
                            {
                                    Pretime ++;
                                    MPU6500_readGyro_Acc(&AccData[3],&AccData[0]);
                                    PreparationAccX += (float)AccData[0] /4096 * GRAVITY_MSS;
                                    PreparationAccY += (float)AccData[1] /4096 * GRAVITY_MSS;
                                    PreparationAccZ += (float)AccData[2] /4096 * GRAVITY_MSS;
                                    OSTimeDly(4);
                            }
                            acce_sample[0].x = PreparationAccX /100 ;
                            acce_sample[0].y = PreparationAccY /100 ;
                            acce_sample[0].z = PreparationAccZ /100 ;
                            PreparationAccX = PreparationAccY = PreparationAccZ =0;
                        }
                        else if(PCData[cntRx+3]==0x02)
                        {
                            while( Pretime < 100)
                            {
                                    Pretime ++;
                                    MPU6500_readGyro_Acc(&AccData[3],&AccData[0]);
                                    PreparationAccX += (float)AccData[0] /4096 * GRAVITY_MSS ;
                                    PreparationAccY += (float)AccData[1] /4096 * GRAVITY_MSS;
                                    PreparationAccZ += (float)AccData[2] /4096 * GRAVITY_MSS;
                                    OSTimeDly(4);
                            }
                            acce_sample[5].x = PreparationAccX /100 ;
                            acce_sample[5].y = PreparationAccY /100 ;
                            acce_sample[5].z = PreparationAccZ /100 ;
                            PreparationAccX = PreparationAccY = PreparationAccZ =0;
                        }
                        else if(PCData[cntRx+3]==0x03)
                        {
                            while( Pretime < 100)
                            {
                                    Pretime ++;
                                    MPU6500_readGyro_Acc(&AccData[3],&AccData[0]);
                                    PreparationAccX += (float)AccData[0] /4096 * GRAVITY_MSS;
                                    PreparationAccY += (float)AccData[1] /4096 * GRAVITY_MSS;
                                    PreparationAccZ += (float)AccData[2] /4096 * GRAVITY_MSS;
                                    OSTimeDly(4);
                            }
                            acce_sample[2].x = PreparationAccX /100 ;
                            acce_sample[2].y = PreparationAccY /100 ;
                            acce_sample[2].z = PreparationAccZ /100 ;
                            PreparationAccX = PreparationAccY = PreparationAccZ =0;
                        }
                        else if(PCData[cntRx+3]==0x04)
                        {
                            while( Pretime < 100)
                            {
                                    Pretime ++;
                                    MPU6500_readGyro_Acc(&AccData[3],&AccData[0]);
                                    PreparationAccX += (float)AccData[0] /4096 * GRAVITY_MSS;
                                    PreparationAccY += (float)AccData[1] /4096 * GRAVITY_MSS;
                                    PreparationAccZ += (float)AccData[2] /4096 * GRAVITY_MSS;
                                    OSTimeDly(4);
                            }
                            acce_sample[1].x = PreparationAccX /100 ;
                            acce_sample[1].y = PreparationAccY /100 ;
                            acce_sample[1].z = PreparationAccZ /100 ;
                            PreparationAccX = PreparationAccY = PreparationAccZ =0;
                        }
                        else if(PCData[cntRx+3]==0x05)
                        {
                            while( Pretime < 100)
                            {
                                    Pretime ++;
                                    MPU6500_readGyro_Acc(&AccData[3],&AccData[0]);
                                    PreparationAccX += (float)AccData[0] /4096 * GRAVITY_MSS;
                                    PreparationAccY += (float)AccData[1] /4096 * GRAVITY_MSS;
                                    PreparationAccZ += (float)AccData[2] /4096 * GRAVITY_MSS;
                                    OSTimeDly(4);
                            }
                            acce_sample[3].x = PreparationAccX /100 ;
                            acce_sample[3].y = PreparationAccY /100 ;
                            acce_sample[3].z = PreparationAccZ /100 ;
                            PreparationAccX = PreparationAccY = PreparationAccZ =0;
                        }
                        else if(PCData[cntRx+3]==0x06)
                        {
                            while( Pretime < 100)
                            {
                                    Pretime ++;
                                    MPU6500_readGyro_Acc(&AccData[3],&AccData[0]);
                                    PreparationAccX += (float)AccData[0] /4096 * GRAVITY_MSS;
                                    PreparationAccY += (float)AccData[1] /4096 * GRAVITY_MSS;
                                    PreparationAccZ += (float)AccData[2] /4096 * GRAVITY_MSS;
                                    OSTimeDly(4);
                            }
                            acce_sample[4].x = PreparationAccX /100 ;
                            acce_sample[4].y = PreparationAccY /100 ;
                            acce_sample[4].z = PreparationAccZ /100 ;
                            PreparationAccX = PreparationAccY = PreparationAccZ =0;
                        }
                        else if(PCData[cntRx+3]==0xFF)
                        {
                            while(!Calibrate_accel(acce_sample,&new_offset,&new_scales));
                            OffsetData.AccX = new_offset.x;
                            OffsetData.AccY = new_offset.y;
                            OffsetData.AccZ = new_offset.z;
                            OffsetData.AccXScale = new_scales.x ;
                            OffsetData.AccYScale = new_scales.y ;
                            OffsetData.AccZScale = new_scales.z ;
                            Write_Config();
                        }

                }

        else if(PCData[cntRx+2]==0x33) //鏍″噯纾佸姏璁�
        {
            if(PCData[cntRx+3]==1)
            {
                OffsetData.MagOffseting = true;
               // LSM303_Start_Calib();
            }
            else if (PCData[cntRx+3]==2)
            {
                OffsetData.MagOffseting = false;
               // LSM303_Save_Calib();
                Write_Config();
            }
        }
        cntRx = 32;
    }
    }
}


