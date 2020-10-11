/*
 * task.h
 *
 *  Created on: 2019年3月13日
 *      Author: Jccao
 */

#ifndef TASK_H_
#define TASK_H_


#include "F28x_Project.h"
#include <app_cfg.h>
#include <ucos_ii.h>
#include <cpu_core.h>
#include <lib_def.h>
#include <stdbool.h>
#include "C28x_BSP.h"
#include "C28x_CPU.h"
#include "AHRS_Hardware.h"
#include "PID_Control.h"
#include "PositionEstimation.h"
#include "DataToPC.h"
#include "ProcessPCData.h"
#include "ProcessVisionData.h"
#include "ProcessFlowData.h"
#include "ADC_Battery.h"
#include "SimpleDigitalFiltering.h"
//#include "ProcessRemoteData.h"
#include "Position_control.h"
#include "Attitude_control.h"
#include "FlashAPI.h"
#include "MPU6500.h"
//#include "LSM303D.h"
//#include "I2C.h"
//#include "MS5611.h"
#include "Key.h"
#include "MahonyAHRS.h"
#include "ProcessLaserData.h"
#include "DronePara.h"
#include "Calibrate.h"
#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
//#include "IST8310.h"
#include "mathTool.h"
#include "matrix3.h"
#include <math.h>


//信号量
extern OS_EVENT ProcessPCData_proc;//信号量
extern Uint16 RecivePCData[32];
extern OS_EVENT ProcessVisionData_proc;//信号量
extern Uint16 ReciveVisionData[32];
extern OS_EVENT ProcessFlowData_proc;//信号量
extern Uint16 ReciveFlowData[28];
//extern OS_EVENT RemoteControl_proc;//信号量
//extern Uint16 Receive_PPM_In[9];
extern OS_EVENT ProcessLaserData_proc;//信号量
extern unsigned char ReciveLaserData[16];



//全局外部变量
extern DroneFlightControl FlightControl;
extern DroneRTInfo RT_Info;
extern DroneErrangle Errangle_Info;
extern DroneTargetInfo Target_Info;
extern RemoteControl RockerControl;
extern Controller Control_Info;
extern Remote_Control  Flight_Remote_Control;
extern RemoteSensing   Remote_Sensing;
extern OffsetInfo OffsetData;
extern FlyMode Fly_Mode;
extern Thrust UAVThrust;
extern Throttle Throttle_Info;
extern SensorData Sensor_Info;
//控制参数
extern PIDOut OriginalPitch,OriginalRoll,OriginalYaw,OriginalPosX,OriginalPosY,OriginalPosZ,
                    OriginalPitchRate,OriginalRollRate,OriginalYawRate,OriginalVelX,OriginalVelY,OriginalVelZ,
                        OriginalFlowX,OriginalFlowY,OriginalFlowVelX,OriginalFlowVelY,
                                                    OriginalAccZ;
extern PIDPara PID_ParaInfo;
//融合参数
extern KalmanFilter XAxis,YAxis,ZAxis,Barometer,FlowX,FlowY;
extern Butter_Parameter FlowX_Parameter;
extern Butter_Parameter FlowY_Parameter;
//extern Butter_Parameter VIO_Parameter;
//extern Butter_Parameter RotateX_Parameter;
//extern Butter_Parameter RotateY_Parameter;
//extern Butter_Parameter RotateZ_Parameter;
extern Butter_Parameter RotateFlowX_Parameter;
extern Butter_Parameter RotateFlowY_Parameter;
//extern Butter_Parameter LineAngel_Parameter;

#define Drone_Wheelbase_330
//#define LaunchPad_PinConfig
//#define Remote     //遥控器控制
//#define 3RingControl  //是否定义高度三环
#define ICM20689

#endif /* TASK_H_ */
