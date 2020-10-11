 /******************* (C) COPYRIGHT 2018- Xiluna Tech ************************
 * 作者    ：Xiluna Tech
 * 文件名  ：Bird_main.c
 * 描述    ：High performance UAV
 * 官网    ：http://xiluna.com/
 * 公众号  ：XilunaTech
 * gitlab ：git.xiluna.com
 * 已经修改
*****************************************************************************/
#include "task.h"

//全局变量
DroneRTInfo RT_Info;                                                                     //四旋翼实时数据
DroneTargetInfo Target_Info;                                                             //四旋翼目标全局变量
DroneErrangle Errangle_Info;                                                             //四旋翼平地校准数据
DroneFlightControl FlightControl;                                                        //四旋翼状态变量
RemoteControl RockerControl;                                                             //四旋翼摇杆变量
Remote_Control  Flight_Remote_Control;                                                   //遥控器飞行设置
RemoteSensing   Remote_Sensing;                                                          //外部控制信号设置
Controller Control_Info;                                                                 //遥控器控制全局变量
SensorData Sensor_Info;                                                                  //四旋翼定位传感器数据
FlyMode Fly_Mode;                                                                        //四旋翼飞行模式
OffsetInfo OffsetData;                                                                   //磁偏量
Thrust UAVThrust;                                                                        //飞行器扭力计算
Throttle Throttle_Info;                                                                  //飞行器扭力输出
//控制参数
PIDOut OriginalPitch,OriginalRoll,OriginalYaw,OriginalPosX,OriginalPosY,OriginalPosZ,
                    OriginalPitchRate,OriginalRollRate,OriginalYawRate,OriginalVelX,OriginalVelY,OriginalVelZ,
                        OriginalFlowX,OriginalFlowY,OriginalFlowVelX,OriginalFlowVelY,OriginalAccZ;
PIDPara PID_ParaInfo;
//融合参数
KalmanFilter XAxis,YAxis,ZAxis,Barometer,FlowX,FlowY;                                                                             //卡尔曼滤波融合参数

/*
*********************************************************************************************************
*                                         LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
/* Start Task's stack.                               */
CPU_STK_SIZE  App_TaskStartStk[APP_CFG_TASK_STK_SIZE];
/* IMU Task's stack.                                 */
CPU_STK_SIZE  App_TaskIMUStk[APP_CFG_TASK_STK_SIZE];
/* Attitude Task's stack.                            */
CPU_STK_SIZE  App_TaskAttitudeStk[APP_CFG_TASK_STK_SIZE];
/* Position Task's stack.                            */
CPU_STK_SIZE  App_TaskPositionStk[APP_CFG_TASK_STK_SIZE];
/* Combine  Task's stack.                            */
CPU_STK_SIZE  App_TaskCombineStk[APP_CFG_TASK_STK_SIZE];
/* ProcessVisionData Task's stack.                   */
CPU_STK_SIZE  App_TaskProcessVisionDataStk[APP_CFG_TASK_STK_SIZE];
/* ProcessPCData Task's stack.                       */
CPU_STK_SIZE  App_TaskProcessPCDataStk[APP_CFG_TASK_STK_SIZE];
/* DataToPC Task's stack.                            */
CPU_STK_SIZE  App_TaskDataToPCStk[APP_CFG_TASK_STK_SIZE];
/* Battery Task's stack.                             */
CPU_STK_SIZE  App_TaskBatteryStk[APP_CFG_TASK_STK_SIZE];
///* RemoteControl Task's stack.                                 */
//CPU_STK_SIZE  App_TaskRemoteControlStk[APP_CFG_TASK_STK_SIZE];
/* Flow Task's stack.                                 */
CPU_STK_SIZE  App_TaskFlowStk[APP_CFG_TASK_STK_SIZE];
/* Laser Task's stack.                                 */
CPU_STK_SIZE  App_TaskLaserStk[APP_CFG_TASK_STK_SIZE];


/*
*********************************************************************************************************
*                                          FUNCTION PROTOTYPES
*********************************************************************************************************
*/
/* Start Task */
static  void  App_TaskStart(void  *p_arg);
/* IMU Task */
static  void  App_TaskIMU(void  *p_arg);
/* Attitude Task */
static  void  App_TaskAttitude(void  *p_arg);
/* Position Task */
static  void  App_TaskPosition(void  *p_arg);
/* Combine Task */
static  void  App_TaskCombine(void  *p_arg);
/* ProcessVisionData Task */
static  void  App_TaskProcessVisionData(void  *p_arg);
/* ProcessPCData Task */
static  void  App_TaskProcessPCData(void  *p_arg);
/* DataToPC Task */
static  void  App_TaskDataToPC(void  *p_arg);
/* ADC Task */
static  void  App_TaskBattery(void *p_arg);
///* RemoteControl Task */
//static  void  App_TaskRemoteControl(void  *p_arg);
/* Laser Task */
static  void  App_TaskLaser(void  *p_arg);
/* Flow Task */
static  void  App_TaskFlow(void  *p_arg);

int main(void){
/* Initialize the CPU and Board.                        */
    C28x_CPU_Init();
/* Initialize the BSP.                                  */
    C28x_BSP_Init();
/* Initialize the AHRS_HardWare.                        */
    AHRS_HardWareinit();
/* Initialize the KalmanFilter Para.                    */
    KalmanFilter_Init(&XAxis,&YAxis,&ZAxis,&Barometer,&FlowX,&FlowY);

//   Set_Cutoff_Frequency(200.0f, 50.0f,&VIO_Parameter);          // VIO速度巴特沃斯参数初始化
//   Set_Cutoff_Frequency(200.0f, 25.0f,&RotateX_Parameter);       //光流旋转X巴特沃斯参数初始化
//   Set_Cutoff_Frequency(200.0f, 25.0f,&RotateY_Parameter);       //光流旋转Y巴特沃斯参数初始化
//   Set_Cutoff_Frequency(200.0f, 25.0f,&RotateZ_Parameter);       //光流旋转Z巴特沃斯参数初始化
   Set_Cutoff_Frequency(25.0f, 2.0f,&FlowX_Parameter);          // 光流速度巴特沃斯参数初始化
   Set_Cutoff_Frequency(25.0f, 2.0f,&FlowY_Parameter);          // 光流速度巴特沃斯参数初始化
   Set_Cutoff_Frequency(25.0f, 1.2f,&RotateFlowX_Parameter);       //光流旋转X巴特沃斯参数初始化
   Set_Cutoff_Frequency(25.0f, 1.2f,&RotateFlowY_Parameter);       //光流旋转Y巴特沃斯参数初始化
   Fly_Mode = Data_Follow;//Data_Flow;
   RT_Info.VioAbnormal = false;
/* Load Control Para.                                   */
    Load_ParaConfig();
    /* Initialize "uC/OS-II, The Real-Time Kernel".         */
    OSInit();
    /* Create the Start task.                               */
    OSTaskCreateExt(App_TaskStart,
                    (void    *)0,
                    (CPU_STK *)&App_TaskStartStk[0],
                    (INT8U    )APP_CFG_TASK_START_PRIO,
                    (INT16U   )APP_CFG_TASK_START_PRIO,
                    (CPU_STK *)&App_TaskStartStk[APP_CFG_TASK_STK_SIZE - 1u],
                    (INT32U   )APP_CFG_TASK_STK_SIZE,
                    (void    *)0,
                    (INT16U   )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
    /* Start multitasking (i.e. give control to uC/OS-II).  */
     OSStart();
    /* Should never get here.   */
    while(DEF_TRUE){
          ;
    }
}


static  void  App_TaskStart (void *p_arg)
{
    OS_CPU_SR  cpu_sr = 0;
    /* Prevent compiler warning for not using 'p_arg'    */
    (void)&p_arg;

    C28x_BSP_Tick_Init();
    /* Start the Ticker.                                 */
   OS_ENTER_CRITICAL();
    OSTaskCreateExt(App_TaskIMU,
                    (void    *)0,
                    (CPU_STK *)&App_TaskIMUStk[0],
                    (INT8U    )APP_CFG_TASK_IMU_PRIO,
                    (INT16U   )APP_CFG_TASK_IMU_PRIO,
                    (CPU_STK *)&App_TaskIMUStk[APP_CFG_TASK_STK_SIZE - 1u],
                    (INT32U   )APP_CFG_TASK_STK_SIZE,
                    (void    *)0,
                    (INT16U   )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
    /* Attitude Task.                                    */
    OSTaskCreateExt(App_TaskAttitude,
                   (void    *)0,
                   (CPU_STK *)&App_TaskAttitudeStk[0],
                   (INT8U    )APP_CFG_TASK_Attitude_PRIO,
                   (INT16U   )APP_CFG_TASK_Attitude_PRIO,
                   (CPU_STK *)&App_TaskAttitudeStk[APP_CFG_TASK_STK_SIZE - 1u],
                   (INT32U   )APP_CFG_TASK_STK_SIZE,
                   (void    *)0,
                   (INT16U   )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
    /* Position Task.                                    */
    OSTaskCreateExt(App_TaskPosition,
                   (void    *)0,
                   (CPU_STK *)&App_TaskPositionStk[0],
                   (INT8U    )APP_CFG_TASK_Position_PRIO,
                   (INT16U   )APP_CFG_TASK_Position_PRIO,
                   (CPU_STK *)&App_TaskPositionStk[APP_CFG_TASK_STK_SIZE - 1u],
                   (INT32U   )APP_CFG_TASK_STK_SIZE,
                   (void    *)0,
                   (INT16U   )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
    /* Combine Task.                                    */
    OSTaskCreateExt(App_TaskCombine,
                   (void    *)0,
                   (CPU_STK *)&App_TaskCombineStk[0],
                   (INT8U    )APP_CFG_TASK_Combine_PRIO,
                   (INT16U   )APP_CFG_TASK_Combine_PRIO,
                   (CPU_STK *)&App_TaskCombineStk[APP_CFG_TASK_STK_SIZE - 1u],
                   (INT32U   )APP_CFG_TASK_STK_SIZE,
                   (void    *)0,
                   (INT16U   )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
    /* ProcessPCData Task.                                */
    OSTaskCreateExt(App_TaskProcessPCData,
                   (void    *)0,
                   (CPU_STK *)&App_TaskProcessPCDataStk[0],
                   (INT8U    )APP_CFG_TASK_ProcessPCData_PRIO,
                   (INT16U   )APP_CFG_TASK_ProcessPCData_PRIO,
                   (CPU_STK *)&App_TaskProcessPCDataStk[APP_CFG_TASK_STK_SIZE - 1u],
                   (INT32U   )APP_CFG_TASK_STK_SIZE,
                   (void    *)0,
                   (INT16U   )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
    /* ProcessVisionData Task.                           */
    OSTaskCreateExt(App_TaskProcessVisionData,
                   (void    *)0,
                   (CPU_STK *)&App_TaskProcessVisionDataStk[0],
                   (INT8U    )APP_CFG_TASK_ProcessVisionData_PRIO,
                   (INT16U   )APP_CFG_TASK_ProcessVisionData_PRIO,
                   (CPU_STK *)&App_TaskProcessVisionDataStk[APP_CFG_TASK_STK_SIZE - 1u],
                   (INT32U   )APP_CFG_TASK_STK_SIZE,
                   (void    *)0,
                   (INT16U   )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
    /* DataToPC Task.                                    */
    OSTaskCreateExt(App_TaskDataToPC,
                    (void    *)0,
                    (CPU_STK *)&App_TaskDataToPCStk[0],
                    (INT8U    )APP_CFG_TASK_DataToPC_PRIO,
                    (INT16U   )APP_CFG_TASK_DataToPC_PRIO,
                    (CPU_STK *)&App_TaskDataToPCStk[APP_CFG_TASK_STK_SIZE - 1u],
                    (INT32U   )APP_CFG_TASK_STK_SIZE,
                    (void    *)0,
                    (INT16U   )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
    /* Battery Task.                                    */
    OSTaskCreateExt(App_TaskBattery,
                    (void    *)0,
                    (CPU_STK *)&App_TaskBatteryStk[0],
                    (INT8U    )APP_CFG_TASK_Battery_PRIO,
                    (INT16U   )APP_CFG_TASK_Battery_PRIO,
                    (CPU_STK *)&App_TaskBatteryStk[APP_CFG_TASK_STK_SIZE - 1u],
                    (INT32U   )APP_CFG_TASK_STK_SIZE,
                    (void    *)0,
                    (INT16U   )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
//    /* RemoteControl Task.                                        */
//    OSTaskCreateExt(App_TaskRemoteControl,
//                    (void    *)0,
//                    (CPU_STK *)&App_TaskRemoteControlStk[0],
//                    (INT8U    )APP_CFG_TASK_RemoteControl_PRIO,
//                    (INT16U   )APP_CFG_TASK_RemoteControl_PRIO,
//                    (CPU_STK *)&App_TaskRemoteControlStk[APP_CFG_TASK_STK_SIZE - 1u],
//                    (INT32U   )APP_CFG_TASK_STK_SIZE,
//                    (void    *)0,
//                    (INT16U   )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
    OSTaskCreateExt(App_TaskFlow,
                    (void    *)0,
                    (CPU_STK *)&App_TaskFlowStk[0],
                    (INT8U    )APP_CFG_TASK_Flow_PRIO,
                    (INT16U   )APP_CFG_TASK_Flow_PRIO,
                    (CPU_STK *)&App_TaskFlowStk[APP_CFG_TASK_STK_SIZE - 1u],
                    (INT32U   )APP_CFG_TASK_STK_SIZE,
                    (void    *)0,
                    (INT16U   )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
    /* Laser Task.                                        */
    OSTaskCreateExt(App_TaskLaser,
                    (void    *)0,
                    (CPU_STK *)&App_TaskLaserStk[0],
                    (INT8U    )APP_CFG_TASK_Laser_PRIO,
                    (INT16U   )APP_CFG_TASK_Laser_PRIO,
                    (CPU_STK *)&App_TaskLaserStk[APP_CFG_TASK_STK_SIZE - 1u],
                    (INT32U   )APP_CFG_TASK_STK_SIZE,
                    (void    *)0,
                    (INT16U   )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
    OS_EXIT_CRITICAL();
    OSTaskSuspend(APP_CFG_TASK_START_PRIO); //挂起起始任务.
}


/*
*********************************************************************************************************
*                                            App_TaskIMU
*
* Description : imu task
*
* Argument(s) : p_arg       the argument passed by 'OSTaskCreateExt()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*********************************************************************************************************
*/
static  void  App_TaskIMU (void *p_arg){
   /* Prevent compiler warning for not using 'p_arg'       */
   (void)&p_arg;
   /* Task body, always written as an infinite loop.       */
    while (DEF_TRUE) {
        IMU_getInfo();
//        OSTimeDly(2);
        OSTimeDlyHMSM(0,0,0,2);
    }
}
/*
*********************************************************************************************************
*                                            App_TaskAttitude
*
* Description : Attitude task
*
* Argument(s) : p_arg       the argument passed by 'OSTaskCreateExt()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*********************************************************************************************************
*/
static  void  App_TaskAttitude (void *p_arg){
   /* Prevent compiler warning for not using 'p_arg'       */
   (void)&p_arg;
   /* Task body, always written as an infinite loop.       */
   Uint16 Preparationtime = 0;
   float PreparationPitch = 0;
   float PreparationRoll = 0;
//   float PreparationYaw = 0;
   _Bool PreparationFlag = true;
   FlightControl.ControlStart = false;
    while (DEF_TRUE) {
        if(FlightControl.OnOff == Drone_On ){
            if(FlightControl.DroneMode == Drone_Mode_4Axis){
                if(Preparationtime < 500){
                    Preparationtime ++;
                    PreparationPitch += RT_Info.Pitch;
                    PreparationRoll += RT_Info.Roll;
                    PWM_OUTPUT(300,300,300,300);
                }
                else{
                    /*起飞平地自校准*/
                    if(PreparationFlag){
                        PreparationPitch /= 500;
                        PreparationRoll /= 500;
                        RT_Info.FlowX =0 ;
                        RT_Info.FlowY =0 ;
                        PreparationFlag = false;
                        FlightControl.ControlStart = true;
                    }
                    if(FlightControl.ControlStart){
                        Attitude_control(PreparationPitch,PreparationRoll);
                        Safety_Protection();     //侧倾保护
                    }
               }
            }
            else{
                Attitude_control(0,0);
            }
        }
        else{
            PreparationFlag = true;
            PreparationPitch = 0;
            PreparationRoll = 0;
            Preparationtime = 0;
            OriginalPitchRate.iOut = 0;
            OriginalRollRate.iOut = 0;
            OriginalYaw.iOut = 0;
            OriginalVelZ.iOut = 0;
            OriginalAccZ.iOut = 0;
            OriginalVelX.iOut = 0;
            OriginalVelY.iOut = 0;
            OriginalFlowVelX.iOut =0;
            OriginalFlowVelY.iOut =0;
            OriginalVelX.iOut =0;
            OriginalVelY.iOut =0;
            Target_Info.Height = 1.0f;
            Target_Info.Pitch = 0.0f;
            Target_Info.Roll = 0.0f;
            Target_Info.Yaw = 0.0f;
            Target_Info.RoutePlanX = 0.0f;
            Target_Info.RoutePlanY = 0.0f;
            Target_Info.RoutePlanZ = 1.0f;
            Target_Info.PositionX =0;
            Target_Info.PositionY =0;
            RT_Info.VioHeartbeat =0;
            FlightControl.LaunchFlag = true;
            RT_Info.VioAbnormal = false;
            RT_Info.VioHeartbeat =0;
            Flight_Remote_Control.FlightControlMode = VIOCruise ;
            Fly_Mode =  Data_Follow;
            FlightControl.landFlag =0;
            FlightControl.ControlStart = false;
            FlightControl.poshold = 0;
            FlightControl.setTargetPos = 0;
            PWM_OUTPUT(0,0,0,0);
        }
//        OSTimeDly(2);
        OSTimeDlyHMSM(0,0,0,2);
    }
}

/*
*********************************************************************************************************
*                                            App_TaskPosition
*
* Description : Position task
*
* Argument(s) : p_arg       the argument passed by 'OSTaskCreateExt()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*********************************************************************************************************
*/
static  void  App_TaskPosition (void *p_arg){
   /* Prevent compiler warning for not using 'p_arg'       */
   (void)&p_arg;
   /* Task body, always written as an infinite loop.       */
   float Climbing = 0.002f;
   float Declining = 0.002f;
   Target_Info.Height = 0.90f;
    while (DEF_TRUE) {
        if(FlightControl.DroneMode == Drone_Mode_4Axis && FlightControl.OnOff==Drone_On && FlightControl.ControlStart == true){
            Position_control(Fly_Mode,Climbing,Declining);
        if(RT_Info.VioHeartbeat >=100)
        {
                Fly_Mode = Data_Flow;
                RT_Info.VioAbnormal = true;
                FlightControl.poshold = 1;
                FlightControl.setTargetPos = 0;
                Target_Info.Yaw = RT_Info.Yaw;
                Target_Info.RoutePlanZ = 0.6f;
//                  Target_Info.PositionX = 0;
//                  Target_Info.PositionY = 0;
              if(RT_Info.VioHeartbeat >=1500)
                {
                    FlightControl.landFlag=1;
                }

        }
        RT_Info.VioHeartbeat ++;
        }
//        OSTimeDly(5);
        OSTimeDlyHMSM(0,0,0,5);
    }
}


/*
*********************************************************************************************************
*                                            App_TaskCombine
*
* Description : Combine task
*
* Argument(s) : p_arg       the argument passed by 'OSTaskCreateExt()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*********************************************************************************************************
*/
static  void  App_TaskCombine (void *p_arg){
   /* Prevent compiler warning for not using 'p_arg'       */
   (void)&p_arg;
   /* Task body, always written as an infinite loop.       */
    while (DEF_TRUE) {
//        /* 超声波数据的卡尔曼滤波融合 */
//        if(Sensor_Info.US100_Zaxis>=0 && Sensor_Info.US100_Zaxis<=2.0f)
//        {
//            POS_KalmanFilter(&ZAxis,Sensor_Info.US100_Zaxis,RT_Info.accZaxis);
//            RT_Info.Height = ZAxis.Axis_Pos;
//            RT_Info.Height_V = ZAxis.Axis_Vel;
//        }
//        else
//        {
//            Sensor_Info.US100_Zaxis =0;
//        }


        /* 激光数据的卡尔曼滤波融合 */
       POS_KalmanFilter(&ZAxis, Sensor_Info.Laser,RT_Info.accZaxis);
       RT_Info.Height = ZAxis.Axis_Pos;
       RT_Info.Height_V = ZAxis.Axis_Vel;

//        POS_KalmanFilter(&Barometer,Sensor_Info.MS5611_Zaxis,RT_Info.accZaxis);
//        RT_Info.Height = Barometer.Axis_Pos;
//        RT_Info.Height_V = Barometer.Axis_Vel;

        /* 物体追踪的位置数据卡尔曼融合 */
        if(Fly_Mode == Data_Point )
        {
            /*地理坐标系的加速度X正轴  对应 相机的X正轴数据 */
            POS_KalmanFilter(&XAxis,Sensor_Info.Raspberry_Xaxis/100,-RT_Info.AccX);
            RT_Info.PointX = XAxis.Axis_Pos;
            RT_Info.PointX_V = XAxis.Axis_Vel;

            /*地理坐标系的加速度Y负轴  对应 相机的Y正轴数据 */
            POS_KalmanFilter(&YAxis,Sensor_Info.Raspberry_Yaxis/100,RT_Info.AccY);
            RT_Info.PointY = YAxis.Axis_Pos;
            RT_Info.PointY_V = YAxis.Axis_Vel;
        }
        else if(Fly_Mode == Data_Follow)
        {
        }

        POS_KalmanFilter(&FlowX,Sensor_Info.FlowX,-RT_Info.AccX);
        RT_Info.FlowX = FlowX.Axis_Pos;
        RT_Info.FlowX_V = FlowX.Axis_Vel;

        POS_KalmanFilter(&FlowY,-Sensor_Info.FlowY,-RT_Info.AccY);
        RT_Info.FlowY = FlowY.Axis_Pos;
        RT_Info.FlowY_V = FlowY.Axis_Vel;

//        OSTimeDly(5);
        OSTimeDlyHMSM(0,0,0,5);
    }
}




/*
*********************************************************************************************************
*                                            App_TaskProcessVisionData
*
* Description : ProcessVisionData task
*
* Argument(s) : p_arg       the argument passed by 'OSTaskCreateExt()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*********************************************************************************************************
*/
OS_EVENT ProcessVisionData_proc;//信号量
Uint16 ReciveVisionData[32];
static  void  App_TaskProcessVisionData (void *p_arg){
   /* Prevent compiler warning for not using 'p_arg'       */
   (void)&p_arg;
   INT8U err;
   ProcessVisionData_proc = *OSSemCreate(1);
   /* Task body, always written as an infinite loop.       */
    while (DEF_TRUE) {
        OSSemPend (&ProcessVisionData_proc,0,&err);

        Process_VisionData(ReciveVisionData);
    }
}





/*
*********************************************************************************************************
*                                            App_TaskProcessPCData
*
* Description : ProcessPCData task
*
* Argument(s) : p_arg       the argument passed by 'OSTaskCreateExt()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*********************************************************************************************************
*/
OS_EVENT ProcessPCData_proc;//信号量
Uint16 RecivePCData[32];
static  void  App_TaskProcessPCData (void *p_arg){
   /* Prevent compiler warning for not using 'p_arg'       */
   (void)&p_arg;
   INT8U err;
   ProcessPCData_proc = *OSSemCreate(1);
   /* Task body, always written as an infinite loop.       */
    while (DEF_TRUE) {
        OSSemPend (&ProcessPCData_proc,0,&err);
        Process_PCData(RecivePCData);
    }
}

/*
*********************************************************************************************************
*                                            App_TaskDataToPC
*
* Description : DataToPC task
*
* Argument(s) : p_arg       the argument passed by 'OSTaskCreateExt()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*********************************************************************************************************
*/
static  void  App_TaskDataToPC (void *p_arg){
   /* Prevent compiler warning for not using 'p_arg'       */
   (void)&p_arg;
   /* Task body, always written as an infinite loop.       */
    while (DEF_TRUE) {
        if(FlightControl.ReportSW==Report_SET){
            sendParaInfo();
            FlightControl.ReportSW=Report_RESET;
        }else{
            if(OffsetData.MagOffseting){
                /*上传校准数据*/
                sendRTOffset();
            }
            else{
                sendRTInfo(); //上传实时数据
//                sendGyroData();  //上传陀螺仪数据
//                sendAccData(); //上传加速度数据
//                sendMagData(); //上传磁力计数据
            }
        }
        if( FlightControl.TakeOff == Complete_TakingOff &&  RT_Info.Height >= 0.1f ){
              SendTakeOffFlag();
              FlightControl.TakeOff = Not_TakingOff;
        }

        if( FlightControl.ArmPower ==   ArmPowerON  &&  RT_Info.CpuTick>=100)
        {
              SendT265StartFlag();
              RT_Info.CpuTick=0;
        }
        else if(FlightControl.ArmPower ==   ArmPowerON  &&  RT_Info.CpuTick<100)
        {
              RT_Info.CpuTick ++;
        }
        OSTimeDlyHMSM(0,0,0,10);
    }
}

/*
*********************************************************************************************************
*                                            App_TaskBattery
*
* Description : Battery task
*
* Argument(s) : p_arg       the argument passed by 'OSTaskCreateExt()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*********************************************************************************************************
*/

static  void  App_TaskBattery (void *p_arg){
    /* Prevent compiler warning for not using 'p_arg'       */
    (void)&p_arg;
    /* Task body, always written as an infinite loop.       */
    float Battery_Array[5];
    while (DEF_TRUE) {
        /* 获取滤波后的电压 */
        RT_Info.batteryVoltage = Average_Filter(Get_Battery(),5,Battery_Array);
        /* 起飞电压必须高于11.30V 才可以起飞 */
        if(RT_Info.batteryVoltage<11.30f && (FlightControl.OnOff != Drone_On))
        {
            RT_Info.lowPowerFlag = 1;
            //此处可加小灯闪烁表示电池电量低
        }
        else
        {
            /*飞行中如果电压低于10.60V则自动降落*/
            if(RT_Info.batteryVoltage < 10.60f)
            {
                FlightControl.landFlag = 1;
            }
            RT_Info.lowPowerFlag = 0;
        }
//        OSTimeDly(200);
        OSTimeDlyHMSM(0,0,0,200);
    }
}

///*
//*********************************************************************************************************
//*                                            App_TaskRemoteControl
//*
//* Description : RemoteControl task
//*
//* Argument(s) : p_arg       the argument passed by 'OSTaskCreateExt()'.
//*
//* Return(s)   : none.
//*
//* Caller(s)   : This is a task.
//*********************************************************************************************************
//*/
//OS_EVENT RemoteControl_proc;//信号量
//Uint16 Receive_PPM_In[9];
//static  void  App_TaskRemoteControl (void *p_arg)
//{
//    (void)&p_arg;
//    INT8U err;
//    RemoteControl_proc = *OSSemCreate(1);
//    /* Task body, always written as an infinite loop.       */
//     while (DEF_TRUE) {
//         OSSemPend (&RemoteControl_proc,0,&err);
////         Process_RemoteData(Receive_PPM_In);
//     }
//}

/**
 */
OS_EVENT ProcessFlowData_proc;//信号量
Uint16 ReciveFlowData[28];
static void App_TaskFlow(void *p_arg)
{
    (void)&p_arg;
    INT8U err;
    ProcessFlowData_proc = *OSSemCreate(1);
    while(1)
    {
        OSSemPend (&ProcessFlowData_proc,0,&err);
        Process_FlowData(ReciveFlowData);
    }
}


/*
*********************************************************************************************************
*                                  App_TaskLaser
*
* Description : Laser task
*
* Argument(s) : p_arg       the argument passed by 'OSTaskCreateExt()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*********************************************************************************************************
*/
OS_EVENT ProcessLaserData_proc;//信号量
unsigned char ReciveLaserData[16];
static  void  App_TaskLaser (void *p_arg)
{
    /* Prevent compiler warning for not using 'p_arg'       */
    (void)&p_arg;
    INT8U err;
    ProcessLaserData_proc = *OSSemCreate(1);
    /* Task body, always written as an infinite loop.       */
     while (DEF_TRUE) {
         OSSemPend (&ProcessLaserData_proc,0,&err);
         Process_LaserData(ReciveLaserData);
     }
}
