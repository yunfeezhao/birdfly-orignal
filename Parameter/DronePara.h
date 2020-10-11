/*
 * DronePara.h
 *
 *  Created on: 2018骞�4鏈�17鏃�
 *      Author: Xiluna Tech
 */

#ifndef PARAMETER_DRONEPARA_H_
#define PARAMETER_DRONEPARA_H_

#include "F2837xD_device.h"

typedef enum
{
  Report_SET      = 0x01,
  Report_RESET    = 0x00,
}DroneReportSW_TypeDef;

//定义姿态数据来源
#define VIO_Attitude
//#define IMU_Attitude

//定义机型
#define Model330
#ifdef Model330
    #define ARM_Length 0.165f
    #define Drone_Mass 1.2f
    #define Drag_Coeff 0.016f
    //转动惯量设置  转动惯量 = M*R^2  对于质点而言。这里无法测得转动惯量大小，变为一个可调参数设置
    #define Inertia_Wx    0.002f
    #define Inertia_Wy    0.002f
    #define Inertia_Wz    0.004f
#else
    #define ARM_Length 0.125f
    #define Drone_Mass 0.86f
    #define Drag_Coeff 0.016f
    //转动惯量设置  转动惯量 = M*R^2  对于质点而言。这里无法测得转动惯量大小，变为一个可调参数设置
    #define Inertia_Wx    0.001f
    #define Inertia_Wy    0.001f
    #define Inertia_Wz    0.002f
#endif


typedef struct {
    float x;
    float y;
    float z;
} Vector3f_t;

typedef enum{
  Drone_Mode_None=0,
  Drone_Mode_RatePitch, //濮挎�佸唴鐜�
  Drone_Mode_RateRoll,
  Drone_Mode_Pitch,     //濮挎�佸鐜�
  Drone_Mode_Roll,
  Drone_Mode_4Axis,     //鍥涜酱椋炶
}DroneFlightMode_TypeDef;

typedef enum{
    Drone_Off  = 0x00,//鍏抽棴鐢垫満
    Drone_On   = 0x01,//璧烽
    Drone_Land = 0x02,//闄嶈惤
}DroneFlightOnOff_TypeDef;


typedef enum{
    Not_TakingOff   = 0x00,
    Complete_TakingOff  = 0x01,
}DroneTakeOff_TypeDef;

typedef enum{
    ArmPowerOFF  = 0x00,
    ArmPowerON  = 0x01,
}ArmPowerStatus_TypeDef;

typedef struct{
    DroneFlightOnOff_TypeDef OnOff;
    DroneFlightMode_TypeDef DroneMode;
    DroneReportSW_TypeDef ReportSW;
    DroneTakeOff_TypeDef TakeOff;
    ArmPowerStatus_TypeDef  ArmPower;
    int landFlag;
    _Bool LaunchFlag;
    _Bool ControlStart;
    unsigned char poshold;
    unsigned char setTargetPos;
}DroneFlightControl;

typedef enum{
    Data_Headmode = 0,
    Data_Headfree = 1,
    Data_Point = 2,
    Data_Flow = 3,
    Data_Follow = 4,
}FlyMode;

typedef struct{
    //婊ゆ尝鏃堕棿
    float Merge_t;
    //鍙傛暟
    float Q_Position;
    float Q_Velocity;
    float Q_Bias;
    float R_Position;
    //鐘舵��
    float Axis_Pos;
    float Axis_Vel;
    float Axis_Bias;
    float Axis_Err;
    float AxisPCt_0;
    float AxisPCt_1;
    float AxisPCt_2;
    float AxisE;
    char AxisC_0;
    float AxisK_0;
    float AxisK_1;
    float AxisK_2;
    float Axist_0;
    float Axist_1;
    float Axist_2;
    float AxisPdot[9];
    float AxisPP[3][3];
}KalmanFilter;

typedef struct{
    float Pitch;
    float Roll;
    float Yaw;
    float RateRoll;
    float RatePitch;
    float RateYaw;
    float Height;
    float VelHeight;
    float AccHeight;
    float BlackLineV;
    float BlackLineYaw;
    float DesiredAccelerationX;
    float DesiredAccelerationY;
    float DesiredAccelerationZ;
    float RemotePitch;
    float RemoteRoll;
    float RemoteYaw;
    float RemoteSpeedZ;
    float RoutePlanX;
    float RoutePlanY;
    float RoutePlanZ;
    float PositionX;
    float PositionY;
    float PositionZ;
}DroneTargetInfo;

typedef struct{
    float fixedErroPitch;
    float fixedErroRoll;
}DroneErrangle;


typedef struct{
    //鐘舵��
    float error;
    float lasterror;
    float differential;
    float differentialFliter;
    float pOut;
    float iOut;
    float dOut;
    float value;
}PIDOut;

typedef struct{
    //鍙傛暟
    float Kp;
    float Ki;
    float Kd;
}PID;

typedef struct
{
    PID Pitch;
    PID Roll;
    PID Yaw;

    PID PitchRate;
    PID RollRate;
    PID YawRate;

    PID PosX;
    PID PosY;
    PID PosZ;

    PID VelX;
    PID VelY;
    PID VelZ;

    PID AccZ;

    PID FlowX;
    PID FlowY;
    PID FlowVelX;
    PID FlowVelY;

}PIDPara;

typedef enum
{
    Auto       = 0x01 ,   //鑷ǔ妯″紡
    AltHold    = 0x02 ,     //瀹氶珮
    PosHold    = 0x03 ,   //GPS瀹氱偣
    RTL        = 0x04 ,   //杩旇埅妯″紡
    NoGPS      = 0x05 ,   //鏃燝PS瀹氫綅妯″紡
    VIOPosHold = 0x06,
    VIOCruise  = 0x07,
}FlightMode;

typedef enum
{
    Heading       = 0x01 ,   //鏈夊ご妯″紡
    Headless      = 0x02 ,     //鏃犲ご妯″紡
}HeadingMode;

typedef enum
{
    On       = 0x01 ,  //寮�
    Off      = 0x02 ,  //鍏�
}Switch;


typedef struct{
    float XaxisPos;
    float YaxisPos;
    float ZaxisPos;
    float Navigation;
}RemoteControl;


typedef struct{
    float P;
    float R;
    float Z;
    float Y;
}RemoteSensing;

/*閬ユ帶鍣ㄦ暟鎹笌鎺у埗妯″紡*/
typedef struct
{
    FlightMode FlightControlMode; //閬ユ帶椋炶妯″紡
    HeadingMode FlightHeadingMode ; //閬ユ帶椋炶鏈夊ご鏃犲ご妯″紡璁剧疆
    FlightMode CH5_Mode1;   //绗簲閫氶亾鐨勬ā寮�1
    FlightMode CH5_Mode2;   //绗簲閫氶亾鐨勬ā寮�2
    FlightMode CH5_Mode3;   //绗簲閫氶亾鐨勬ā寮�3
    FlightMode CH6_Mode1;   //绗叚閫氶亾鐨勬ā寮�1
    FlightMode CH6_Mode2;   //绗叚閫氶亾鐨勬ā寮�2
    FlightMode CH6_Mode3;   //绗叚閫氶亾鐨勬ā寮�3
    Switch CH7_Mode1;   //绗竷閫氶亾鐨勬ā寮�1
    Switch CH7_Mode2;   //绗竷閫氶亾鐨勬ā寮�2
    int SafeThrottle;   //閬ユ帶鍣ㄥけ鎺т繚鎶ゆ补闂�
    unsigned char TakeOff_Flag; //璧烽鏍囧織浣�
}Remote_Control;

/*閬ユ帶淇℃伅*/
typedef struct
{
    int CH1;// 閫氶亾1 鍓考
    int CH2;// 閫氶亾2 鍗囬檷鑸�
    int CH3;// 閫氶亾3 娌归棬
    int CH4;// 閫氶亾4 鏂瑰悜鑸�
    int CH5;// 閫氶亾5 椋炶妯″紡
    int CH6;// 閫氶亾6 鏈夊ご鏃犲ご璁剧疆
    int CH7;// 閫氶亾7 澶辨帶淇濇姢
    int CH8;// 閫氶亾8

    int CH1_Max ,CH1_Min ,CH1_Mid ; // 閫氶亾1 鏈�澶ф渶灏忓拰鍥炰腑鍊�
    int CH2_Max ,CH2_Min ,CH2_Mid ; // 閫氶亾2 鏈�澶ф渶灏忓拰鍥炰腑鍊�
    int CH3_Max ,CH3_Min ,CH3_Mid ; // 閫氶亾3 鏈�澶ф渶灏忓拰鍥炰腑鍊�
    int CH4_Max ,CH4_Min ,CH4_Mid ; // 閫氶亾4 鏈�澶ф渶灏忓拰鍥炰腑鍊�
    int CH5_Max ,CH5_Min ,CH5_Mid ; // 閫氶亾5 鏈�澶ф渶灏忓拰鍥炰腑鍊�
    int CH6_Max ,CH6_Min ,CH6_Mid ; // 閫氶亾6 鏈�澶ф渶灏忓拰鍥炰腑鍊�
    int CH7_Max ,CH7_Min ,CH7_Mid ; // 閫氶亾7 鏈�澶ф渶灏忓拰鍥炰腑鍊�
    int CH8_Max ,CH8_Min ,CH8_Mid ; // 閫氶亾8 鏈�澶ф渶灏忓拰鍥炰腑鍊�
}Controller;

typedef struct{
    float PitchThrust;
    float RollThrust;
    float YawThrust;
    float HeightThrust;
    float Gravity_Acceleration;
    float f1;
    float f2;
    float f3;
    float f4;
    float t1;
    float t2;
    float t3;
    float t4;
    float PaddleEffect;
}Thrust;

typedef struct
{
    unsigned int M1;
    unsigned int M2;
    unsigned int M3;
    unsigned int M4;
}Throttle;

/*鏍″噯鏁版嵁*/
typedef struct
{
    _Bool MagOffseting;
    Uint16 MagX;
    Uint16 MagY;
    Uint16 MagZ;
    float GyroX;
    float GyroY;
    float GyroZ;
    float AccX;
    float AccY;
    float AccZ;
    float AccXScale;
    float AccYScale;
    float AccZScale;
}OffsetInfo;

typedef enum
{
    Lift     = 0x01 ,   //鍗囬檷
    Hover    = 0x02 ,     //鎮仠
}Remote_Control_Status;

typedef enum
{
    Sonar       = 0x01 ,   //澹板憪
//  Barometer   = 0x02 ,     //姘斿帇璁�
}Height_Data_Switching;


/*鏃犱汉鏈哄疄鏃朵俊鎭�*/
typedef struct
{
    float Pitch;
    float Roll;
    float Yaw;
    float HomeYaw;
    float ratePitch;
    float HeadfreeYaw;    //鏃犲ご妯″紡涓媦aw瑙掑害
    float Headfreezeta;   //鏃犲ご妯″紡鏃嬭浆Zeta瑙�
    float rateRoll;
    float rateYaw;
    float accXaxis;
    float accYaxis;
    float accZaxis;
    float Height;
    float Height_V;
    float Barometer;      //姘斿帇璁℃暟鎹�
    float Barometer_V;    //姘斿帇璁￠�熷害
    float FlowX;
    float FlowY;
    float FlowX_V;
    float FlowY_V;
    float PointX;
    float PointY;
    float PointX_V;
    float PointY_V;
    float VioPosition_X;
    float VioPosition_Y;
    float VioPosition_Z;
    float VioVel_X;
    float VioVel_Y;
    float VioVel_Z;
    float VioPitch;
    float VioRoll;
    float VioYaw;
    float Quaternion0;
    float Quaternion1;
    float Quaternion2;
    float Quaternion3;
    float batteryVoltage;
    float AccX;
    float AccY;
    float AccZ;
    float GyroX;
    float GyroY;
    float GyroZ;
    float MagX;
    float MagY;
    float MagZ;
    int lowPowerFlag;
    bool VioAbnormal;
    int CpuTick;
    int VioHeartbeat;
    float LPFTest1;
    float LPFTest2;
    unsigned char Key1Status;
    unsigned char Key2Status;
    unsigned char Key3Status;
    unsigned char AllowLanding;  // 閬ユ帶鍣ㄦ帶鍒堕檷钀芥爣蹇�
    Remote_Control_Status  controlStatus;  //涓撲笟閬ユ帶鍣ㄦ帶鍒剁姸鎬�
    Height_Data_Switching  heightDataSwitching;  //楂樺害鏁版嵁鏉ユ簮鍒囨崲
}DroneRTInfo;

typedef struct{
    float US100_Zaxis;
    float Laser;
    float Raspberry_Xaxis;
    float Raspberry_Yaxis;
    float VIO_Xaxis;
    float VIO_Yaxis;
    float VIO_Zaxis;
    float FlowVelX;
    float FlowVelY;
    float FlowX;
    float FlowY;
    float FixFlowX;
    float FixFlowY;
    float TofHeight;
}SensorData;

//float 鑱斿悎浣�
typedef union{
        float fv;
        Uint16 sv[2];
}float_union;

//flash 淇濆瓨
typedef struct {
    Uint16 isGood;
    PIDPara pidPara;
    OffsetInfo Offset_Data;
    Controller ControlData;
}FlashData;


#endif /* PARAMETER_DRONEPARA_H_ */
