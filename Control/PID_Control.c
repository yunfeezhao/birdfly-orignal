/*
 * PID_Control.c
 *
 *  Created on: 2018年5月18日
 *      Author: Xiluna Tech
 */

#include "PID_Control.h"

// 经典PID控制
float PID_Control(PID *PIDpara, PIDOut *PIDstatus, float expect_PID, float feedback_PID, float PIDtime, float Integrallimiter){

    PIDstatus->error = expect_PID - feedback_PID;
    PIDstatus->differential = (PIDstatus->error - PIDstatus->lasterror)/PIDtime;

    PIDstatus->differential = PIDstatus->differentialFliter + (PIDtime / (lowpass_filter + PIDtime)) * (PIDstatus->differential - PIDstatus->differentialFliter);

    PIDstatus->differentialFliter = PIDstatus->differential;

    PIDstatus->lasterror = PIDstatus->error;

    PIDstatus->pOut = PIDpara->Kp * PIDstatus->error;
    PIDstatus->iOut += PIDpara->Ki * PIDstatus->error;
    PIDstatus->dOut = PIDpara->Kd * PIDstatus->differential;
    PIDstatus->iOut = Limits_data(PIDstatus->iOut,Integrallimiter,-Integrallimiter);

    PIDstatus->value = PIDstatus->pOut + PIDstatus->iOut + PIDstatus->dOut;

    return PIDstatus->value;
}


// 积分分离PID   超过设定条件的阈值时，I控制无效，采用PD控制；反之，使用PID控制
float IntegralSeparation_PID_Control(PID *PIDpara, PIDOut *PIDstatus, float expect_PID, float feedback_PID , float PIDtime
                                     , float Integrallimiter,float SeparationThreshold, float SeparationConditions,float LowpassFilter){

    PIDstatus->error = expect_PID - feedback_PID;
    PIDstatus->differential = (PIDstatus->error - PIDstatus->lasterror)/PIDtime;

    PIDstatus->differential = PIDstatus->differentialFliter + (PIDtime / (LowpassFilter + PIDtime)) * (PIDstatus->differential - PIDstatus->differentialFliter);

    PIDstatus->differentialFliter = PIDstatus->differential;

    PIDstatus->lasterror = PIDstatus->error;

    PIDstatus->pOut = PIDpara->Kp * PIDstatus->error;
    if(SeparationConditions< SeparationThreshold)
        PIDstatus->iOut += PIDpara->Ki * PIDstatus->error;
    else
        PIDstatus->iOut = 0 ;
    PIDstatus->dOut = PIDpara->Kd * PIDstatus->differential;

    PIDstatus->iOut = Limits_data(PIDstatus->iOut,Integrallimiter,-Integrallimiter);

    PIDstatus->value = PIDstatus->pOut + PIDstatus->iOut + PIDstatus->dOut;

    return PIDstatus->value;
}

// 倾角分离PID   超过设定倾角的阈值时，只注重对轴平衡控制，对其余力矩上的控制进行衰减
float DipSeparation_PID_Control(PID *PIDpara, PIDOut *PIDstatus, float expect_PID, float feedback_PID , float PIDtime
                                     , float Integrallimiter,float SeparationConditions,float AttenuationCoefficient,float LowpassFilter){

    PIDstatus->error = AttenuationCoefficient *(expect_PID - feedback_PID) ;
    PIDstatus->differential = (PIDstatus->error - PIDstatus->lasterror)/PIDtime;

    PIDstatus->differential = PIDstatus->differentialFliter + (PIDtime / (LowpassFilter + PIDtime)) * (PIDstatus->differential - PIDstatus->differentialFliter);

    PIDstatus->differentialFliter = PIDstatus->differential;

    PIDstatus->lasterror = PIDstatus->error;

    PIDstatus->pOut = PIDpara->Kp * PIDstatus->error;

    PIDstatus->iOut += PIDpara->Ki * PIDstatus->error;

    PIDstatus->dOut = PIDpara->Kd * PIDstatus->differential;

    PIDstatus->iOut = Limits_data(PIDstatus->iOut,Integrallimiter,-Integrallimiter);

    PIDstatus->value = PIDstatus->pOut + PIDstatus->iOut + PIDstatus->dOut;

    return PIDstatus->value;
}

