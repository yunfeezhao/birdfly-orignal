/*
 * Attitude_control.h
 *
 *  Created on: 2018年4月23日
 *      Author: Xiluna Tech
 */

#ifndef CONTROL_ATTITUDE_CONTROL_H_
#define CONTROL_ATTITUDE_CONTROL_H_

#include "DronePara.h"
#include "F28x_Project.h"
#include "AHRS_Hardware.h"
#include "PID_Control.h"
#include "SimpleDigitalFiltering.h"
#include <math.h>
#include "task.h"


void PWM_OUTPUT(unsigned int Motor1,unsigned int Motor2,unsigned int Motor3,unsigned int Motor4);
void Calculate_Thrust(float arm_length);
void Attitude_control(float PitchCalibration,float RollCalibration);
void Safety_Protection(void);
void MotorThrust(float f1,float f2,float f3,float f4);
void Throttle_Angle_Compensate(void);//油门倾角补偿

typedef struct{
    Vector3f_t ExpectWRate;
    Vector3f_t LastEstimateGyro;
    Vector3f_t LastDerivative;
    Vector3f_t Thrust;
}AttitudeControl;

#endif /* CONTROL_ATTITUDE_CONTROL_H_ */
