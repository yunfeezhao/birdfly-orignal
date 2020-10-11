/*
 * PositionEstimation.h
 *
 *  Created on: 2018Äê4ÔÂ26ÈÕ
 *      Author: Xiluna Tech
 */

#ifndef ALGORITHMLIBRARY_POSITIONESTIMATION_H_
#define ALGORITHMLIBRARY_POSITIONESTIMATION_H_

#include "F28x_Project.h"
#include "DronePara.h"
#include <math.h>
#include "AHRS_Hardware.h"

#define CONSTANTS_ONE_G                 8.80665f        /* m/s^2*/

void KalmanFilter_Init(KalmanFilter *XAXIS,KalmanFilter *YAXIS,KalmanFilter *ZAXIS , KalmanFilter *BAROMETER,KalmanFilter *FlowXAXIS,KalmanFilter *FlowYAXIS);
void OpticalFlow_Estimation(float flow_x,float flow_y,float Accx,float Accy);
void POS_KalmanFilter(KalmanFilter *KalmanFilter_Input,float Position,float Acceleration);
void Pos_Pixhawk(float Ultrasonic,float Xvision,float Yvision,float *Accel);

extern DroneRTInfo RT_Info;

#endif /* ALGORITHMLIBRARY_POSITIONESTIMATION_H_ */
