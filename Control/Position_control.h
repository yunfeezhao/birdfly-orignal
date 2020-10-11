/*
 * Position_control.h
 *
 *  Created on: 2018Äê4ÔÂ25ÈÕ
 *      Author: Xiluna Tech
 */

#ifndef CONTROL_POSITION_CONTROL_H_
#define CONTROL_POSITION_CONTROL_H_

#include "task.h"

#define kAlmostZeroValueThreshold   0.001f

void Position_control(unsigned char Data_flag,float Climb,float Decline);
void ComputeDesiredAttitude(Vector3f_t DesiredAcceleration,float reference_heading);

#endif /* CONTROL_POSITION_CONTROL_H_ */
