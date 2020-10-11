/*
 * PID_Control.h
 *
 *  Created on: 2018Äê5ÔÂ18ÈÕ
 *      Author: Xiluna Tech
 */

#ifndef CONTROL_PID_CONTROL_H_
#define CONTROL_PID_CONTROL_H_

#include "F28x_Project.h"
#include "DronePara.h"
#include "SimpleDigitalFiltering.h"
#include <math.h>

#define lowpass_filter  7.9577e-3f

float PID_Control(PID *PIDpara, PIDOut *PIDstatus, float expect_PID, float feedback_PID, float PIDtime, float Integrallimiter);

#endif /* CONTROL_PID_CONTROL_H_ */
