/*
 * AHRS_Hardware.h
 *
 *  Created on: 2018
 *      Author: Xiluna Tech
 */

#ifndef SENSOR_AHRS_HARDWARE_H_
#define SENSOR_AHRS_HARDWARE_H_

#include "task.h"

#define PI 3.14159265358979f

void AHRS_HardWareinit(void);
void IMU_getValues(float * values);
void IMU_getInfo(void);

extern volatile float RDrone_R[3][3];

#endif /* SENSOR_AHRS_HARDWARE_H_ */
