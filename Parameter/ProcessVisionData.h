/*
 * ProcessVisionData.h
 *
 *  Created on: 2018Äê5ÔÂ2ÈÕ
 *      Author: Xiluna Tech
 */

#ifndef PARAMETER_PROCESSVISIONDATA_H_
#define PARAMETER_PROCESSVISIONDATA_H_

#include "task.h"

void Process_VisionData(Uint16 *VisionData);
void SendTakeOffFlag(void);
void SendT265StartFlag(void);
void RestartT265(void);

#endif /* PARAMETER_PROCESSVISIONDATA_H_ */
