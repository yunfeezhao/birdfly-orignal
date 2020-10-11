/*
 * SimpleDigitalFiltering.h
 *
 *  Created on: 2018Äê5ÔÂ2ÈÕ
 *      Author: Xiluna Tech
 */

#ifndef ALGORITHMLIBRARY_SIMPLEDIGITALFILTERING_H_
#define ALGORITHMLIBRARY_SIMPLEDIGITALFILTERING_H_

#include "F28x_Project.h"
#include <math.h>

typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;


typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;

float Average_Filter(float FilterData,Uint16 Length,float *FilterArray);
float Median_Filter(int FilterData,Uint16 Length,int *FilterArray);
float Limits_data(float LimitsData,float LimitsUp,float LimitsDown);
float ABS(float data);
float MAX(float data1,float data2);
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF);
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);
#endif /* ALGORITHMLIBRARY_SIMPLEDIGITALFILTERING_H_ */
