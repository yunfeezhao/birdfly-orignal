/*
 * SimpleDigitalFiltering.c
 *
 *  Created on: 2018年5月2日
 *      Author: Xiluna Tech
 */
#include "SimpleDigitalFiltering.h"
/*
 *  平均滤波 (浮点类型)
 */
float Average_Filter(float FilterData,Uint16 Length,float *FilterArray){
    float AverageValue = 0;
    int i;
    for(i=0;i<Length-1;i++)
    {
        FilterArray[i]=FilterArray[i+1];
    }
    FilterArray[Length - 1] = FilterData;
    for(i=0;i<Length;i++)
    {
        AverageValue += FilterArray[i];
    }
    AverageValue /= Length;
    return AverageValue;
}
/*
 *  中位值滤波 (整形)
 *  length 长度不宜过大，否者延时会过大，本例程使用全部使用7个值进行滤波处理
 */
float Median_Filter(int FilterData,Uint16 Length,int *FilterArray){
    Uint16 i = 0,j = 0;
    int temp = 0;
    int sum = 0;
    float AverageValue = 0;
    FilterArray[Length - 1] = FilterData;
    for(i=0;i<Length-1;i++)
    {
        FilterArray[i]=FilterArray[i+1];
    }
    //排序
    for(i = 0 ; i < Length-1 ; i++)
    {
        for(j = 0 ; j < Length-1-i; j++)
        {
            if(FilterArray[j] > FilterArray[j+1] )
            {
                temp = FilterArray[j];
                FilterArray[j] =  FilterArray[j+1];
                FilterArray[j+1] = temp;
            }
        }
    }
    if(Length%2 == 0){
        for(i = Length/2-2;i < Length/2+2;i++){
            sum += FilterArray[i];
        }
        AverageValue = sum/4;
    }
    else{
        for(i = Length/2-1;i < Length/2+2;i++){
            sum += FilterArray[i];
        }
        AverageValue = sum/3;
    }
    return AverageValue;
}



/*限幅函数*/
float Limits_data(float LimitsData,float LimitsUp,float LimitsDown)
{
    if(LimitsData > LimitsUp)
    {
        LimitsData = LimitsUp;
    }
    if(LimitsData < LimitsDown)
    {
        LimitsData = LimitsDown;
    }
    return LimitsData;
}
/*赋零函数*/
float Limits_zero(float LimitsData,float LimitsUp,float LimitsDown)
{
    if(LimitsData   < LimitsUp && LimitsData > LimitsDown)
    {
        LimitsData = 0;
    }
    else
    {
        LimitsData = LimitsData;
    }
    return LimitsData;
}


/*绝对值函数*/
float ABS(float data)
{
    if(data<0)
        data = -data;
    else
        data = data;
    return data;
}

/*比大小函数*/
float MAX(float data1,float data2)
{
    float Max =0 ;
    if(data1>=data2)
        Max =data1;
    else
        Max =data2;
    return Max;
}


void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF)
{
  float fr = sample_frequent / cutoff_frequent;
  float ohm = tanf(3.141592653589793f / fr);
  float c = 1.0f + 2.0f * cosf(3.141592653589793f / 4.0f) * ohm + ohm * ohm;
  if (cutoff_frequent <= 0.0f) {
    // no filtering
    return;
  }
  LPF->b[0] = ohm * ohm / c;
  LPF->b[1] = 2.0f * LPF->b[0];
  LPF->b[2] = LPF->b[0];
  LPF->a[0]=1.0f;
  LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
  LPF->a[2] = (1.0f - 2.0f * cosf(3.141592653589793f / 4.0f) * ohm + ohm * ohm) / c;
}


/*************************************************
函数名:    float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
说明: 加速度计低通滤波器
入口: float curr_input 当前输入加速度计,滤波器参数，滤波器缓存
出口: 无
备注: 2阶Butterworth低通滤波器
*************************************************/
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
  /* 加速度计Butterworth滤波 */
  /* 获取最新x(n) */
  Buffer->Input_Butter[2]=curr_input;
  /* Butterworth滤波 */
  Buffer->Output_Butter[2]=
    Parameter->b[0] * Buffer->Input_Butter[2]
      +Parameter->b[1] * Buffer->Input_Butter[1]
        +Parameter->b[2] * Buffer->Input_Butter[0]
          -Parameter->a[1] * Buffer->Output_Butter[1]
            -Parameter->a[2] * Buffer->Output_Butter[0];
  /* x(n) 序列保存 */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) 序列保存 */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
  return Buffer->Output_Butter[2];
}

