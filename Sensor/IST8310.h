/*
 * IST8310.h
 *
 *  Created on: 2019Äê8ÔÂ23ÈÕ
 *      Author: xiluna
 */

#ifndef SENSOR_IST8310_H_
#define SENSOR_IST8310_H_
#include "task.h"

#define IST8310_SLAVE_ADDRESS     0x0E<<1
#define IST8310_REG_CNTRL1        0x0A  //Control setting register 1



typedef struct {
  unsigned char Buf[6];
  unsigned char Mag_Data[3];
  int16_t Mag_Data_Correct[3];
  int16_t thx;
  int16_t thy;
  int16_t x;
  int16_t y;
  int16_t z;
}IST8310;

extern IST8310 Mag_IST8310;

extern void IST8310_Init(void);
extern void Get_Mag_IST8310(void);

#endif /* SENSOR_IST8310_H_ */
