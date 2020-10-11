/*
 * FlashAPI.h
 *
 *  Created on: 2018Äê11ÔÂ29ÈÕ
 *      Author: xiluna
 */

#ifndef FLASH_API_FLASHAPI_H_
#define FLASH_API_FLASHAPI_H_

#include "task.h"
#include "F021_F2837xD_C28x.h"
#include "F28x_Project.h"

#define Bzero_SectorN_start         0xBE000
#define Bzero_SectorN_End           0xBFFFF

#define Bzero_16KSector_u32length   0x1000
#define Bzero_64KSector_u32length   0x4000

extern Uint16 *ptr;

void Load_ParaConfig(void);
void Write_Config(void);

#endif /* FLASH_API_FLASHAPI_H_ */
