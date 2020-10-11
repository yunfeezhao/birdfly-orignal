/*
 * BSP.h
 *
 *  Created on: 2018Äê4ÔÂ12ÈÕ
 *      Author: Xiluna Tech
 */

#ifndef HARDWARE_C28X_BSP_H_
#define HARDWARE_C28X_BSP_H_

#include "F28x_Project.h"
#include <app_cfg.h>
#include <ucos_ii.h>
#include <cpu_core.h>
#include <lib_def.h>
#include "DronePara.h"
#include "task.h"
//init
void C28x_BSP_Init(void);
void C28x_BSP_Tick_Init(void);
void C28x_BSP_Led_Init(void);
void C28x_BSP_Spi_Init(void);
void C28x_BSP_Epwm_Init(void);
void C28x_BSP_Scia_Init(void);
void C28x_BSP_Scib_Init(void);
void C28x_BSP_Scic_Init(void);
void C28x_BSP_ADC_Init(void);
void C28x_BSP_FLASH_Init(void);
void C28x_BSP_eCAP_Init(void);
void C28x_BSP_Scid_Init(void);
void SetTofMode(void);


//Function function
void scia_msg(unsigned char msg);
void scib_msg(unsigned char * msg);
void scib_safe_msg(unsigned char * msg);
void scic_msg(unsigned char * msg);
void scic_Xmsg(unsigned char *msg);



// Function function
void spi_xmit(Uint16 a);
#endif /* HARDWARE_C28X_BSP_H_ */
