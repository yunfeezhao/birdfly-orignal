/*
 * ADC_Battery.c
 *
 *  Created on: 2018年5月2日
 *      Author: Xiluna Tech
 */

#include "ADC_Battery.h"

float Get_Battery(void){
    float BatteryPower;
    //
    //convert, wait for completion, and store results
    //start conversions immediately via software, ADCA
    //
    AdcaRegs.ADCSOCFRC1.all = 0x0003; //SOC0 and SOC1
    //
    //wait for ADCA to complete, then acknowledge flag
    //
    while(AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0);
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    BatteryPower = ((float)AdcaResultRegs.ADCRESULT1) /4096.0f * 3.3f * 4.9f;   // 3.3V为基准电压  4.9为电阻分压比
    return BatteryPower;
}

