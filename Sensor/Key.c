/*
 * Key.c
 *
 *  Created on: 2019Äê7ÔÂ23ÈÕ
 *      Author: Lenovo
 */

#include "Key.h"


void C28x_BSP_Key_Init(void)
{
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_INPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_INPUT, GPIO_PULLUP);


    PieVectTable.XINT1_INT = &XINT1_ISR;
    PieVectTable.XINT2_INT = &XINT2_ISR;
    PieVectTable.XINT3_INT = &XINT3_ISR;

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;          // Enable PIE Group 1 INT5
    PieCtrlRegs.PIEIER12.bit.INTx1 = 1;
    IER |= M_INT1;                              // Enable CPU INT1
    IER |= M_INT12;

    GPIO_SetupXINT1Gpio(28);
    GPIO_SetupXINT2Gpio(29);
    GPIO_SetupXINT3Gpio(31);


    //
    // Configure XINT1
    //
    XintRegs.XINT1CR.bit.POLARITY = 2;          //0 Falling edge interrupt
    XintRegs.XINT2CR.bit.POLARITY = 2;          //1 Rising edge interrupt
    XintRegs.XINT3CR.bit.POLARITY = 2;
    //
    // Enable XINT1 and XINT2
    //
    XintRegs.XINT1CR.bit.ENABLE = 1;            // Enable XINT1
    XintRegs.XINT2CR.bit.ENABLE = 1;            // Enable XINT2
    XintRegs.XINT3CR.bit.ENABLE = 1;
    //


}


