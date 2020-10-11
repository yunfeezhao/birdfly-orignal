//###########################################################################
//
// FILE:   F2837xD_Spi.c
//
// TITLE:  F2837xD SPI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2837xD Support Library v3.04.00.00 $
// $Release Date: Sun Mar 25 13:26:04 CDT 2018 $
// $Copyright:
// Copyright (C) 2013-2018 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "task.h"
//
// Calculate BRR: 7-bit baud rate register value
// SPI CLK freq = 500 kHz
// LSPCLK freq  = CPU freq / 4  (by default)
// BRR          = (LSPCLK freq / SPI CLK freq) - 1
//
#if CPU_FRQ_200MHZ
#define SPI_BRR        ((200E6 / 4) / 500E3) - 1
#endif

#if CPU_FRQ_150MHZ
#define SPI_BRR        ((150E6 / 4) / 500E3) - 1
#endif

#if CPU_FRQ_120MHZ
#define SPI_BRR        ((120E6 / 4) / 500E3) - 1
#endif

//
// InitSPI - This function initializes the SPI to a known state
//
void InitSpi(void)
{
    // Initialize SPI-A

    // Set reset low before configuration changes
    // Clock polarity (0 == rising, 1 == falling)
    // 8-bit character
    // Disable loop-back
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;
    SpiaRegs.SPICCR.bit.SPICHAR = (8-1);
    SpiaRegs.SPICCR.bit.SPILBK = 0 ;
    SpiaRegs.SPICCR.bit.HS_MODE = 0 ;

    // Enable master (0 == slave, 1 == master)
    // Enable transmission (Talk)
    // Clock phase (0 == normal, 1 == delayed)
    // SPI interrupts are disabled
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpiaRegs.SPICTL.bit.TALK = 1;
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
    SpiaRegs.SPICTL.bit.SPIINTENA = 0;

    // Set the baud rate
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = SPI_BRR;

    // Set FREE bit
    // Halting on a breakpoint will not halt the SPI
    SpiaRegs.SPIPRI.bit.FREE = 1;

    // Release the SPI from reset
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;


    /*-----------------------------------------------------------------------*/

    // Initialize SPI-B

    // Set reset low before configuration changes
    // Clock polarity (0 == rising, 1 == falling)
    // 8-bit character
    // Disable loop-back
    SpibRegs.SPICCR.bit.SPISWRESET = 0;
    SpibRegs.SPICCR.bit.CLKPOLARITY = 1;
    SpibRegs.SPICCR.bit.SPICHAR = (8-1);
    SpibRegs.SPICCR.bit.SPILBK = 0 ;
    SpibRegs.SPICCR.bit.HS_MODE = 0 ;

    // Enable master (0 == slave, 1 == master)
    // Enable transmission (Talk)
    // Clock phase (0 == normal, 1 == delayed)
    // SPI interrupts are disabled
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpibRegs.SPICTL.bit.TALK = 1;
    SpibRegs.SPICTL.bit.CLK_PHASE = 0;
    SpibRegs.SPICTL.bit.SPIINTENA = 0;

    // Set the baud rate
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = SPI_BRR;

    // Set FREE bit
    // Halting on a breakpoint will not halt the SPI
    SpibRegs.SPIPRI.bit.FREE = 1;

    // Release the SPI from reset
    SpibRegs.SPICCR.bit.SPISWRESET = 1;
}

//
// InitSpiGpio - This function initializes GPIO pins to function as SPI pins.
//               Each GPIO pin can be configured as a GPIO pin or up to 3
//               different peripheral functional pins. By default all pins come
//               up as GPIO inputs after reset.
//
//               Caution:
//               For each SPI peripheral
//               Only one GPIO pin should be enabled for SPISOMO operation.
//               Only one GPIO pin should be enabled for SPISOMI operation.
//               Only one GPIO pin should be enabled for SPICLK  operation.
//               Only one GPIO pin should be enabled for SPISTE  operation.
//               Comment out other unwanted lines.
//
void InitSpiGpio()
{
    /* IMU SPI接口 */
   InitSpiaGpio();
//   /* OLED接口 */
//   InitSpibGpio();
}

//
// InitSpiaGpio - Initialize SPIA GPIOs
//
void InitSpiaGpio(void)
{
    EALLOW;

#ifdef ICM20689
    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;  // Enable pull-up on GPIO58 (SPISIMOA)
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;  // Enable pull-up on GPIO59 (SPISOMIA)
    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;  // Enable pull-up on GPIO60 (SPICLKA)

    GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3; // Asynch input GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Asynch input GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3; // Asynch input GPIO18 (SPICLKA)

    GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 3;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO17 = 0;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 3;

    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3; // Configure GPIO16 as SPISIMOA
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // Configure GPIO17 as SPISOMIA
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3; // Configure GPIO18 as SPICLKA

    //mpu6500
    GPIO_SetupPinMux(57, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(57, GPIO_OUTPUT, GPIO_PUSHPULL);
    //lsm303
    GPIO_SetupPinMux(124, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(124, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_WritePin(57, 1);
    GPIO_WritePin(124, 1);
#else
    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;  // Enable pull-up on GPIO58 (SPISIMOA)
    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;  // Enable pull-up on GPIO59 (SPISOMIA)
    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;  // Enable pull-up on GPIO60 (SPICLKA)

    GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3; // Asynch input GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3; // Asynch input GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3; // Asynch input GPIO18 (SPICLKA)

    GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 3;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO59 = 3;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 3;

    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3; // Configure GPIO16 as SPISIMOA
    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 3; // Configure GPIO17 as SPISOMIA
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3; // Configure GPIO18 as SPICLKA
    //mpu6500
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    //lsm303

    GPIO_SetupPinMux(41, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(41, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_WritePin(41, 1);
    GPIO_WritePin(61, 1);
#endif
    EDIS;
}


void InitSpibGpio(void)
{
   EALLOW;

    //
    // Enable internal pull-up for the selected pins
    //
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //

   GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;  // Enable pull-up on GPIO63 (SPISIMOA)
   GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;  // Enable pull-up on GPIO65 (SPICLKA)


    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Asynch input GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Asynch input GPIO18 (SPICLKA)

    //
    //Configure SPI-A pins using GPIO regs
    //
    // This specifies which of the possible GPIO pins will be SPI functional
    // pins.
    // Comment out other unwanted lines.
    //

    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 3;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 3;

    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 3; // Configure GPIO16 as SPISIMOA

    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3; // Configure GPIO18 as SPICLKA

    //RES(restart)
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(64, GPIO_OUTPUT, GPIO_PUSHPULL);
    //DC
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_WritePin(64, 0);
    GPIO_WritePin(66, 1);
    EDIS;
}



//
// End of file
//
