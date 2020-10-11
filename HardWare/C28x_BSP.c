/*
 * BSP.c
 *
 *  Created on: 2018年4月12日
 *      Author: Xiluna Tech
 */

#include "C28x_BSP.h"

//
// Function Prototypes
//
__interrupt void cpu_timer2_isr(void){
    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    OSIntEnter();
    OSTimeTick();
    OSIntExit();
    EDIS;
}

void C28x_BSP_Init(void){
//    C28x_BSP_Led_Init();
    C28x_BSP_Spi_Init();
    C28x_BSP_Scia_Init();
    C28x_BSP_Scib_Init();
    C28x_BSP_Scic_Init();
    C28x_BSP_Scid_Init();
    C28x_BSP_ADC_Init();
    C28x_BSP_FLASH_Init();
//    C28x_BSP_eCAP_Init();
//    C28x_BSP_OLED_init();
//    C28x_BSP_Key_Init();
//    IIC_Init();                                           // iic总线初始化
//    MS5611_Init();                                      // MS5611气压计初始化
//    WaitBaroInitOffset();                               // 等待气压初始化高度完成
    DELAY_US(500*1000);
    C28x_BSP_Epwm_Init();
}
/*
 * 操作系统时基配置
 */
void C28x_BSP_Tick_Init(void){
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers
    InitCpuTimers();   // For this example, only initialize the Cpu Timers
//
// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
// 200MHz CPU Freq, 1 millisecond Period (in uSeconds)
//
    ConfigCpuTimer(&CpuTimer2, 200, 1000);

    CpuTimer2Regs.TCR.all = 0x4001;

    IER |= M_INT14;// Enable CPU int1 which is connected to CPU-Timer 0

//    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM
}
/*
 * C28x LED配置
 */
void C28x_BSP_Led_Init(){
    GPIO_SetupPinMux(71, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(71, GPIO_OUTPUT, GPIO_PUSHPULL);
}

/*
 * C28x spi端口配置
 *
 * launchpad gpio16,17,18,19
 */
//
// spi_fifo_init - Initialize SPIA FIFO
//
void spi_fifo_init()
{
    SpiaRegs.SPIFFTX.all = 0xE040;
    SpiaRegs.SPIFFRX.all = 0x2044;
    SpiaRegs.SPIFFCT.all = 0x0;

    InitSpi();
}

void C28x_BSP_Spi_Init(){

    InitSpiGpio();

    spi_fifo_init();     // Initialize the SPI FIFO
}


void spi_xmit(Uint16 a)
{
    SpiaRegs.SPITXBUF = a;
}


/*
 * C28x epwm端口配置
 *
 * launchpad gpio2,3,10,11
 */

//
// InitEPwm2Example - Initialize EPWM2 configuration
//

void InitEPwm2Example()
{
    EPwm2Regs.TBPRD = 12499;                       // Set timer period

    EPwm2Regs.CMPA.bit.CMPA = 6250;
    EPwm2Regs.CMPB.bit.CMPB = 6250;
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                     // Clear counter

    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Count up
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV4;          // Slow just to observe on
                                                   // the scope

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;    // Set PWM2A on Zero
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;

    EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;    // Set PWM2B on Zero
    EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;

}


//
// InitEPwm6Example - Initialize EPWM2 configuration
//
void InitEPwm6Example()
{
    EPwm6Regs.TBPRD = 12499;                       // Set timer period

    EPwm6Regs.CMPA.bit.CMPA = 6250;
    EPwm6Regs.CMPB.bit.CMPB = 6250;
    EPwm6Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm6Regs.TBCTR = 0x0000;                     // Clear counter

    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Count up
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV4;          // Slow just to observe on
                                                   // the scope
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;    // Set PWM2A on Zero
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;

    EPwm6Regs.AQCTLB.bit.ZRO = AQ_SET;    // Set PWM2B on Zero
    EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;
}

void C28x_BSP_Epwm_Init(){

    CpuSysRegs.PCLKCR2.bit.EPWM2=1;
    CpuSysRegs.PCLKCR2.bit.EPWM6=1;

    InitEPwm2Gpio();
    InitEPwm6Gpio();

    InitEPwm2Example();
    InitEPwm6Example();

}
/*
 * SCIA 初始化 115200 baund @ LSPCLK = 50Mhz
 *
 * void scia_fifo_init(void);
 *
 * void scia_msg(unsigned char * msg);
 *
 * interrupt void sciaRxFifoIsr(void);
 *
 * void C28x_BSP_Scia_Init(void);
 *
 */

//
// scia_fifo_init - Configure SCIA FIFO
//
void scia_fifo_init()
{
   SciaRegs.SCICCR.all = 0x0007;      // 1 stop bit,  No loopback
                                      // No parity,8 char bits,
                                      // async mode, idle-line protocol
   SciaRegs.SCICTL1.all = 0x0003;     // enable TX, RX, internal SCICLK,
                                      // Disable RX ERR, SLEEP, TXWAKE
   SciaRegs.SCICTL2.bit.TXINTENA = 1;
   SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
   SciaRegs.SCIHBAUD.all = 0x0001;    //19200 baund @ LSPCLK = 50Mhz
   SciaRegs.SCILBAUD.all = 0x0044;
//   SciaRegs.SCIHBAUD.all = 0x0002;    //9600 baund @ LSPCLK = 50Mhz
//   SciaRegs.SCILBAUD.all = 0x008A;
   SciaRegs.SCICCR.bit.LOOPBKENA = 0; // Enable loop back
   SciaRegs.SCIFFTX.all = 0xC040;
   SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
   SciaRegs.SCIFFRX.bit.RXFFIL = 14;
//   SciaRegs.SCIFFRX.all = 0x0030;     //接收16个数据
   SciaRegs.SCIFFCT.all = 0x00;

   SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
   SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
   SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
}


void scia_msg(unsigned char msg)//* msg
{
    int i;
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    for (i = 0;i < 16;i ++){
        SciaRegs.SCITXBUF.all = msg;
    }
}

//
// sciaRxFifoIsr - SCIA Receive FIFO ISR
//

interrupt void sciaRxFifoIsr(void)
{
    Uint16 i;
//    for(i=0;i<14;i++)
//    {
//       ReciveFlowData[i]=SciaRegs.SCIRXBUF.all;  // Read data
//    }
    for(i=14;i<28;i++)
    {
        ReciveFlowData[i-14]=ReciveFlowData[i];  // Read data
    }

    for(i=14;i<28;i++)
    {
        ReciveFlowData[i]=SciaRegs.SCIRXBUF.all;  // Read data
    }

   SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
   SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
    OSSemPost(&ProcessFlowData_proc);
}

void C28x_BSP_Scia_Init(){
    GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 15);
    GPIO_SetupPinOptions(43, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 15);
    GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_ASYNC);

    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.SCIA_RX_INT = &sciaRxFifoIsr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    scia_fifo_init();   // Init SCI-A

    IER |= M_INT9;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE Group 9, INT1
    EINT;
    ERTM;  // Enable Global realtime interrupt DBGM
}


/*
 * SCIB 初始化 115200 baund @ LSPCLK = 50Mhz
 *
 * void scib_fifo_init(void);
 *
 * void scib_msg(unsigned char * msg);
 *
 * interrupt void scibRxFifoIsr(void);
 *
 * void C28x_BSP_Scib_Init(void);
 *
 */

//
// scib_fifo_init - Configure SCIB FIFO
//
void scib_fifo_init()
{
   ScibRegs.SCICCR.all = 0x0007;      // 1 stop bit,  No loopback
                                      // No parity,8 char bits,
                                      // async mode, idle-line protocol
   ScibRegs.SCICTL1.all = 0x0003;     // enable TX, RX, internal SCICLK,
                                      // Disable RX ERR, SLEEP, TXWAKE
   ScibRegs.SCICTL2.bit.TXINTENA = 1;
   ScibRegs.SCICTL2.bit.RXBKINTENA = 1;
   ScibRegs.SCIHBAUD.all = 0x0000;    //115200 baund @ LSPCLK = 50Mhz
   ScibRegs.SCILBAUD.all = 0x0035;
   ScibRegs.SCICCR.bit.LOOPBKENA = 0; // Enable loop back
   ScibRegs.SCIFFTX.all = 0xC050;   //0xC040
   ScibRegs.SCIFFRX.all = 0x0030;
   ScibRegs.SCIFFCT.all = 0x01;

   ScibRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
   ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;
   ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
}


void scib_safe_msg(unsigned char * msg)
{
    int i;
    if (ScibRegs.SCIFFTX.bit.TXFFST == 0) {
        for (i = 0;i < 16;i ++){
                ScibRegs.SCITXBUF.all = msg[i];
            }
    }
}

void scib_msg(unsigned char * msg)
{
    int i;
    while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {}
    for (i = 0;i < 16;i ++){
        ScibRegs.SCITXBUF.all = msg[i];
    }
}



//
// scibRxFifoIsr - SCIB Receive FIFO ISR
//

interrupt void scibRxFifoIsr(void)
{
    Uint16 i;
    for(i=16;i<32;i++)
    {
        RecivePCData[i-16]=RecivePCData[i];  // Read data
    }

    for(i=16;i<32;i++)
    {
        RecivePCData[i]=ScibRegs.SCIRXBUF.all;  // Read data
    }

    ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
    OSSemPost(&ProcessPCData_proc);
}

void C28x_BSP_Scib_Init(){

    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 2);
    GPIO_SetupPinOptions(19, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(18, GPIO_MUX_CPU1, 2);
    GPIO_SetupPinOptions(18, GPIO_OUTPUT, GPIO_ASYNC);

    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.SCIB_RX_INT = &scibRxFifoIsr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    scib_fifo_init();   // Init SCI-B

    IER |= M_INT9;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER9.bit.INTx3=1;     // PIE Group 9, INT3
    EINT;
    ERTM;  // Enable Global realtime interrupt DBGM
}


/*
 * SCIC 初始化 115200 baund @ LSPCLK = 50Mhz
 *
 * void scic_fifo_init(void);
 *
 * void scic_msg(unsigned char * msg);
 *
 * interrupt void scicRxFifoIsr(void);
 *
 * void C28x_BSP_Scic_Init(void);
 *
 */

//
// scic_fifo_init - Configure SCIC FIFO
//
void scic_fifo_init()
{
   ScicRegs.SCICCR.all = 0x0007;      // 1 stop bit,  No loopback
                                      // No parity,8 char bits,
                                      // async mode, idle-line protocol
   ScicRegs.SCICTL1.all = 0x0003;     // enable TX, RX, internal SCICLK,
                                      // Disable RX ERR, SLEEP, TXWAKE
   ScicRegs.SCICTL2.bit.TXINTENA = 1;
   ScicRegs.SCICTL2.bit.RXBKINTENA = 1;
//   ScicRegs.SCIHBAUD.all = 0x0000;    //115200 baund @ LSPCLK = 50Mhz
//   ScicRegs.SCILBAUD.all = 0x0035;
   ScicRegs.SCIHBAUD.all = 0x0000;    //115200 baund @ LSPCLK = 50Mhz
   ScicRegs.SCILBAUD.all = 0x001A;    //230400 baund @ LSPCLK = 50Mhz
   ScicRegs.SCICCR.bit.LOOPBKENA = 0; // Enable loop back
   ScicRegs.SCIFFTX.all = 0xC040;
   ScicRegs.SCIFFRX.bit.RXFFIENA = 1;
   ScicRegs.SCIFFRX.bit.RXFFIL = 16;
   ScicRegs.SCIFFCT.all = 0x00;

   ScicRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
   ScicRegs.SCIFFTX.bit.TXFIFORESET = 1;
   ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;
//    ScicRegs.SCICCR.all = 0x0007;      // 1 stop bit,  No loopback
//                                       // No parity,8 char bits,
//                                       // async mode, idle-line protocol
//    ScicRegs.SCICTL1.all = 0x0003;     // enable TX, RX, internal SCICLK,
//                                       // Disable RX ERR, SLEEP, TXWAKE
//    ScicRegs.SCICTL2.bit.TXINTENA = 1;
//    ScicRegs.SCICTL2.bit.RXBKINTENA = 1;
//    ScicRegs.SCIHBAUD.all = 0x0002;    //9600 baund @ LSPCLK = 50Mhz
//    ScicRegs.SCILBAUD.all = 0x008A;
//    ScicRegs.SCICCR.bit.LOOPBKENA = 0; // Enable loop back
//    ScicRegs.SCIFFTX.all = 0xC040;
//    ScicRegs.SCIFFRX.all = 0x0022;
//    ScicRegs.SCIFFCT.all = 0x00;
//
//    ScicRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
//    ScicRegs.SCIFFTX.bit.TXFIFORESET = 1;
//    ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;
}


void scic_msg(unsigned char * msg)
{
    int i;
    while (ScicRegs.SCIFFTX.bit.TXFFST != 0) {}
    for (i = 0;i < 16;i ++){
        ScicRegs.SCITXBUF.all = msg[i];
    }
}


void scic_Xmsg(unsigned char *msg)
{
    int i;
    while (ScicRegs.SCIFFTX.bit.TXFFST != 0) {}
    for (i = 0;i < sizeof(msg);i ++){
        ScicRegs.SCITXBUF.all = msg[i];
    }

}


//
// scicRxFifoIsr - SCIC Receive FIFO ISR
//
interrupt void scicRxFifoIsr(void)
{
    static unsigned char i=0;
    for(i=16;i<32;i++)
    {
        ReciveVisionData[i-16]=ReciveVisionData[i];  // Read data
    }

    for(i=16;i<32;i++)
    {
        ReciveVisionData[i]=ScicRegs.SCIRXBUF.all;  // Read data
    }

//    for(i=0;i<16;i++)
//    {
//        ReciveVisionData[i]=ScicRegs.SCIRXBUF.all;  // Read data
//    }

    ScicRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    ScicRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEIFR8.bit.INTx5 = 0;

    PieCtrlRegs.PIEACK.bit.ACK8 = 1;       // Issue PIE ack
    OSSemPost(&ProcessVisionData_proc);
}

void C28x_BSP_Scic_Init(){
    #ifdef LaunchPad_PinConfig
    // launchpad usartC引脚
    GPIO_SetupPinMux(139, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(139, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(56, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(56, GPIO_OUTPUT, GPIO_ASYNC);
#endif
    #ifndef LaunchPad_PinConfig
    // C2000飞控 usartC引脚
    GPIO_SetupPinMux(13, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(13, GPIO_INPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_ASYNC);
#endif

    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.SCIC_RX_INT = &scicRxFifoIsr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    scic_fifo_init();   // Init SCI-C

    IER |= M_INT8;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER8.bit.INTx5 = 1;     // PIE Group 8, INT5
    EINT;
    ERTM;  // Enable Global realtime interrupt DBGM
}


void scid_msg(unsigned char msg)
{
    while (ScidRegs.SCIFFTX.bit.TXFFST != 0) {}
        ScidRegs.SCITXBUF.all = msg;
}
void scid_Xmsg(unsigned char *msg)
{
    int i;
    while (ScidRegs.SCIFFTX.bit.TXFFST != 0) {}
    for (i = 0;i < sizeof(msg);i ++){
        ScidRegs.SCITXBUF.all = msg[i];
    }
}
void scid_fifo_init()
{
    ScidRegs.SCICCR.all = 0x0007;      // 1 stop bit,  No loopback
                                       // No parity,8 char bits,
                                       // async mode, idle-line protocol
    ScidRegs.SCICTL1.all = 0x0003;     // enable TX, RX, internal SCICLK,
                                       // Disable RX ERR, SLEEP, TXWAKE
    ScidRegs.SCICTL2.bit.TXINTENA = 1;
    ScidRegs.SCICTL2.bit.RXBKINTENA = 1;
    ScidRegs.SCIHBAUD.all = 0x0000;    //115200 baund @ LSPCLK = 50Mhz
    ScidRegs.SCILBAUD.all = 0x0035;
    ScidRegs.SCICCR.bit.LOOPBKENA = 0; // Enable loop back
    ScidRegs.SCIFFTX.all = 0xC040;
    ScidRegs.SCIFFRX.bit.RXFFIENA = 1;
    ScidRegs.SCIFFRX.bit.RXFFIL = 9;
    ScidRegs.SCIFFCT.all = 0x00;

    ScidRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
    ScidRegs.SCIFFTX.bit.TXFIFORESET = 1;
    ScidRegs.SCIFFRX.bit.RXFIFORESET = 1;
}

interrupt void scidRxFifoIsr(void)
{
    unsigned char i=0;
    for(i=9;i<18;i++)
    {
        ReciveLaserData[i-9]=ReciveLaserData[i];  // Read data
    }

    for(i=9;i<18;i++)
    {
        ReciveLaserData[i]=ScidRegs.SCIRXBUF.all;  // Read data
    }

    ScidRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    ScidRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
    PieCtrlRegs.PIEIFR8.bit.INTx7 = 0;
    PieCtrlRegs.PIEACK.bit.ACK8 = 1;         // Issue PIE ack
    OSSemPost(&ProcessLaserData_proc);
}


void C28x_BSP_Scid_Init(void){

    GPIO_SetupPinMux(46, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(46, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(47, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(47, GPIO_OUTPUT, GPIO_ASYNC);

    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.SCID_RX_INT = &scidRxFifoIsr;//接收中断
    EDIS;    // This is needed to disable write to EALLOW protected registers

    scid_fifo_init();   // Init SCI-A

    IER |= M_INT8;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER8.bit.INTx7 = 1;     // PIE Group 8, INT7
    EINT;
    ERTM;  // Enable Global realtime interrupt DBGM
}



//Uint16 CH_State = 0;
//__interrupt void Enhanced_Capture1_ISR(void)
//{
//    Uint32 pulseTime= ECap1Regs.CAP2/200;
//
//   if(pulseTime<=4000)
//   {
//       Receive_PPM_In[CH_State++]=pulseTime;
//   }
//   else
//       CH_State=0;
//
//  if(CH_State>7)
//  {
//      CH_State=0;
//  }
//
//    ECap1Regs.ECCLR.bit.CEVT2 = 1;              //清标志位
//    ECap1Regs.ECCLR.bit.INT = 1;
//    ECap1Regs.ECCTL2.bit.REARM = 1;             //重新计数
//    PieCtrlRegs.PIEACK.bit.ACK4 = 1;     // Acknowledge this __interrupt to receive more __interrupts from group 4
//    OSSemPost(&RemoteControl_proc);
//}

//void C28x_BSP_eCAP_Init(void)
//{
//    InitECap();
//    GPIO_SetupPinOptions(30, GPIO_INPUT, GPIO_PULLUP);
//    InitECap1Gpio(30);
//
//    EALLOW;  // This is needed to write to EALLOW protected registers
//    PieVectTable.ECAP1_INT = &Enhanced_Capture1_ISR;
//    EDIS;
//
//    IER |= M_INT4;
// // Enable eCAP INTn in the PIE: Group 3 __interrupt 1-6
//    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;
//    EINT;
//    ERTM;  // Enable Global realtime interrupt DBGM
//}


/*
 * C28x ADC端口配置
 *
 * 电池电量检测
 */

//
// ConfigureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC B
//
void ConfigureADC(void)
{
    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADCs
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

//
// SetupADCSoftware - Setup ADC channels and acquisition window
//
void SetupADCSoftware(void)
{
    Uint16 acqps;

    //
    // Determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    //
    //Select the channels to convert and end of conversion flag
    //ADCA
    //
    EALLOW;

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;     //SOC1 will convert pin A3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycles
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    EDIS;
}



void C28x_BSP_ADC_Init(void){
    ConfigureADC();
    SetupADCSoftware();
}


void C28x_BSP_FLASH_Init(void){

    InitFlash();

    SeizeFlashPump();
}
