/*
 * MS5611.c
 *
 *  Created on: 2019骞�3鏈�28鏃�
 *      Author: SXJ
 */
#include "MS5611.h"

#define MS5611Press_OSR  MS561101BA_OSR_4096  //脝酶脩鹿录脝虏脡录炉戮芦露脠
#define MS5611Temp_OSR   MS561101BA_OSR_4096  //脦脗露脠虏脡录炉戮芦露脠

// 脝酶脩鹿录脝脳麓脤卢禄煤
#define SCTemperature    0x01     //驴陋脢录脦脗露脠脳陋禄禄
#define CTemperatureing  0x02   //脮媒脭脷脳陋禄禄脦脗露脠
#define SCPressure       0x03     //驴陋脢录脳陋禄禄脝酶脩鹿
#define SCPressureing    0x04     //脮媒脭脷脳陋禄禄脝酶脩鹿脰碌

#define MOVAVG_SIZE      20     //脝酶脩鹿录脝脗脣虏篓鲁陇露脠

#define PA_OFFSET_INIT_NUM  100

static uint8_t  Now_doing = SCTemperature;                      //碌卤脟掳脳陋禄禄脳麓脤卢
static uint16_t PROM_C[MS561101BA_PROM_REG_COUNT];      //卤锚露篓脰碌麓忙路脜
static int32_t  tempCache;

static float Alt_offset_Pa=0;   //麓忙路脜脳脜0脙脳脢卤 露脭脫娄碌脛脝酶脩鹿录脝脰碌

double paOffsetNum = 0;
uint16_t  paInitCnt=0;
uint8_t paOffsetInited=0;

//interface for outside
uint8_t Baro_ALT_Updated = 0;   //脝酶脩鹿录脝赂脽露脠赂眉脨脗脥锚鲁脡卤锚脰戮
//units (Celsius degrees*100, mbar*100  ).
//碌楼脦禄  [脦脗露脠 露脠] [脝酶脩鹿 脜脕]  [赂脽露脠 脙脳]
volatile float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude,MS5611_VerticalSpeed;

// 脩脫脢卤脢卤录盲 us    虏禄脥卢碌脛虏脡脩霉戮芦露脠露脭脫娄虏禄脥卢碌脛脩脫脢卤脰碌
uint32_t MS5611_DELAY_US[9] = {
    1500,//MS561101BA_OSR_256 0.9ms  0x00
    1500,//MS561101BA_OSR_256 0.9ms
    2000,//MS561101BA_OSR_512 1.2ms  0x02
    2000,//MS561101BA_OSR_512 1.2ms
    3000,//MS561101BA_OSR_1024 2.3ms 0x04
    3000,//MS561101BA_OSR_1024 2.3ms
    5000,//MS561101BA_OSR_2048 4.6ms 0x06
    5000,//MS561101BA_OSR_2048 4.6ms
    11000,//MS561101BA_OSR_4096 9.1ms 0x08
};

// FIFO 露脫脕脨
static float Temp_buffer[MOVAVG_SIZE],Alt_buffer[MOVAVG_SIZE];
static uint8_t temp_index=0; //露脫脕脨脰赂脮毛

uint32_t micros(void)
{
    uint32_t temp=0 ;
    temp = (uint32_t)(CpuTimer0Regs.TIM.bit.MSW <<8 ) + CpuTimer0Regs.TIM.bit.LSW;
    return temp;
}


//脤铆录脫脪禄赂枚脨脗碌脛脰碌碌陆脦脗露脠露脫脕脨陆酶脨脨脗脣虏篓
void MS561101BA_NewTemp(float val)
{
    Temp_buffer[temp_index] = val;
    temp_index = (temp_index + 1) % MOVAVG_SIZE;
}


//脤铆录脫脪禄赂枚脨脗碌脛脰碌碌陆赂脽露脠露脫脕脨陆酶脨脨脗脣虏篓
void MS561101BA_NewAlt(float val)
{
    int16_t i;
    for(i=1;i<MOVAVG_SIZE;i++)
        Alt_buffer[i-1] = Alt_buffer[i];
    Alt_buffer[MOVAVG_SIZE-1] = val;
}

//露脕脠隆露脫脕脨碌脛脝陆戮霉脰碌
float MS561101BA_getAvg(float * buff, int size)
{
    float sum = 0.0;
    int i;
    for(i=0; i<size; i++)
    {
        sum += buff[i];
    }
    return sum / size;
}

/**************************脢碌脧脰潞炉脢媒********************************************
*潞炉脢媒脭颅脨脥:      void MS561101BA_readPROM(void)
*鹿娄    脛脺:    露脕脠隆 MS561101B 碌脛鹿陇鲁搂卤锚露篓脰碌
露脕脠隆 脝酶脩鹿录脝碌脛卤锚露篓脰碌 脫脙脫脷脨脼脮媒脦脗露脠潞脥脝酶脩鹿碌脛露脕脢媒
*******************************************************************************/
void MS561101BA_readPROM(void)
{
    uint8_t  inth,intl;
    int i;
    for (i=0;i<MS561101BA_PROM_REG_COUNT;i++)
    {
        #ifdef DEBUG_HW_I2C
            i2cRead(MS5611_ADDR, MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE),2, i2cret);
            PROM_C[i]=i2cret[0]<<8 | i2cret[1];
        #else
            IIC_Start();
            IIC_Send_Byte(MS5611_ADDR);
            IIC_Wait_Ack();
            IIC_Send_Byte(MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
            IIC_Wait_Ack();
            IIC_Stop();
            DELAY_US(5);
            IIC_Start();
            IIC_Send_Byte(MS5611_ADDR+1);   //陆酶脠毛陆脫脢脮脛拢脢陆
            DELAY_US(1);
            IIC_Wait_Ack();
            inth = IIC_Read_Byte(1);            //麓酶ACK碌脛露脕脢媒戮脻
            DELAY_US(1);
            intl = IIC_Read_Byte(0);                //脳卯潞贸脪禄赂枚脳脰陆脷NACK
            IIC_Stop();

            PROM_C[i] = (((uint16_t)inth << 8) | intl);
        #endif
    }
}

/**************************脢碌脧脰潞炉脢媒********************************************
*潞炉脢媒脭颅脨脥:      void MS561101BA_reset(void)
*鹿娄    脛脺:    路垄脣脥赂麓脦禄脙眉脕卯碌陆MS561101B
*******************************************************************************/
void MS561101BA_reset(void)
{
    #ifdef DEBUG_HW_I2C
        i2cWrite(MS5611_ADDR, MS561101BA_RESET, 1);
    #else
    IIC_Start();
  IIC_Send_Byte(MS5611_ADDR);           //脨麓碌脴脰路
    IIC_Wait_Ack();
  IIC_Send_Byte(MS561101BA_RESET);  //路垄脣脥赂麓脦禄脙眉脕卯
    IIC_Wait_Ack();
  IIC_Stop();
    #endif
}

/**************************脢碌脧脰潞炉脢媒********************************************
*潞炉脢媒脭颅脨脥:      void MS561101BA_startConversion(uint8_t command)
*鹿娄    脛脺:    路垄脣脥脝么露炉脳陋禄禄脙眉脕卯碌陆 MS561101B
驴脡脩隆脳陋禄禄脙眉脕卯脦陋 MS561101BA_D1  脳陋禄炉脝酶脩鹿
                             MS561101BA_D2  脳陋禄炉脦脗露脠
*******************************************************************************/
void MS561101BA_startConversion(uint8_t command)
{
#ifdef DEBUG_HW_I2C
    i2cWrite(MS5611_ADDR, command, 1);
#else
    // initialize pressure conversion
    IIC_Start();
    IIC_Send_Byte(MS5611_ADDR);     //脨麓碌脴脰路
    IIC_Wait_Ack();
    IIC_Send_Byte(command);             //脨麓脳陋禄炉脙眉脕卯
    IIC_Wait_Ack();
    IIC_Stop();
#endif
}
#define CMD_ADC_READ            0x00 // ADC read command
/**************************脢碌脧脰潞炉脢媒********************************************
*潞炉脢媒脭颅脨脥:      unsigned long MS561101BA_getConversion(void)
*鹿娄    脛脺:    露脕脠隆 MS561101B 碌脛脳陋禄炉陆谩鹿没
*******************************************************************************/
uint32_t MS561101BA_getConversion(void)
{
    uint32_t conversion = 0;
    uint8_t temp[3];
    #ifdef DEBUG_HW_I2C
        i2cRead(MS5611_ADDR,CMD_ADC_READ ,3, temp);
        conversion=temp[0] << 16 | temp[0] <<8 | temp[2];
    #else
    IIC_Start();
    IIC_Send_Byte(MS5611_ADDR); //脨麓碌脴脰路
    IIC_Wait_Ack();
    IIC_Send_Byte(0);// start read sequence
    IIC_Wait_Ack();
    IIC_Stop();

    IIC_Start();
    IIC_Send_Byte(MS5611_ADDR+1);  //陆酶脠毛陆脫脢脮脛拢脢陆
    IIC_Wait_Ack();
    temp[0] = IIC_Read_Byte(1);  //麓酶ACK碌脛露脕脢媒戮脻  bit 23-16
    temp[1] = IIC_Read_Byte(1);  //麓酶ACK碌脛露脕脢媒戮脻  bit 8-15
    temp[2] = IIC_Read_Byte(0);  //麓酶NACK碌脛露脕脢媒戮脻 bit 0-7
    IIC_Stop();
    conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
    #endif
    return conversion;
}

/**************************脢碌脧脰潞炉脢媒********************************************
*潞炉脢媒脭颅脨脥:      void MS561101BA_init(void)
*鹿娄    脛脺:    鲁玫脢录禄炉 MS561101B
*******************************************************************************/
void MS5611_Init(void)
{
    MS561101BA_reset();         // 赂麓脦禄 MS561101B
    DELAY_US(100000);                     // 脩脫脢卤
    MS561101BA_readPROM();  // 露脕脠隆EEPROM脰脨碌脛卤锚露篓脰碌麓媒脫脙
}

/**************************脢碌脧脰潞炉脢媒********************************************
*潞炉脢媒脭颅脨脥:      void MS561101BA_GetTemperature(void)
*鹿娄    脛脺:    露脕脠隆 脦脗露脠脳陋禄炉陆谩鹿没
*******************************************************************************/
void MS561101BA_GetTemperature(void)
{
    tempCache = MS561101BA_getConversion();
}


/**************************脢碌脧脰潞炉脢媒********************************************
*潞炉脢媒脭颅脨脥:      float MS561101BA_get_altitude(void)
*鹿娄    脛脺:    陆芦碌卤脟掳碌脛脝酶脩鹿录脝脰碌脳陋鲁脡赂脽露脠隆拢
*******************************************************************************/
float MS561101BA_get_altitude(void)
{
    static float Altitude;

    // 脢脟路帽鲁玫脢录禄炉鹿媒0脙脳脝酶脩鹿脰碌拢驴
    if(Alt_offset_Pa == 0)
    {
        if(paInitCnt > PA_OFFSET_INIT_NUM)
        {
            Alt_offset_Pa = paOffsetNum / paInitCnt;
            paOffsetInited=1;
        }
        else
            paOffsetNum += MS5611_Pressure;

        paInitCnt++;

        Altitude = 0; //赂脽露脠脦陋 0

        return Altitude;
    }
    //录脝脣茫脧脿露脭脫脷脡脧碌莽脢卤碌脛脦禄脰脙碌脛赂脽露脠脰碌隆拢碌楼脦禄脦陋m
    Altitude = 4433000.0 * (1 - pow((MS5611_Pressure / Alt_offset_Pa), 0.1903))*0.01f;

    return Altitude;
}

/**************************脢碌脧脰潞炉脢媒********************************************
*潞炉脢媒脭颅脨脥:      void MS561101BA_getPressure(void)
*鹿娄    脛脺:    露脕脠隆 脝酶脩鹿脳陋禄禄陆谩鹿没 虏垄脳枚虏鹿鲁楼脨脼脮媒
*******************************************************************************/

void MS561101BA_getPressure(void)
{
    int64_t off,sens;
    int64_t TEMP,T2,Aux_64,OFF2,SENS2;  // 64 bits
    int32_t rawPress = MS561101BA_getConversion();
    int64_t dT  = tempCache - (((int32_t)PROM_C[4]) << 8);

    TEMP = 2000 + (dT * (int64_t)PROM_C[5])/8388608;
    off  = (((int64_t)PROM_C[1]) << 16) + ((((int64_t)PROM_C[3]) * dT) >> 7);
    sens = (((int64_t)PROM_C[0]) << 15) + (((int64_t)(PROM_C[2]) * dT) >> 8);

    if (TEMP < 2000)
    {   // second order temperature compensation
        T2 = (((int64_t)dT)*dT) >> 31;
        Aux_64 = (TEMP-2000)*(TEMP-2000);
        OFF2 = (5*Aux_64)>>1;
        SENS2 = (5*Aux_64)>>2;
        TEMP = TEMP - T2;
        off = off - OFF2;
        sens = sens - SENS2;
    }
    float newPress;
    //------------- 脝酶脩鹿脨脼脮媒 ----------------
    newPress=(((((int64_t)rawPress) * sens) >> 21) - off) / 32768;

//  float press_limit_coe = 1.0f;

//  if(ALT_LOCK_FLAG == 0xff && (Math_abs(IMU_Pitch)>15 || Math_abs(IMU_Roll)>15))
//  {
//      press_limit_coe = 0.01f;   //0.005
//      if(newPress<lastPress)
//          newPress = (1 - press_limit_coe) * lastPress + press_limit_coe * newPress;
//  }


    MS5611_Pressure = newPress;

    //脦脗露脠露脫脕脨麓娄脌铆
    MS561101BA_NewTemp(TEMP*0.01f);

    MS5611_Temperature = MS561101BA_getAvg(Temp_buffer,MOVAVG_SIZE); //0.01c

    MS5611_Altitude = MS561101BA_get_altitude();

//    Sensor_Info.MS5611_Zaxis = MS5611_Altitude;

//    RT_Info.Height = MS5611_Altitude;
}


/**************************脢碌脧脰潞炉脢媒********************************************
*潞炉脢媒脭颅脨脥:      void MS5611BA_Routing(void)
*鹿娄    脛脺:    MS5611BA 碌脛脭脣脨脨鲁脤脨貌拢卢脨猫脪陋露篓脝脷碌梅脫脙隆拢脌麓赂眉脨脗脝酶脩鹿脰碌潞脥脦脗露脠脰碌
*******************************************************************************/
void MS5611_ThreadNew(void)
{
    switch(Now_doing)
    { //虏茅脩炉脳麓脤卢
        case SCTemperature:  //脝么露炉脦脗露脠脳陋禄禄
                //驴陋脝么脦脗露脠脳陋禄禄
                MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
//                Current_delay = MS5611_DELAY_US[MS5611Temp_OSR] ;//脳陋禄禄脢卤录盲
//                Start_Convert_Time = micros();  //录脝脢卤驴陋脢录
                Now_doing = CTemperatureing;        //脧脗脪禄赂枚脳麓脤卢
        break;

        case CTemperatureing:  //脮媒脭脷脳陋禄禄脰脨
//            if((Start_Convert_Time-micros()) > Current_delay)
//            { //脩脫脢卤脢卤录盲脢脟路帽碌陆麓茂
                MS561101BA_GetTemperature(); //脠隆脦脗露脠
                //脝么露炉脝酶脩鹿脳陋禄禄
                MS561101BA_startConversion(MS561101BA_D1 + MS5611Press_OSR);
//                Current_delay = MS5611_DELAY_US[MS5611Press_OSR];//脳陋禄禄脢卤录盲
//                Start_Convert_Time = micros();//录脝脢卤驴陋脢录
                Now_doing = SCPressureing;//脧脗脪禄赂枚脳麓脤卢
//            }
            break;

        case SCPressureing:  //脮媒脭脷 脳陋禄禄脝酶脩鹿脰碌
//            if((Start_Convert_Time-micros()) > Current_delay)
//            { //脩脫脢卤脢卤录盲
                MS561101BA_getPressure();       //赂眉脨脗 录脝脣茫
                Baro_ALT_Updated = 0xff;            //赂脽露脠赂眉脨脗  脥锚鲁脡
                //驴陋脝么脦脗露脠 脳陋禄禄
                MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
//                Current_delay = MS5611_DELAY_US[MS5611Temp_OSR] ;//脳陋禄禄脢卤录盲
//                Start_Convert_Time = micros();                      //录脝脢卤驴陋脢录
                Now_doing = CTemperatureing;                            //脧脗脪禄赂枚脳麓脤卢
//            }
            break;
        default:
            Now_doing = CTemperatureing;
            break;
    }
}
//脢鹿脫脙脟掳卤锚露篓
uint8_t  WaitBaroInitOffset(void)
{
    uint32_t startTime=0;
    uint32_t now=0;

    startTime=micros(); //us
  while(!paOffsetInited)
    {
            MS5611_ThreadNew();
            now=micros();
            if((startTime-now)/1000 >= PA_OFFSET_INIT_NUM * 50) //
            {
                return 0;
            }
    }

    return 1;
}





