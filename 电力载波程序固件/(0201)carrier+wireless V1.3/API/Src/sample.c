
#include "sample.h"
#include "string.h"

#define MAX_BUF 224

#define SOFTWARE_RELATED
#define DIFF(a,b) ((a)>(b)?(a-b):(b-a))
/*for fsk*/

#define PWM_ENABLE()            EnablePWMOut()//定时器打开。产生频率信号 278K
#define PWM_DISABLE()           DisablePWMOut()

#define SAMP_TIME_ENABLE()      EnableSampeTime()
#define SAMP_TIME_ENABLE_EX()   EnableSampeTime_Ex()
#define SAMP_TIME_DISABLE()     DisableSampeTime()

#define PHASE_TIME_ENABLE()     EnablePhaseTime()
#define PHASE_TIME_DISABLE()    DisablePhaseTime()


/*定时器采样有关数据*/
/*

 每个相位过零点一共用2867.2us时间做为数据有效区。一共3333us
 每个相位2867/64(个采样点) = 44.8S（采样间隔）
 如果采用48MHz的主频，那么计数器的值为：44.8/(1/48) = 2150.4。也就是说：每2150时钟周期中断（采样）一次。
 发送和接收一样，采用固定定时器进行操作。每44.8uS中断一次，理论上每相位最大通讯速度为：64*100 = 6.4Kbit/S。
 每个相位的每个过零点采样64次，采用一个64位的变量保存每个采样点的真值。

*/
/*采样相关计数比较变量*/
#define S_TIME (4)
#define SAMPLE_TIMES_PER_PHASE              (4*S_TIME)
#define  Locale_max_sample_times_400        (1*S_TIME)
#define  Locale_max_sample_times_400_div2   ((S_TIME/2))

/*每个相位的工作模块*/
FSK_mode location_phaseA_mode, location_phaseB_mode, location_phaseC_mode;
static  FSK_mode  slice_phase_mode;

uint64_t PhaseSampleValue;
uint64_t PhaseTickOutValue, PhaseATickOutValue, PhaseBTickOutValue, PhaseCTickOutValue;
/*全局的相位值，在任一时间的绝对相位*/
unsigned char GloblePhaseValue;

static struct
{
    unsigned int CurrentSendBitValue: 1;
    unsigned int LastSendBitValue: 1;
    unsigned int Trans_mode: 1;
    unsigned int CallAfterInterrupt: 1;
    unsigned int phaseA_RxAslice: 1;
    unsigned int phaseB_RxAslice: 1;
    unsigned int phaseC_RxAslice: 1;
    unsigned int phaseA_TxAslice: 1;
    unsigned int phaseB_TxAslice: 1;
    unsigned int phaseC_TxAslice: 1;
    unsigned int DBG0: 1;
    unsigned int DBG1: 1;
    unsigned int DBG2: 1;
};

static uint8_t  Current_sample_send_times = 0;
static uint8_t  SendBitCount[3];

static uint16_t PhaseAPastZeroTick;
static uint16_t PhaseBPastZeroTick;
static uint16_t PhaseCPastZeroTick;

StructTypedefPhaseByteSample phase_byte_sample;


/**********************************************************************
***************************************************************************
************************************************************************/

void sample_eval_init( void )
{
    char* p;
    unsigned char i;
    location_phaseA_mode = RXmode;
    location_phaseB_mode = RXmode;
    location_phaseC_mode = RXmode;
    SendBitCount[0] = 0;
    SendBitCount[1] = 0;
    SendBitCount[2] = 0;
    p = ( char* )&phase_byte_sample;
    for( i = 0; i < sizeof( StructTypedefPhaseByteSample ); i++ )
    {
        *p++ = 0;
    }
}

/**********************************************************************
******************************************************************************************************
*********************************************************************************************************************/

void set_fsk_mode( FSK_mode mode, t_phase phase )
{
    if( phase == PhaseA )
    {
        location_phaseA_mode = mode;
    }
    else if( phase == PhaseB )
    {
        location_phaseB_mode = mode;
    }
    else if( phase == PhaseC )
    {
        location_phaseC_mode = mode;
    }
}
/**********************************************************************
*******************************************************************************
*********************************************************************************************/

FSK_mode get_fsk_mode( t_phase phase )
{
    if( phase == PhaseA )
    {
        return location_phaseA_mode;
    }
    else if( phase == PhaseB )
    {
        return  location_phaseB_mode;
    }
    else
    {
        return  location_phaseC_mode;
    }
}

unsigned char get_phase_power_value( void )
{
    uint8_t val;
    val = 0;
    if( PhaseAPastZeroTick < 30 )
    {
        val |= 0x01;
    }
    if( PhaseBPastZeroTick < 30 )
    {
        val |= 0x02;
    }
    if( PhaseCPastZeroTick < 30 )
    {
        val |= 0x04;
    }
    return val;
}
/**********************************************************************
***************************准备开始发送数据****************************
**********************************************************************/
void start_send_buf( unsigned char mode, t_phase phase )
{
    set_fsk_mode( TXmode, phase );
}


/******************************************************************
****************************结束发送流程*****************************
*****************************************************************/
void stop_send_buf( t_phase phase )
{
    set_fsk_mode( RXmode, phase );
}

/**********************************************************************************
**************************以指定的波特率发送一字节数据**********************************/

unsigned char LowLevel_send_byte( unsigned char* p, unsigned char sBandrate, t_phase phase )
{
    unsigned char err = 0;
    unsigned char tmp;
    __IO uint64_t buf;
    uint8_t sendbitevalue = 0;
    tmp = *p;
    sendbitevalue = SendBitCount[phase];
    if( sBandrate == 0 )
    {
        uint8_t i;
        buf = 0;
        for( i = 0; i < 4; i++ )
        {
            buf >>= 16;
            if( tmp & ( 0x80 >> ( sendbitevalue + i ) ) )
            {
                buf += 0xffff000000000000L;
            }
        }
    }
    else if( sBandrate == 1 )
    {
        uint8_t i;
        buf = 0;
        for( i = 0; i < 2; i++ )
        {
            buf >>= 32;
            if( tmp & ( 0x80 >> ( sendbitevalue + i ) ) )
            {
                buf += 0xffffffff00000000L;
            }
        }
    }
    else if( sBandrate == 2 )
    {
        if( tmp & ( 0x80 >> sendbitevalue ) )
        {
            buf = 0xffffffffffffffffL;
        }
        else
        {
            buf = 0;
        }
    }
    else
    {
        uint8_t i;
        buf = 0;
        for( i = 0; i < 8; i++ )
        {
            buf >>= 8;
            if( tmp & ( 0x80 >> ( sendbitevalue + i ) ) )
            {
                buf += 0xff00000000000000L;
            }
        }
    }
    err = LowLevel_send_slice( buf, phase );
    if( err )
    {
        if( sBandrate == 0 )
        {
            sendbitevalue += 4;
        }
        else if( sBandrate == 1 )
        {
            sendbitevalue += 2;
        }
        else if( sBandrate == 2 )
        {
            sendbitevalue += 1;
        }
        else
        {
            sendbitevalue += 8;
        }
        if( sendbitevalue > 7 )
        {
            sendbitevalue = 0;
        }
        else
        {
            err = 0;
        }
    }
    SendBitCount[phase] = sendbitevalue;
    return err;
}

unsigned char LowLevel_send_slice( uint64_t buf, t_phase phase )
{
    switch( phase )
    {
        case PhaseA:
            if( phaseA_TxAslice )
            {
                phaseA_TxAslice = 0;
                PhaseATickOutValue = buf;
                return 1;
            }
            break;
        case PhaseB:
            if( phaseB_TxAslice )
            {
                phaseB_TxAslice = 0;
                PhaseBTickOutValue = buf;
                return 1;
            }
            break;
        case PhaseC:
            if( phaseC_TxAslice )
            {
                phaseC_TxAslice = 0;
                PhaseCTickOutValue = buf;
                return 1;
            }
            break;
        default:
            break;
    }
    return 0;
}


unsigned char lowLevel_get_800bps_phase_data( unsigned char* p, unsigned char phase )
{
    unsigned char flag, tmp_buf, m;
    flag = 0;
    phase = phase % 3;
    m = phase_byte_sample.bit_sample[phase].accept_bit_count_800;
    if( m >= 8 )
    {
        flag = 1;
        __disable_interrupt();
        tmp_buf = phase_byte_sample.byte_sample[phase].buf_800;
        phase_byte_sample.bit_sample[phase].accept_bit_count_800 = 0;
        __enable_interrupt();
        *p = tmp_buf;
    }
    return flag;
}

unsigned char lowLevel_get_800bps_phase_data2( unsigned char* p, unsigned char phase )
{
    unsigned char flag, tmp_buf, m;
    flag = 0;
    phase = phase % 3;
    m = phase_byte_sample.bit_sample[phase].accept_bit_count_800;
    if( m >= 8 )
    {
        flag = 1;
        __disable_interrupt();
        tmp_buf = phase_byte_sample.byte_sample[phase].buf_800;
        phase_byte_sample.bit_sample[phase].accept_bit_count_800 = 0;
        __enable_interrupt();
        *p = tmp_buf;
    }
    return flag;
}

/**********************************************************************
*********************************取400bps的一个时间片数据*******************************
****************************************************************************/

unsigned char lowLevel_get_400bps_phase_data( unsigned char* p, unsigned char phase )
{
    unsigned char flag, tmp_buf, m;
    flag = 0;
    phase = phase % 3;
    m = phase_byte_sample.bit_sample[phase].accept_bit_count_400; //用400去计算数据是否正确
    if( m >= 4 )
    {
        flag = 1;
        __disable_interrupt();
        tmp_buf = phase_byte_sample.byte_sample[phase].buf_400;
        phase_byte_sample.bit_sample[phase].accept_bit_count_400 = 0;
        __enable_interrupt();
        *p = tmp_buf;
    }
    return flag;
}
/**********************************************************************
*****************取400bps的一个字节数据（两时间片数据）************************
************************************************************************************/

unsigned char lowLevel_get_400bps_phase_data2( unsigned char* p, unsigned char phase )
{
    unsigned char flag, tmp_buf, m;
    flag = 0;
    phase = phase % 3;
    m = phase_byte_sample.bit_sample[phase].accept_bit_count_400;
    if( m >= 8 )
    {
        flag = 1;
        __disable_interrupt();
        tmp_buf = phase_byte_sample.byte_sample[phase].buf_400;
        phase_byte_sample.bit_sample[phase].accept_bit_count_400 = 0;
        __enable_interrupt();
        *p = tmp_buf;
    }
    return flag;
}

/**********************************************************************
****************************以200bps取一个时间片数据********************
*************************************************************************/

unsigned char lowLevel_get_200bps_phase_data( unsigned char* p, unsigned char phase )
{
    unsigned char flag, tmp_buf, m;
    flag = 0;
    phase = phase % 3;
    m = phase_byte_sample.bit_sample[phase].accept_bit_count_200;
    if( m >= 2 )
    {
        flag = 1;
        __disable_interrupt();
        tmp_buf = phase_byte_sample.byte_sample[phase].buf_200;
        phase_byte_sample.bit_sample[phase].accept_bit_count_200 = 0;
        __enable_interrupt();
        *p = tmp_buf;
    }
    return flag;
}
/**********************************************************************
**************************以200bps取一字节数据（4个时间片）***************************
***************************************************************************************/
unsigned char lowLevel_get_200bps_phase_data2( unsigned char* p, unsigned char phase )
{
    unsigned char flag, tmp_buf, m;
    flag = 0;
    phase = phase % 3;
    m = phase_byte_sample.bit_sample[phase].accept_bit_count_200;
    if( m >= 8 )
    {
        flag = 1;
        __disable_interrupt();
        tmp_buf = phase_byte_sample.byte_sample[phase].buf_200;
        phase_byte_sample.bit_sample[phase].accept_bit_count_200 = 0;
        __enable_interrupt();
        *p = tmp_buf;
    }
    return flag;
}

/**********************************************************************
*************************以100bps取一个时间片数据***********************
*********************************************************************/

unsigned char lowLevel_get_100bps_phase_data( unsigned char* p, unsigned char phase )
{
    unsigned char flag, tmp_buf, m;
    flag = 0;
    phase = phase % 3;
    m = phase_byte_sample.bit_sample[phase].accept_bit_count_100;
    if( m >= 1 )
    {
        flag = 1;
        __disable_interrupt();
        tmp_buf = phase_byte_sample.byte_sample[phase].buf_100;
        phase_byte_sample.bit_sample[phase].accept_bit_count_100 = 0;
        __enable_interrupt();
        *p = tmp_buf;
    }
    return flag;
}
/**********************************************************************
*****************以100bps取一个字节数据（8个时间片数据）*****************************
*********************************************************************************************************************/

unsigned char lowLevel_get_100bps_phase_data2( unsigned char* p, unsigned char phase )
{
    unsigned char flag, tmp_buf, m;
    flag = 0;
    phase = phase % 3;
    m = phase_byte_sample.bit_sample[phase].accept_bit_count_100;
    if( m >= 8 )
    {
        flag = 1;
        __disable_interrupt();
        tmp_buf = phase_byte_sample.byte_sample[phase].buf_100;
        phase_byte_sample.bit_sample[phase].accept_bit_count_100 = 0;
        __enable_interrupt();
        *p = tmp_buf;
    }
    return flag;
}

/*同步信号处理，同步时间片信息*/
void Signal_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
    if( HAL_GPIO_ReadPin( SIG_PIN1_GPIO_PORT, GPIO_Pin ) != GPIO_PIN_RESET )
    {
        return;
    }
    if( HAL_GPIO_ReadPin( SIG_PIN1_GPIO_PORT, GPIO_Pin ) != GPIO_PIN_RESET )
    {
        return;
    }
    /*关中断*/
    PHASE_TIME->CR1 &= ~( TIM_CR1_CEN );
    if( PHASE_TIME->CNT > 6000 ) //没有产生过中断;
    {
        NVIC_SetPendingIRQ( PHASE_TIME_IRQn );
        // NVIC->ISPR[0] = 0x00100000;//1<< PHASE_TIME_IRQn;
        CallAfterInterrupt = 1;
    }
    else if( NVIC_GetPendingIRQ( PHASE_TIME_IRQn ) )
        //else if(NVIC->ISPR[0] &0x00100000)
    {
        CallAfterInterrupt = 1;
    }
    PHASE_TIME->SR = ~( TIM_IT_UPDATE );
    PHASE_TIME->CNT = 0;
    PHASE_TIME->CR1 |= TIM_CR1_CEN;
    switch( GPIO_Pin )
    {
        case SYNC_PIN1:
            if( CallAfterInterrupt )
            {
                PhaseAPastZeroTick = 0;    //A相过零后经过的时间片数量。每个时间片为3333US。
            }
            else
            {
                PhaseAPastZeroTick = 1;
            }
            break;
        case SYNC_PIN2:
            if( CallAfterInterrupt )
            {
                PhaseBPastZeroTick = 0;    //B相过零后经过的时间片数量。每个时间片为3333US。
            }
            else
            {
                PhaseBPastZeroTick = 1;
            }
            break;
        case SYNC_PIN3:
            if( CallAfterInterrupt )
            {
                PhaseCPastZeroTick = 0;    //B相过零后经过的时间片数量。每个时间片为3333US。
            }
            else
            {
                PhaseCPastZeroTick = 1;
            }
            break;
        default:
            return;
            break;
    }
    CallAfterInterrupt = 0;
}
/*每3.33mS操作一次*/
void PhaseTimeCallBack( void )
{
    if( PhaseAPastZeroTick < 1000 )
    {
        PhaseAPastZeroTick++;
    }
    if( PhaseBPastZeroTick < 1000 )
    {
        PhaseBPastZeroTick++;
    }
    if( PhaseCPastZeroTick < 1000 )
    {
        PhaseCPastZeroTick++;
    }
    if( PhaseAPastZeroTick < 15 ) /*A相上电*/
    {
        GloblePhaseValue = ( PhaseAPastZeroTick - 1 ) % 3;
    }
    else if( PhaseBPastZeroTick < 15 )
    {
        GloblePhaseValue = ( PhaseBPastZeroTick ) % 3;
    }
    else if( PhaseCPastZeroTick < 15 )
    {
        GloblePhaseValue = ( PhaseCPastZeroTick + 1 ) % 3;
    }
    else
    {
        GloblePhaseValue ++;
        if( GloblePhaseValue >= 3 )
        {
            GloblePhaseValue = 0;
        }
    }
    SelectControlPinChannal( GloblePhaseValue );
    TimeSliceProcessPhase();
    /*just for test*/
    //   BSP_LED_On(LED_BLUE);
}
/*每3.33mS操作一次*/
void TimeSliceProcessPhase( void )
{
    Current_sample_send_times = 0;
    DisableFskOutATime();
    DisableFskOutBTime();
    DisableFskOutCTime();
    switch( GloblePhaseValue )
    {
        case 0:
            slice_phase_mode = location_phaseA_mode;
            break;
        case 1:
            slice_phase_mode = location_phaseB_mode;
            break;
        case 2:
            slice_phase_mode = location_phaseC_mode;
            break;
        default:
            break;
    }
    if( slice_phase_mode == RXmode )
    {
        SAMP_TIME_ENABLE();
    }
    else
    {
        SAMP_TIME_ENABLE_EX();
    }
}
/*44.8us操作一次*/
void SampleTimeCallBack( void )
{
    if( Current_sample_send_times == 0 )
    {
        if( slice_phase_mode == TXmode )
        {
            if( GloblePhaseValue == 0 )
            {
                PhaseTickOutValue = PhaseATickOutValue;
            }
            else if( GloblePhaseValue == 1 )
            {
                PhaseTickOutValue = PhaseBTickOutValue;
            }
            else
            {
                PhaseTickOutValue = PhaseCTickOutValue;
            }
            if( PhaseTickOutValue & 0x01 )
            {
                LastSendBitValue = 0;
            }
            else
            {
                LastSendBitValue = 1;
            }
        }
    }
    if( slice_phase_mode == RXmode )
    {
#ifdef USE_ADC_SAMPLE
#else
        if( TEST_SIG_PIN1 )
        {
            PhaseSampleValue |= 1;
        }
#endif
        PhaseSampleValue <<= 1;
    }
    else
    {
        CurrentSendBitValue = ( PhaseTickOutValue & 0x01 );
        if( ( LastSendBitValue != CurrentSendBitValue ) )
        {
#ifdef  ISPLC_13_1X
            if( CurrentSendBitValue )
            {
                FskOutAHighBitFreq();
            }
            else
            {
                FskOutALowBitFreq();
            }
#else
            if( GloblePhaseValue == 0 )
            {
                if( CurrentSendBitValue )
                {
                    FskOutAHighBitFreq();
                }
                else
                {
                    FskOutALowBitFreq();
                }
            }
            else if( GloblePhaseValue == 1 )
            {
                if( CurrentSendBitValue )
                {
                    FskOutBHighBitFreq();
                }
                else
                {
                    FskOutBLowBitFreq();
                }
            }
            else
            {
                if( CurrentSendBitValue )
                {
                    FskOutCHighBitFreq();
                }
                else
                {
                    FskOutCLowBitFreq();
                }
            }
#endif
            LastSendBitValue  =  CurrentSendBitValue;
        }
        PhaseTickOutValue >>= 1;
    }
    Current_sample_send_times ++;
    /*发送或接收完一个过零数据处理*/
    if( Current_sample_send_times >= 64 )
    {
        Current_sample_send_times = 0;
        SAMP_TIME_DISABLE();
        if( slice_phase_mode == RXmode )
        {
#ifndef USE_ADC      
            relatedprocess( PhaseSampleValue, GloblePhaseValue );
#endif
        }
        else
        {
            if( GloblePhaseValue == 0 )
            {
                DisableFskOutATime();
                phaseA_TxAslice = 1;
            }
            else if( GloblePhaseValue == 1 )
            {
#ifdef  ISPLC_13_1X
                DisableFskOutATime();
#else
                DisableFskOutBTime();
#endif
                phaseB_TxAslice = 1;
            }
            else if( GloblePhaseValue == 2 )
            {
#ifdef  ISPLC_13_1X
                DisableFskOutATime();
#else
                DisableFskOutCTime();
#endif
                phaseC_TxAslice = 1;
            }
        }
    }
}

#define SOFTWARE_RELATED
/*****************************************************************************
*********************软硬件相关性判断程序*************************************
****************************************************************************/
#define NUM_OF_SAMPLE  (672)
#define NUM_OF_SAMPLE_DIV2  (uint32_t)(NUM_OF_SAMPLE/2)
#define NUM_OF_SAMPLE_DIV4  (uint32_t)(NUM_OF_SAMPLE/4)
#define NUM_OF_SAMPLE_DIV8  (uint32_t)(NUM_OF_SAMPLE/8)
#define NUM_OF_SAMPLE_DIV16 (uint32_t)(NUM_OF_SAMPLE/16)
#define ADC_GATE   170
#define TOTLE_800BPS_GATE (uint32_t)(ADC_GATE*NUM_OF_SAMPLE_DIV16)
#define TOTLE_400BPS_GATE (uint32_t)(ADC_GATE*NUM_OF_SAMPLE_DIV8)
#define TOTLE_200BPS_GATE (uint32_t)(ADC_GATE*NUM_OF_SAMPLE_DIV4)
#define TOTLE_100BPS_GATE (ADC_GATE*NUM_OF_SAMPLE_DIV2)


void ADC_SampleRelatedProcess( uint8_t* ADCBuff )
{
    // BSP_LED_On(LED_BLUE);
    uint8_t* p;
    uint8_t tmp_buf, n;
    uint16_t i, j;
    //800bps
    tmp_buf = 0;
    for( i = 0; i < 8; i++ )
    {
        uint32_t accbuf = 0;
        tmp_buf <<= 1;
        p = &ADCBuff[NUM_OF_SAMPLE_DIV8 * i];
        for( j = 0; j < NUM_OF_SAMPLE_DIV16; j++ )
        {
            accbuf += *p;
            p += 2;
        }
        if( accbuf < TOTLE_800BPS_GATE )
        {
            tmp_buf += 1;
        }
    }
    phase_byte_sample.byte_sample[GloblePhaseValue].buf_800 = tmp_buf;
    phase_byte_sample.bit_sample[GloblePhaseValue].accept_bit_count_800 += 8;
    /*
    switch(GloblePhaseValue)
    {
      case 0:
        phase_byte_sample.byte_sample[0].buf_800 = tmp_buf;
        phase_byte_sample.bit_sample[0].accept_bit_count_800+=8;
        break;
      case 1:
        phase_byte_sample.byte_sample[1].buf_800 = tmp_buf;
        phase_byte_sample.bit_sample[1].accept_bit_count_800+=8;
        break;
      case 2:
        phase_byte_sample.byte_sample[2].buf_800 = tmp_buf;
        phase_byte_sample.bit_sample[2].accept_bit_count_800+=8;
        break;
      default:break;
    } */
    //400bps
    tmp_buf = 0;
    for( i = 0; i < 4; i++ )
    {
        uint32_t accbuf = 0;
        tmp_buf <<= 1;
        p = &ADCBuff[NUM_OF_SAMPLE_DIV4 * i];
        for( j = 0; j < NUM_OF_SAMPLE_DIV8; j++ )
        {
            accbuf += *p;
            p += 2;
        }
        if( accbuf < TOTLE_400BPS_GATE )
        {
            tmp_buf += 1;
        }
    }
    n = phase_byte_sample.byte_sample[GloblePhaseValue].buf_400;
    n <<= 4;
    phase_byte_sample.byte_sample[GloblePhaseValue].buf_400 = ( n | tmp_buf );
    phase_byte_sample.bit_sample[GloblePhaseValue].accept_bit_count_400 += 4;
    //200bps
    tmp_buf = 0;
    for( i = 0; i < 2; i++ )
    {
        uint32_t accbuf = 0;
        tmp_buf <<= 1;
        p = &ADCBuff[NUM_OF_SAMPLE_DIV2 * i];
        for( j = 0; j < NUM_OF_SAMPLE_DIV4; j++ )
        {
            accbuf += *p;
            p += 2;
        }
        if( accbuf < TOTLE_200BPS_GATE )
        {
            tmp_buf += 1;
        }
    }
    n = phase_byte_sample.byte_sample[GloblePhaseValue].buf_200;
    n <<= 2;
    phase_byte_sample.byte_sample[GloblePhaseValue].buf_200 = ( n | tmp_buf );
    phase_byte_sample.bit_sample[GloblePhaseValue].accept_bit_count_200 += 2;
    //100bps
    tmp_buf = 0;
    uint32_t accbuf = 0;
    p = ADCBuff;
    for( j = 0; j < NUM_OF_SAMPLE_DIV2; j++ )
    {
        accbuf += *p;
        p += 2;
    }
    if( accbuf < TOTLE_100BPS_GATE )
    {
        tmp_buf += 1;
    }
    n = phase_byte_sample.byte_sample[GloblePhaseValue].buf_100;
    n <<= 1;
    phase_byte_sample.byte_sample[GloblePhaseValue].buf_100 = ( n | tmp_buf );
    phase_byte_sample.bit_sample[GloblePhaseValue].accept_bit_count_100 += 1;
    //BSP_LED_Off(LED_BLUE);
}

void relatedprocess( uint64_t sPhase, unsigned char phase )
{
    unsigned char tmp_buf, m, n, i, j, k;
    uint64_t same1;
#ifdef SOFTWARE_RELATED
    // const u16 s_codec1 = 0xe63c;
    const uint64_t s_codec1 = 0xffffffffffffffff;
#else
    const uint64_t s_codec1 = 0xffffffffffffffff;  /*硬件相关时，不用软件相关处理*/
#endif
    /*800bps receive*/
    same1 = sPhase ^ s_codec1;
    tmp_buf = 0;
    for( j = 0; j < 8; j++ )
    {
        k = 0;
        tmp_buf <<= 1;
        for( i = 0; i < 8; i++ )
        {
            if( same1 & 0x8000000000000000 )
            {
                k++;
            }
            same1 <<= 1;
        }
        if( k <= 4 )
        {
            tmp_buf += 1;    //1
        }
    } /*
  n = phase_byte_sample.byte_sample[phase].buf_800;
  n<<=8;
  m = n|tmp_buf;
  phase_byte_sample.byte_sample[phase].buf_800 = m;*/
    phase_byte_sample.byte_sample[phase].buf_800 = tmp_buf;
    phase_byte_sample.bit_sample[phase].accept_bit_count_800 += 8;
    /*400bps receive*/
    same1 = sPhase ^ s_codec1;
    tmp_buf = 0;
    for( j = 0; j < 4; j++ )
    {
        k = 0;
        tmp_buf <<= 1;
        for( i = 0; i < 16; i++ )
        {
            if( same1 & 0x8000000000000000 )
            {
                k++;
            }
            same1 <<= 1;
        }
        if( k <= 8 )
        {
            tmp_buf += 1;    //1
        }
    }
    n = phase_byte_sample.byte_sample[phase].buf_400;
    n <<= 4;
    m = n | tmp_buf;
    phase_byte_sample.byte_sample[phase].buf_400 = m;
    phase_byte_sample.bit_sample[phase].accept_bit_count_400 += 4;
    /*200bps*/
    same1 = sPhase ^ s_codec1;
    tmp_buf = 0;
    for( j = 0; j < 2; j++ )
    {
        k = 0;
        tmp_buf <<= 1;
        for( i = 0; i < 32; i++ )
        {
            if( same1 & 0x8000000000000000 )
            {
                k++;
            }
            same1 <<= 1;
        }
        if( k <= 16 )
        {
            tmp_buf += 1;    //1
        }
    }
    n = phase_byte_sample.byte_sample[phase].buf_200;
    n <<= 2;
    m = n | tmp_buf;
    phase_byte_sample.byte_sample[phase].buf_200 = m;
    phase_byte_sample.bit_sample[phase].accept_bit_count_200 += 2;
    /*100bps*/
    same1 = sPhase ^ s_codec1;
    tmp_buf = 0;
    k = 0;
    for( i = 0; i < 64; i++ )
    {
        if( same1 & 0x8000000000000000 )
        {
            k++;
        }
        same1 <<= 1;
    }
    if( k <= 32 )
    {
        tmp_buf += 1;    //1
    }
    n = phase_byte_sample.byte_sample[phase].buf_100;
    n <<= 1;
    m = n | tmp_buf;
    phase_byte_sample.byte_sample[phase].buf_100 = m;
    phase_byte_sample.bit_sample[phase].accept_bit_count_100 += 1;
}