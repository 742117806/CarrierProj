/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "main_process.h"
#include "stm32f030_peripheral.h"
//#include "wireless_app.h"
#include "stdio.h"
#include "wireless_app.h"
#include "wireless_hal.h"


/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef IwdgHandle;
__IO uint32_t time_tick;
//WLS WIRELESS_DATA;                  //定义无线结构体
volatile uint8_t WIRELESS_LED_FLAG; //无线接收状态指示标志
SPI_HandleTypeDef hspi1;              //无线模块驱动接口定义


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config( void );
static void Error_Handler( void );
static void BoardEvalConfig( void );
#ifdef USE_WDT
static void IWDT_Config( void );
#endif
void SysTickCallback( void );



/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler( char* file, int line )
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while( 1 )
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init( void )
{
    GPIO_InitTypeDef GPIO_InitStruct;
    /* GPIO Ports Clock Enable */
    __GPIOF_CLK_ENABLE();
    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin( GPIOA, LEDG_Pin | LEDR_Pin | SI4438_NSS_Pin, GPIO_PIN_RESET );
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin( SI4438_SDN_GPIO_Port, SI4438_SDN_Pin, GPIO_PIN_RESET );
    /*Configure GPIO pins : LEDG_Pin LEDR_Pin */
    GPIO_InitStruct.Pin = LEDG_Pin | LEDR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );
    /*Configure GPIO pin : SI4438_SDN_Pin */
    GPIO_InitStruct.Pin = SI4438_SDN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init( SI4438_SDN_GPIO_Port, &GPIO_InitStruct );
    /*Configure GPIO pin : SI4438_nIRQ_Pin */
    GPIO_InitStruct.Pin = SI4438_nIRQ_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( SI4438_nIRQ_GPIO_Port, &GPIO_InitStruct );
    /*Configure GPIO pin : SI4438_NSS_Pin */
    GPIO_InitStruct.Pin = SI4438_NSS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init( SI4438_NSS_GPIO_Port, &GPIO_InitStruct );
    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority( EXTI4_15_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( EXTI4_15_IRQn );
}

/* SPI1 init function */
static void MX_SPI1_Init( void )
{
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLED;
    if( HAL_SPI_Init( &hspi1 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }
    SPI_RWbyte( 0xFF );
}


void Get_WireLessChannel( uint8_t* wire_chnel )
{
    extern unsigned char  RF_RX_HOP_CONTROL_12[];
    extern uint8_t const Channel_Frequency_Index[66];
    uint32_t temp_val = 0x00 + 0x2A + 0x5B;
    if( temp_val == 0 )
    {
        wire_chnel[0] = Default_Channel;
        wire_chnel[1] = Default_Channel;
    }
    else
    {
        wire_chnel[0] = ( temp_val & 0x1f ) << 1;
        wire_chnel[1] = wire_chnel[0] + 1;
        if( wire_chnel[0] == Default_Channel )
        {
            wire_chnel[0] = wire_chnel[0] + 2;
            wire_chnel[1] = wire_chnel[0] + 1;
        }
    }
    RF_RX_HOP_CONTROL_12[7] = Channel_Frequency_Index[wire_chnel[0]];
    RF_RX_HOP_CONTROL_12[8] = Channel_Frequency_Index[wire_chnel[1]];
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main( void )
{
    HAL_Init();
    /* Configure the system clock to have a system clock = 48 Mhz */
    SystemClock_Config();
    /*Board Init*/
    BoardEvalConfig();
    MX_GPIO_Init();
    MX_SPI1_Init();
    Get_WireLessChannel( Wireless_Channel );
    Wireless_Init();
    Si4438_Receive_Start( Wireless_Channel[0] );
    DebugPrintf( "Modfiy Date = %s", __DATE__ );
    DebugPrintf( "\r\nVersion = %08x", BSP_GetVersion() );
#ifdef USE_WDT
    IWDT_Config();
#endif
    main_process();
}

static void SystemClock_Config( void )
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    /* Enable HSE Oscillator and Activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        Error_Handler();
    }
    /* Select PLL as system clock source and configure the HCLK and PCLK1 clocks dividers */
    RCC_ClkInitStruct.ClockType = ( RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 );
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    if( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 ) != HAL_OK )
    {
        Error_Handler();
    }
}

static void BoardEvalConfig( void )
{
    //BSP_LED_Init(LED1);
    //BSP_LED_Init(LED2);
    //BSP_LED_Init(LED3);
    //BSP_LED_Init(LED4);
    Init_lowlevel_para();
    USARTx_Init();
    // Crc_Init();
    SigInConfig();
    EXTI4_15_IRQHandler_Config();
    InitPWMTime();
    //DisablePWMOut();
    InitFskOutATime();
    DisableFskOutATime();
    InitFskOutBTime();
    DisableFskOutBTime();
    InitFskOutCTime();
    DisableFskOutCTime();
    /* 38KHz */
    //Init38KhzOutTime();
    InitSampeTime();
    DisableSampeTime();
    InitPhaseTime();
#ifdef USE_ADC
    SigInADCIOCofnig();
#endif
}

void SysTickCallback( void )
{
    static uint8_t LEDR_blink_cnt; //无线灯闪计数
    if( ( HAL_GetTick() % 10 ) == 0 )
    {
        time_tick++;
        /************** 无线灯闪判断 *******************/
        if( WIRELESS_LED_FLAG )
        {
            if( WIRELESS_LED_FLAG == 1 )
            {
                if( ( LEDR_blink_cnt & 0x03 ) == 0 )
                {
                    GPIOA->ODR ^= 1 << 1; //灯闪
                }
                if( ++LEDR_blink_cnt > 15 )
                {
                    LEDR_blink_cnt = 0;
                    GPIOA->BRR = 1 << 1;   //灯熄
                    WIRELESS_LED_FLAG = 0; //清标志
                }
            }
            else if( WIRELESS_LED_FLAG == 2 )
            {
                WIRELESS_LED_FLAG = 0; //清标志
                LEDR_blink_cnt = 0;    //计数清0
            }
        }
    }
    if( STDownUartFrame.Over_time_count )
    {
        STDownUartFrame.Over_time_count--;
        if( 0 == STDownUartFrame.Over_time_count )
        {
            STDownUartFrame.len_sub = STDownUartFrame.len;
            STDownUartFrame.len = 0;
            STDownUartFrame.is_flag |= 0x01;        //超时接收串口数据使用，如果不是超时接收，需要屏蔽
        }
    }
}
#ifdef USE_WDT
static void IWDT_Config( void )
{
    RCC_OscInitTypeDef oscinit = {0};
    RCC_PeriphCLKInitTypeDef periphclkinit = {0};
    /* Enable LSI Oscillator */
    oscinit.OscillatorType = RCC_OSCILLATORTYPE_LSI;
    oscinit.LSIState = RCC_LSI_ON;
    oscinit.PLL.PLLState = RCC_PLL_NONE;
    if( HAL_RCC_OscConfig( &oscinit ) != HAL_OK )
    {
        Error_Handler();
    }
    /* Configue LSI as RTC clock soucre */
    periphclkinit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    periphclkinit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    HAL_RCCEx_PeriphCLKConfig( &periphclkinit );
    __HAL_RCC_RTC_ENABLE();
    if( __HAL_RCC_GET_FLAG( RCC_FLAG_IWDGRST ) != RESET )
    {
        /* Clear reset flags */
        __HAL_RCC_CLEAR_RESET_FLAGS();
    }
    IwdgHandle.Instance = IWDG;
    IwdgHandle.Init.Prescaler = IWDG_PRESCALER_128;
    IwdgHandle.Init.Reload = 0x02EE; //2秒
    IwdgHandle.Init.Window = IWDG_WINDOW_DISABLE;
    if( HAL_IWDG_Init( &IwdgHandle ) != HAL_OK )
    {
        /* Initialization Error */
        Error_Handler();
    }
    /*##-4- Start the IWDG #####################################################*/
    if( HAL_IWDG_Start( &IwdgHandle ) != HAL_OK )
    {
        Error_Handler();
    }
}
#endif
static void Error_Handler( void )
{
    while( 1 )
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed( uint8_t* file, uint32_t line )
{
    while( 1 )
    {
    }
}
#endif

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
