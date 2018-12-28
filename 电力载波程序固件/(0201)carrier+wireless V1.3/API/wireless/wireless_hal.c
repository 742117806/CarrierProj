#include "stm32f0xx.h"
#include "wireless_hal.h"



void HAL_SPI_MspInit( SPI_HandleTypeDef* hspi )
{
    GPIO_InitTypeDef GPIO_InitStruct;
    if( hspi->Instance == SPI1 )
    {
        /* USER CODE BEGIN SPI1_MspInit 0 */
        /* USER CODE END SPI1_MspInit 0 */
        /* Peripheral clock enable */
        __SPI1_CLK_ENABLE();
        /**SPI1 GPIO Configuration
        PB3     ------> SPI1_SCK
        PB4     ------> SPI1_MISO
        PB5     ------> SPI1_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
        HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );
        /* USER CODE BEGIN SPI1_MspInit 1 */
        /* USER CODE END SPI1_MspInit 1 */
    }
}

void HAL_SPI_MspDeInit( SPI_HandleTypeDef* hspi )
{
    if( hspi->Instance == SPI1 )
    {
        /* USER CODE BEGIN SPI1_MspDeInit 0 */
        /* USER CODE END SPI1_MspDeInit 0 */
        /* Peripheral clock disable */
        __SPI1_CLK_DISABLE();
        /**SPI1 GPIO Configuration
        PB3     ------> SPI1_SCK
        PB4     ------> SPI1_MISO
        PB5     ------> SPI1_MOSI
        */
        HAL_GPIO_DeInit( GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 );
        /* USER CODE BEGIN SPI1_MspDeInit 1 */
        /* USER CODE END SPI1_MspDeInit 1 */
    }
}



uint8_t SPI_RWbyte( uint8_t sdata )
{
    extern SPI_HandleTypeDef hspi1;
    uint8_t rData;
    HAL_SPI_TransmitReceive( &hspi1, &sdata, &rData, 1, 100 );
    return  rData;
}





//////////// 调用系统的延时函数供本文件使用 ////////////
void Si4438_Delay_ms( uint16_t nms )
{
    HAL_Delay( nms );
}
