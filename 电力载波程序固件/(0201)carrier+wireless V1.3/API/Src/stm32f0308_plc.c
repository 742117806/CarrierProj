/**2015-03-24*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f0308_plc.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32F0308_DISCOVERY
  * @{
  */

/** @defgroup STM32F0308_DISCOVERY_LOW_LEVEL
  * @brief This file provides firmware functions to manage Leds and push-buttons,
  *        available on STM32F0308_DISCOVERY evaluation board from STMicroelectronics.
  * @{
  */

/** @defgroup STM32F0308_DISCOVERY_LOW_LEVEL_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32F0308_DISCOVERY_LOW_LEVEL_Private_Defines
  * @{
  */

/**
  * @brief STM32F0308 DISCO BSP Driver version number V2.0.0
  */
#define __STM32F0308_DISCO_BSP_VERSION_MAIN (0x03) /*!< [31:24] main version */
#define __STM32F0308_DISCO_BSP_VERSION_SUB1 (0x00) /*!< [23:16] sub1 version */
#define __STM32F0308_DISCO_BSP_VERSION_SUB2 (0x00) /*!< [15:8]  sub2 version */
#define __STM32F0308_DISCO_BSP_VERSION_RC (0x00)   /*!< [7:0]  release candidate */
#define __STM32F0308_DISCO_BSP_VERSION ((__STM32F0308_DISCO_BSP_VERSION_MAIN << 24) | \
                                        (__STM32F0308_DISCO_BSP_VERSION_SUB1 << 16) | \
                                        (__STM32F0308_DISCO_BSP_VERSION_SUB2 << 8) |  \
                                        (__STM32F0308_DISCO_BSP_VERSION_RC))

/**
  * @}
  */

USART_TypeDef* COM_USART[COMn] = {EVAL_COM1, EVAL_COM2};

GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT, EVAL_COM2_TX_GPIO_PORT};

GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT, EVAL_COM2_RX_GPIO_PORT};

const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN, EVAL_COM2_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN, EVAL_COM2_RX_PIN};

const uint16_t COM_TX_AF[COMn] = {EVAL_COM1_TX_AF, EVAL_COM2_TX_AF};

const uint16_t COM_RX_AF[COMn] = {EVAL_COM1_RX_AF, EVAL_COM2_RX_AF};

GPIO_TypeDef* LED_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT, LED3_GPIO_PORT, LED4_GPIO_PORT};
const uint16_t LED_PIN[LEDn] = {LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN};

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT};
const uint16_t BUTTON_PIN[BUTTONn] = {USER_BUTTON_PIN};
const uint8_t BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn};

/**
  * @}
  */

/** @defgroup STM32F0308_DISCOVERY_LOW_LEVEL_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F0308_DISCOVERY_LOW_LEVEL_Private_Functions
  * @{
  */

/**
  * @brief  This method returns the STM32F0308 DISCO BSP Driver revision
  * @param  None
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion( void )
{
    return __STM32F0308_DISCO_BSP_VERSION;
}

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void BSP_LED_Init( Led_TypeDef Led )
{
    GPIO_InitTypeDef GPIO_InitStruct;
    /* Enable the GPIO_LED clock */
    LEDx_GPIO_CLK_ENABLE( Led );
    /* Configure the GPIO_LED pin */
    GPIO_InitStruct.Pin = LED_PIN[Led];
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init( LED_PORT[Led], &GPIO_InitStruct );
    HAL_GPIO_WritePin( LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET );
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void BSP_LED_On( Led_TypeDef Led )
{
    HAL_GPIO_WritePin( LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET );
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void BSP_LED_Off( Led_TypeDef Led )
{
    HAL_GPIO_WritePin( LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET );
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void BSP_LED_Toggle( Led_TypeDef Led )
{
    HAL_GPIO_TogglePin( LED_PORT[Led], LED_PIN[Led] );
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_USER
  * @param  Mode: Specifies Button mode.
  *   This parameter can be one of the following values:
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line
  *                            with interrupt generation capability
  * @retval None
  */
void BSP_PB_Init( Button_TypeDef Button, ButtonMode_TypeDef Mode )
{
    GPIO_InitTypeDef GPIO_InitStruct;
    /* Enable the BUTTON Clock */
    BUTTONx_GPIO_CLK_ENABLE( Button );
    __SYSCFG_CLK_ENABLE();
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    if( Mode == BUTTON_MODE_GPIO )
    {
        /* Configure Button pin as input */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init( BUTTON_PORT[Button], &GPIO_InitStruct );
    }
    if( Mode == BUTTON_MODE_EXTI )
    {
        /* Configure Button pin as input with External interrupt */
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        HAL_GPIO_Init( BUTTON_PORT[Button], &GPIO_InitStruct );
        /* Enable and set Button EXTI Interrupt to the lowest priority */
        HAL_NVIC_SetPriority( ( IRQn_Type )( BUTTON_IRQn[Button] ), 0x03, 0x00 );
        HAL_NVIC_EnableIRQ( ( IRQn_Type )( BUTTON_IRQn[Button] ) );
    }
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_USER
  * @retval The Button GPIO pin value.
  */
uint32_t BSP_PB_GetState( Button_TypeDef Button )
{
    return HAL_GPIO_ReadPin( BUTTON_PORT[Button], BUTTON_PIN[Button] );
}

/**
  * @}
  */
/**
  * @brief  Configures COM port.
  * @param  COM: COM port to be configured.
  *          This parameter can be one of the following values:
  *            @arg  COM1
  *            @arg  COM2
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains the
  *                configuration information for the specified USART peripheral.
  * @retval None
  */
void BSP_COM_Init( COM_TypeDef COM, UART_HandleTypeDef* huart )
{
    GPIO_InitTypeDef GPIO_InitStruct;
    /* Enable GPIO clock */
    EVAL_COMx_TX_GPIO_CLK_ENABLE( COM );
    EVAL_COMx_RX_GPIO_CLK_ENABLE( COM );
    /* Enable USART clock */
    EVAL_COMx_CLK_ENABLE( COM );
    /* Configure USART Tx as alternate function */
    GPIO_InitStruct.Pin = COM_TX_PIN[COM];
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = COM_TX_AF[COM];
    HAL_GPIO_Init( COM_TX_PORT[COM], &GPIO_InitStruct );
    /* Configure USART Rx as alternate function */
    GPIO_InitStruct.Pin = COM_RX_PIN[COM];
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = COM_RX_AF[COM];
    HAL_GPIO_Init( COM_RX_PORT[COM], &GPIO_InitStruct );
    /* USART configuration */
    huart->Instance = COM_USART[COM];
    HAL_UART_Init( huart );
}
/**
  * @}
  */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
