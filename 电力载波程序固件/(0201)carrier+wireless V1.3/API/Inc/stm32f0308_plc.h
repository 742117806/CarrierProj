/**
  ******************************************************************************
  * @file    stm32f0308_discovery.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    20-May-2014
  * @brief   This file contains definitions for STM32F0308-Discovery's Leds, push-
  *          buttons hardware resources.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F0308_DISCOVERY_H
#define __STM32F0308_DISCOVERY_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32F0308_DISCOVERY
  * @{
  */

/** @addtogroup STM32F0308_DISCOVERY_LOW_LEVEL
  * @{
  */

/** @defgroup STM32F0308_DISCOVERY_LOW_LEVEL_Exported_Types
  * @{
  */

/**
 * @brief LED Types Definition
 */
typedef enum
{
  LED1 = 0,
  LED2 = 1,
  LED3 = 2,
  LED4 = 3,
  /* Color led aliases */
  LED_GREEN  = LED2,
  LED_BLUE   = LED2,
  LED_RED    = LED1,
  LED_RUN    = LED3,
  LED_STA    = LED3
}Led_TypeDef;

typedef enum 
{
  COM1 = 0,
  COM2 = 1,
  COM3 = 2
}COM_TypeDef;

/**
 * @brief LED Types Definition
 */
typedef enum
{
  SYNC1 = 0,
  SYNC2 = 1,
  SYNC3 = 2,
  SYNC4 = 3,
}SYNCn_TypeDef;
/**
 * @brief BUTTON Types Definition
 */
typedef enum 
{
  BUTTON_USER = 0
}Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;

/**
  * @}
  */ 

/** @defgroup STM32F0308_DISCOVERY_LOW_LEVEL_Exported_Constants
  * @{
  */ 
/** 
* @brief  Define for STM32F0308_DISCOVERY board  
*/ 
#if !defined (USE_STM320308_DISCO)
 #define USE_STM320308_DISCO
#endif

/** @addtogroup STM32F0308_DISCOVERY_LOW_LEVEL_LED
  * @{
  */
#define LEDn                             4


#define LED1_PIN                         GPIO_PIN_5
#define LED1_GPIO_PORT                   GPIOA
#define LED1_GPIO_CLK_ENABLE()           __GPIOA_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __GPIOA_CLK_DISABLE()
  
#define LED2_PIN                         GPIO_PIN_1
#define LED2_GPIO_PORT                   GPIOA
#define LED2_GPIO_CLK_ENABLE()           __GPIOA_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()          __GPIOA_CLK_DISABLE()


#define LED3_PIN                         GPIO_PIN_11
#define LED3_GPIO_PORT                   GPIOA
#define LED3_GPIO_CLK_ENABLE()           __GPIOA_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __GPIOA_CLK_DISABLE()
  
#define LED4_PIN                         GPIO_PIN_11
#define LED4_GPIO_PORT                   GPIOA
#define LED4_GPIO_CLK_ENABLE()           __GPIOA_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()          __GPIOA_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__LED__)   (((__LED__) == LED1) ? LED1_GPIO_CLK_ENABLE() :\
                                         ((__LED__) == LED2) ? LED2_GPIO_CLK_ENABLE() :\
                                         ((__LED__) == LED3) ? LED3_GPIO_CLK_ENABLE() :\
                                         ((__LED__) == LED4) ? LED4_GPIO_CLK_ENABLE() : 0 )

#define LEDx_GPIO_CLK_DISABLE(__LED__)  (((__LED__) == LED1) ? LED1_GPIO_CLK_DISABLE() :\
                                         ((__LED__) == LED2) ? LED2_GPIO_CLK_DISABLE() :\
                                         ((__LED__) == LED3) ? LED3_GPIO_CLK_DISABLE() :\
                                         ((__LED__) == LED4) ? LED4_GPIO_CLK_DISABLE() : 0 )
/**
  * @}
  */ 

/** @addtogroup STM32F0308_DISCOVERY_LOW_LEVEL_BUTTON
  * @{
  */  
#define BUTTONn                          1

/**
 * @brief USER push-button
 */
#define USER_BUTTON_PIN                GPIO_PIN_0                       /* PA0 */
#define USER_BUTTON_GPIO_PORT          GPIOA
#define USER_BUTTON_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE() __GPIOA_CLK_DISABLE()
#define USER_BUTTON_EXTI_IRQn          EXTI0_1_IRQn 

#define BUTTONx_GPIO_CLK_ENABLE(__BUTTON__)    (((__BUTTON__) == BUTTON_USER) ? USER_BUTTON_GPIO_CLK_ENABLE() : 0 )

#define BUTTONx_GPIO_CLK_DISABLE(__BUTTON__)   (((__BUTTON__) == BUTTON_USER) ? USER_BUTTON_GPIO_CLK_DISABLE() : 0 )


#define SYNCn                          3

/**
 * @brief USER push-button
 */
#define SYNC_PIN1                    GPIO_PIN_10                       /* PB10 */
#define SYNC_PIN1_GPIO_PORT          GPIOB
#define SYNC_PIN1_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define SYNC_PIN1_GPIO_CLK_DISABLE() __GPIOB_CLK_DISABLE()
#define SYNC_PIN1_EXTI_IRQn          EXTI4_15_IRQn 

#define SYNC_PIN2                    GPIO_PIN_11                       /* PB11 */
#define SYNC_PIN2_GPIO_PORT          GPIOB
#define SYNC_PIN2_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define SYNC_PIN2_GPIO_CLK_DISABLE() __GPIOB_CLK_DISABLE()
#define SYNC_PIN2_EXTI_IRQn          EXTI4_15_IRQn 

#define SYNC_PIN3                    GPIO_PIN_12                       /* PB12 */
#define SYNC_PIN3_GPIO_PORT          GPIOB
#define SYNC_PIN3_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define SYNC_PIN3_GPIO_CLK_DISABLE() __GPIOB_CLK_DISABLE()
#define SYNC_PIN3_EXTI_IRQn          EXTI4_15_IRQn 

#define SYNC_PINn_GPIO_CLK_ENABLE(__SYNC__)   (((__SYNC__) == SYNC1) ? SYNC_PIN1_GPIO_CLK_ENABLE() : \
                                                ((__SYNC__) == SYNC2) ? SYNC_PIN2_GPIO_CLK_ENABLE() : \
                                                ((__SYNC__) == SYNC3) ? SYNC_PIN3_GPIO_CLK_ENABLE() : 0)

#define SYNC_PINn_GPIO_CLK_DISABLE(__SYNC__)   (((__SYNC__) == SYNC1) ? SYNC_PIN1_GPIO_CLK_DISABLE() : \
                                                ((__SYNC__) == SYNC2) ? SYNC_PIN2_GPIO_CLK_DISABLE() : \
                                                ((__SYNC__) == SYNC3) ? SYNC_PIN3_GPIO_CLK_DISABLE() : 0)



#define SIG_INn                          3

/**
 * @brief 
 */
#define SIG_PIN1                    GPIO_PIN_13                       /* PB13 */
#define SIG_PIN1_GPIO_PORT          GPIOB
#define SIG_PIN1_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define SIG_PIN1_GPIO_CLK_DISABLE() __GPIOB_CLK_DISABLE()
#define SIG_PIN1_EXTI_IRQn          EXTI4_15_IRQn 

#define SIG_PIN2                    GPIO_PIN_14                       /* PB14 */
#define SIG_PIN2_GPIO_PORT          GPIOB
#define SIG_PIN2_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define SIG_PIN2_GPIO_CLK_DISABLE() __GPIOB_CLK_DISABLE()
#define SIG_PIN2_EXTI_IRQn          EXTI4_15_IRQn 

#define SIG_PIN3                    GPIO_PIN_15                       /* PB15 */
#define SIG_PIN3_GPIO_PORT          GPIOB
#define SIG_PIN3_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define SIG_PIN3_GPIO_CLK_DISABLE() __GPIOB_CLK_DISABLE()
#define SIG_PIN3_EXTI_IRQn          EXTI4_15_IRQn 

#define SIG_PINn_GPIO_CLK_ENABLE(__SYNC__)   (((__SYNC__) == SIG1) ? SIG_PIN1_GPIO_CLK_ENABLE() : \
                                                ((__SYNC__) == SIG2) ? SIG_PIN2_GPIO_CLK_ENABLE() : \
                                                ((__SYNC__) == SIG3) ? SIG_PIN3_GPIO_CLK_ENABLE() : 0)

#define SIG_PINn_GPIO_CLK_DISABLE(__SYNC__)   (((__SYNC__) == SIG1) ? SIG_PIN1_GPIO_CLK_DISABLE() : \
                                                ((__SYNC__) == SIG2) ? SIG_PIN2_GPIO_CLK_DISABLE() : \
                                                ((__SYNC__) == SIG3) ? SIG_PIN3_GPIO_CLK_DISABLE() : 0)


/** @addtogroup STM324xG_EVAL_LOW_LEVEL_COM
  * @{
  */
#define COMn                                 2

/**
 * @brief 
        Definition for COM port1, connected to USART1
        Definition for COM port2, connected to USART3
        Definition for COM port3, connected to USART6
 */ 
//COM1
#define EVAL_COM1                            USART1
#define EVAL_COM1_CLK_ENABLE()               __USART1_CLK_ENABLE()
#define EVAL_COM1_CLK_DISABLE()              __USART1_CLK_DISABLE()
#define EVAL_COM_DMA_CLK_ENABLE()            __DMA1_CLK_ENABLE()  

#define EVAL_COM1_TX_PIN                     GPIO_PIN_9
#define EVAL_COM1_TX_GPIO_PORT               GPIOA
#define EVAL_COM1_TX_GPIO_CLK_ENABLE()       __GPIOA_CLK_ENABLE()
#define EVAL_COM1_TX_GPIO_CLK_DISABLE()      __GPIOA_CLK_DISABLE()
#define EVAL_COM1_TX_AF                      GPIO_AF1_USART1

#define EVAL_COM1_RX_PIN                     GPIO_PIN_10
#define EVAL_COM1_RX_GPIO_PORT               GPIOA
#define EVAL_COM1_RX_GPIO_CLK_ENABLE()       __GPIOA_CLK_ENABLE()
#define EVAL_COM1_RX_GPIO_CLK_DISABLE()      __GPIOA_CLK_DISABLE()
#define EVAL_COM1_RX_AF                      GPIO_AF1_USART1

#define EVAL_COM1_IRQn                       USART1_IRQn
#define EVAL_COM1_IRQHandler                 USART1_IRQHandler

//COM2
#define EVAL_COM2                            USART2
#define EVAL_COM2_CLK_ENABLE()               __USART2_CLK_ENABLE()
#define EVAL_COM2_CLK_DISABLE()              __USART2_CLK_DISABLE()

#define EVAL_COM2_TX_PIN                     GPIO_PIN_2
#define EVAL_COM2_TX_GPIO_PORT               GPIOA
#define EVAL_COM2_TX_GPIO_CLK_ENABLE()       __GPIOA_CLK_ENABLE()
#define EVAL_COM2_TX_GPIO_CLK_DISABLE()      __GPIOA_CLK_DISABLE()
#define EVAL_COM2_TX_AF                      GPIO_AF1_USART2

#define EVAL_COM2_RX_PIN                     GPIO_PIN_3
#define EVAL_COM2_RX_GPIO_PORT               GPIOA
#define EVAL_COM2_RX_GPIO_CLK_ENABLE()       __GPIOA_CLK_ENABLE()
#define EVAL_COM2_RX_GPIO_CLK_DISABLE()      __GPIOA_CLK_DISABLE()
#define EVAL_COM2_RX_AF                      GPIO_AF1_USART2

#define EVAL_COM2_IRQn                       USART2_IRQn
#define EVAL_COM2_IRQHandler                 USART2_IRQHandler


#define EVAL_COMx_CLK_ENABLE(__INDEX__)            (((__INDEX__) == 0) ? EVAL_COM1_CLK_ENABLE() :\
                                                    ((__INDEX__) == 1) ? EVAL_COM2_CLK_ENABLE() :0)

#define EVAL_COMx_CLK_DISABLE(__INDEX__)           (((__INDEX__) == 0) ? EVAL_COM1_CLK_DISABLE() :\
                                                    ((__INDEX__) == 1) ? EVAL_COM2_CLK_DISABLE() : 0)

#define EVAL_COMx_TX_GPIO_CLK_ENABLE(__INDEX__)    (((__INDEX__) == 0) ? EVAL_COM1_TX_GPIO_CLK_ENABLE() :\
                                                    ((__INDEX__) == 1) ? EVAL_COM2_TX_GPIO_CLK_ENABLE() :0)

#define EVAL_COMx_TX_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 0) ? EVAL_COM1_TX_GPIO_CLK_DISABLE() :\
                                                    ((__INDEX__) == 1) ? EVAL_COM2_TX_GPIO_CLK_DISABLE() :0)

#define EVAL_COMx_RX_GPIO_CLK_ENABLE(__INDEX__)    (((__INDEX__) == 0) ? EVAL_COM1_RX_GPIO_CLK_ENABLE() : \
                                                    ((__INDEX__) == 1) ? EVAL_COM2_RX_GPIO_CLK_ENABLE() :0)

#define EVAL_COMx_RX_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 0) ? EVAL_COM1_RX_GPIO_CLK_DISABLE() :\
                                                    ((__INDEX__) == 0) ? EVAL_COM2_RX_GPIO_CLK_DISABLE() : 0)  

#define EVAL_COM1_FORCE_RESET()             __USART1_FORCE_RESET()
#define EVAL_COM1_RELEASE_RESET()           __USART1_RELEASE_RESET()

#define EVAL_COM2_FORCE_RESET()             __USART2_FORCE_RESET()
#define EVAL_COM2_RELEASE_RESET()           __USART2_RELEASE_RESET()

/** @addtogroup STM32F4_DISCOVERY_LOW_LEVEL_BUS
  * @{
  */ 
/**
  * @}
  */ 
  
    
/** @defgroup STM32F0308_DISCOVERY_LOW_LEVEL_Exported_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32F0308_DISCOVERY_LOW_LEVEL_Exported_Functions
  * @{
  */
uint32_t BSP_GetVersion(void);
void     BSP_LED_Init(Led_TypeDef Led);
void     BSP_LED_On(Led_TypeDef Led);
void     BSP_LED_Off(Led_TypeDef Led);
void     BSP_LED_Toggle(Led_TypeDef Led);
void     BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Mode);
uint32_t BSP_PB_GetState(Button_TypeDef Button);
void     BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef *huart);
void     BSP_COM_Transmit(COM_TypeDef COM, UART_HandleTypeDef *huart,uint8_t *ch,uint32_t len);

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

#ifdef __cplusplus
}
#endif

#endif /* __STM32F0308_DISCOVERY_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
