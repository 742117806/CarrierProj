/**
  ******************************************************************************
  * @file    Demonstrations/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    05-Dec-2014
  * @brief   Header for main.c module
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0308_plc.h"
//#include "wireless_app.h"
/* Define --------------------------------------------------------------------*/
#define LEDG_Pin GPIO_PIN_1
#define LEDG_GPIO_Port GPIOA
#define LEDR_Pin GPIO_PIN_5
#define LEDR_GPIO_Port GPIOA
#define SI4438_SDN_Pin GPIO_PIN_7
#define SI4438_SDN_GPIO_Port GPIOB
#define SI4438_nIRQ_Pin GPIO_PIN_6
#define SI4438_nIRQ_GPIO_Port GPIOB
#define SI4438_nIRQ_EXTI_IRQn EXTI4_15_IRQn
#define SI4438_NSS_Pin GPIO_PIN_15
#define SI4438_NSS_GPIO_Port GPIOA

#define LEDR_ON()  	HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);
#define LEDR_OFF() 	HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
#define LEDR_TOGGLE() 	HAL_GPIO_TogglePin(LEDR_GPIO_Port, LEDR_Pin);
#define LEDG_ON()  	HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
#define LEDG_OFF() 	HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
#define LEDG_TOGGLE() 	HAL_GPIO_TogglePin(LEDG_GPIO_Port, LEDG_Pin);


#define WIRELESS_NSEL_L()    (HAL_GPIO_WritePin(SI4438_NSS_GPIO_Port, SI4438_NSS_Pin, GPIO_PIN_RESET))
#define WIRELESS_NSEL_H()    (HAL_GPIO_WritePin(SI4438_NSS_GPIO_Port, SI4438_NSS_Pin, GPIO_PIN_SET))
#define SDN_L()              (HAL_GPIO_WritePin(SI4438_SDN_GPIO_Port, SI4438_SDN_Pin, GPIO_PIN_RESET))
#define SDN_H()              (HAL_GPIO_WritePin(SI4438_SDN_GPIO_Port, SI4438_SDN_Pin, GPIO_PIN_SET))


#define WIRELESS_CHANNEL        20        //定义使用的无线通道，范围2-65    
extern IWDG_HandleTypeDef      IwdgHandle;
//extern WLS WIRELESS_DATA;
extern volatile uint8_t WIRELESS_LED_FLAG;

void SysTickCallback (void);
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 

