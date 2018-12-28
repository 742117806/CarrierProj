/**
  ******************************************************************************
  * @file    Demonstrations/Src/stm32f0xx_it.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    05-Dec-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f030_peripheral.h"
#include "sample.h"
#include "stm32f0xx_it.h"

void NMI_Handler( void )
{
}

void HardFault_Handler( void )
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while( 1 )
    {
    }
}

void SVC_Handler( void )
{
}

void PendSV_Handler( void )
{
}

void SysTick_Handler( void )
{
    HAL_IncTick();
    SysTickCallback();
}

void PHASE_TIME_IRQHandler( void )
{
    //HAL_TIM_IRQHandler(&TimHandle);
    if( PHASE_TIME->DIER & TIM_FLAG_UPDATE )
    {
        PHASE_TIME->SR = ~TIM_IT_UPDATE;
        HAL_ADC_Start( &AdcHandle );
        PhaseTimeCallBack();
    }
}

void SAMPE_TIME_IRQHandler( void )
{
    if( SAMPE_TIME->DIER & TIM_FLAG_UPDATE )
    {
        SAMPE_TIME->SR = ~TIM_IT_UPDATE;
        SampleTimeCallBack();
    }
}

void EXTI0_1_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( USER_BUTTON_PIN );
}

void EXTI4_15_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( SYNC_PIN1 );
    HAL_GPIO_EXTI_IRQHandler( SYNC_PIN2 );
    HAL_GPIO_EXTI_IRQHandler( SYNC_PIN3 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_6 );
}

void USART1_IRQHandler( void )
{
    HAL_UART_IRQHandler( &UartHandleUp );
    USER_UART_Hand( &UartHandleUp );
}

void USART2_IRQHandler( void )
{
    HAL_UART_IRQHandler( &UartHandleDown );
    //USER_UART_Hand(&UartHandleDown);
    /* USER CODE BEGIN USART2_IRQn 0 */
    uint32_t timeout = 0;
    extern uint8_t gDownRxBuffer[4];
    /* USER CODE END USART2_IRQn 0 */
    HAL_UART_IRQHandler( &UartHandleDown );
    /* USER CODE BEGIN USART2_IRQn 1 */
    timeout = 0;
    while( HAL_UART_GetState( &UartHandleDown ) != HAL_UART_STATE_READY ) //等待就绪
    {
        timeout++; ////超时处理
        if( timeout > ( HAL_MAX_DELAY - 1 ) )
        {
            break;
        }
    }
    timeout = 0;
    while( HAL_UART_Receive_IT( &UartHandleDown, &gDownRxBuffer[0], 1 ) != HAL_OK ) //一次处理完成之后，重新开启中断并设置RxXferCount为1
    {
        timeout++; //超时处理
        if( timeout > ( HAL_MAX_DELAY - 1 ) )
        {
            break;
        }
    }
    /* USER CODE END USART2_IRQn 1 */
}

void DMA1_Channel1_IRQHandler( void )
{
    HAL_DMA_IRQHandler( AdcHandle.DMA_Handle );
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
