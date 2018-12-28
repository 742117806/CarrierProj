#ifndef __STM32F030_PER_H
#define __STM32F030_PER_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Definition for SAMPE_TIME clock resources */
#define SAMPE_TIME                           TIM6
#define SAMPE_TIME_CLK_ENABLE()              __TIM6_CLK_ENABLE()

/* Definition for SAMPE_TIME's NVIC */
#define SAMPE_TIME_IRQn                      TIM6_IRQn
#define SAMPE_TIME_IRQHandler                      TIM6_IRQHandler
    
/* Definition for SAMPE_TIME clock resources */
#define PHASE_TIME                           TIM15
#define PHASE_TIME_CLK_ENABLE()              __TIM15_CLK_ENABLE()

/* Definition for PHASE's NVIC */
#define PHASE_TIME_IRQn                      TIM15_IRQn
#define PHASE_TIME_IRQHandler                TIM15_IRQHandler



/**
179.10K  268
178.40K  269
177.77K  270
177.12K  271
176.45K  272
175.82K  273
175.18K  274
174.54K  275

166.0K  289
165.50K  290
164.94K  291
164.38K  292
163.80K  293
163.25K  294
162.70K  295
162.15K ,296,
161.60K ,297
161.07K, 298
160.53K  299
160,00K  300
*/
 #define TARR_LOAD164 (uint32_t)((48000000/164300)-1)    //169.3- 166   6.2K 
#define TCCR_LOAD164 (uint32_t)(48000000/164300/2)
 #define TARR_LOAD175 (uint32_t)((48000000/176400)-1)  
 #define TCCR_LOAD175 (uint32_t)(48000000/176400/2)// 169.3 - 171 = 6.1K  
 #define LOCAL_FREQ (uint32_t)((48000000/285700)-1)
// #define LOCAL_FREQ (uint32_t)((48000000/283700)-1)

// #define TARR_LOAD164 (uint32_t)((48000000/334600)-1)   // 340.7 - 334.6 = 6.1K  
// #define TCCR_LOAD164 (uint32_t)(48000000/334600/2)
// #define TARR_LOAD175 (uint32_t)((48000000/346800)-1)   //340.7- 346.8   6.1K 
// #define TCCR_LOAD175 (uint32_t)(48000000/346800/2)
// #define LOCAL_FREQ (uint32_t)((48000000/114300)-1)


//#define TARR_LOAD164 (uint32_t)((48000000/234600)-1)   // 240.7 - 234.6 = 6.1K  //中心频点+6.1K得到低频
//#define TCCR_LOAD164 (uint32_t)(48000000/234600/2)
//#define TARR_LOAD175 (uint32_t)((48000000/246800)-1)   //240.7- 246.8   6.1K    //中心频点+6.1K得到高频
//#define TCCR_LOAD175 (uint32_t)(48000000/246800/2)
//#define LOCAL_FREQ (uint32_t)((48000000/214300)-1)       //理论值 滤波器（板子上面的元器件）的值减去中心频点（455000-240700）
//#define LOCAL_FREQ (uint32_t)((48000000/216000)-1)      //有误差得出的值


#define TEST_SIG_PIN1  HAL_GPIO_ReadPin(GPIOB,SIG_PIN1)

#define ADDR_FLASH_PAGE_63    ((uint32_t)0x0800FC00) /* Base @ of Page 63, 1 Kbyte */
#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_63   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_63 + FLASH_PAGE_SIZE   /* End @ of user Flash area */


#define MAXDOWNBUF      256
#define MAXUPBUF        256

typedef struct struct_com_trans{
  uint8_t is_flag;//
  uint8_t head;  
  uint8_t addr[6];//地址字节。
  uint16_t len; 
  uint8_t len_sub;
  uint8_t cmd;
  uint8_t *p_buf;//字节接收后的缓存
  uint8_t end_char;
  uint8_t sum;
  uint8_t Over_time_count;
  uint8_t rx_step;
}StructTypedefUartFrame;
extern CRC_HandleTypeDef       CrcHandle;
extern StructTypedefUartFrame  STDownUartFrame,STUpUartFrame;
extern UART_HandleTypeDef UartHandleUp,UartHandleDown;
extern ADC_HandleTypeDef             AdcHandle;

void Crc_Init(void);

/*sigin  IO*/
void SigInConfig(void);
void SigInADCIOCofnig(void);
/*Interrupt*/
void EXTI4_15_IRQHandler_Config(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/*Control pin */
void InitControlPins(void);
void SelectControlPinChannal(unsigned char);
/*phase_TIME  TIM15*/
void InitPhaseTime(void);
void DisablePhaseTime(void);
void EnablePhaseTime(void);
/*SAMPE_TIME  TIM6*/
void InitSampeTime(void);
void DisableSampeTime(void);
void EnableSampeTime(void);
void EnableSampeTime_Ex(void);
/*38KHz time TIM1*/
void Init38KhzOutTime(void);
void Disable38KhzOutTime(void);
void Enable38KhzOutTime(void);
/*PWM  TIM14*/
void InitPWMTime(void);
void DisablePWMOut(void);
void EnablePWMOut(void);
/*phase A  TIM16*/
void InitFskOutATime(void);
void DisableFskOutATime(void);
void EnableFskOutATime(void);
void FskOutAHighBitFreq(void);
void FskOutALowBitFreq(void);
/*phase B TIM17*/
void InitFskOutBTime(void);
void DisableFskOutBTime(void);
void EnableFskOutBTime(void);
void FskOutBHighBitFreq(void);
void FskOutBLowBitFreq(void);
/*phase C  TIM3*/
void InitFskOutCTime(void);
void DisableFskOutCTime(void);
void EnableFskOutCTime(void);
void FskOutCHighBitFreq(void);
void FskOutCLowBitFreq(void);
/*Flash ReadWrite*/
void FLASHWrite(uint32_t offset,uint32_t *buf,uint16_t len);
void FLASHRead(uint32_t  offset,uint32_t *buf,uint16_t len);
/*USART*/
void USARTx_Init(void);
void USER_UART_Hand(UART_HandleTypeDef* UartHandle);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle);
static void LowDownUartProcess(void);
static void LowUpUartProcess(void);

#endif