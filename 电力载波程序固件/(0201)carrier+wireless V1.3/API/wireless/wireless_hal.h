#ifndef WIRELESS_HAL_H
#define WIRELESS_HAL_H

#include "stm32f0xx.h"
//#include "wireless_app.h"




/********************** 接口宏定义 ***************************/
#ifdef USE_DEMO
#define WIRELESS_PORT_CLOCK   1<<17       //I/O port A clock

#define SDN_GPIOX             GPIOA       //SDN->PA0
#define SDN_PIN               0

#define NIRQ_GPIOX            GPIOA       //NIRQ->PA1
#define NIRQ_PIN              1
#define NIRQ_EXTIMAP          0           //外部中断映射，PA口为0，PB口为1，PC口为2 ...
#define NIRQ_IMR_CHANNEL     EXTI0_1_IRQn    //Interrupt Numbers
#define NIRQ_IMR_PRIORITY         0x02        //Interrupt priority

#define NIRQ_EMR_NUMBER       3               //Event request Numbers
#define NIRQ_EMR_CHANNEL     EXTI2_3_IRQn    //Interrupt Numbers
#define NIRQ_EMR_PRIORITY         0x03        //Interrupt priority

#define NSEL_GPIOX            GPIOA       //NSEL->PA8
#define NSEL_PIN              8

#define SCK_AFSEL             0           //SPI1_SCK
#define SCK_GPIOX             GPIOA       //SCK->PA5
#define SCK_PIN               5

#define MOSI_AFSEL            0           //SPI1_MOSI
#define MOSI_GPIOX            GPIOA       //MOSI->PA7
#define MOSI_PIN              7

#define MISO_AFSEL            0           //SPI1_MISO
#define MISO_GPIOX            GPIOA       //MISO->PA6
#define MISO_PIN              6

#define WIRELESS_SPI          SPI1        //Select SPI
#define WIRELESS_SPI_CLKEN    (RCC->APB2ENR |= 1<<12)       //SPI clock enable

#else     //////////////////////////////////////////////////
#define WIRELESS_PORT_CLOCK   (1<<17 | 1<<18)       //I/O port A and B clock

#define SDN_GPIOX             GPIOB       //SDN->PB7
#define SDN_PIN               7

#define NIRQ_GPIOX            GPIOB       //NIRQ->PB6
#define NIRQ_PIN              6
#define NIRQ_EXTIMAP          1           //外部中断映射，PA口为0，PB口为1，PC口为2 ...
#define NIRQ_IMR_CHANNEL     EXTI4_15_IRQn    //Interrupt Numbers
#define NIRQ_IMR_PRIORITY         0x02        //Interrupt priority

#define NIRQ_EMR_NUMBER       3               //Event request Numbers
#define NIRQ_EMR_CHANNEL     EXTI2_3_IRQn    //Interrupt Numbers
#define NIRQ_EMR_PRIORITY         0x03        //Interrupt priority

#define NSEL_GPIOX            GPIOA       //NSEL->PA15
#define NSEL_PIN              15

#define SCK_AFSEL             0           //SPI1_SCK
#define SCK_GPIOX             GPIOB       //SCK->PB3
#define SCK_PIN               3

#define MOSI_AFSEL            0           //SPI1_MOSI
#define MOSI_GPIOX            GPIOB       //MOSI->PB5
#define MOSI_PIN              5

#define MISO_AFSEL            0           //SPI1_MISO
#define MISO_GPIOX            GPIOB       //MISO->PB4
#define MISO_PIN              4

#define WIRELESS_SPI          SPI1
#define WIRELESS_SPI_CLKEN    (RCC->APB2ENR |= 1<<12)       //SPI clock enable

#endif
/********************** 宏定义结束 ***************************/






// #ifdef USE_DEMO
// #define NSEL_H 	(GPIOA->BSRR = 1<<8)
// #define NSEL_L	(GPIOA->BRR = 1<<8)

// #define SDN_H 	(GPIOA->BSRR = 1<<0)
// #define SDN_L		(GPIOA->BRR = 1<<0)

// #define GPIO1_R (GPIOB->IDR & (1<<1))


// #else
// #define DIS_NIRQ_INT    (EXTI->EMR &= ~(1<<6))
// #define EN_NIRQ_INT     (EXTI->EMR |= 1<<6)

// #define NSEL_H 	{GPIOA->BSRR = 1<<15; EN_NIRQ_INT;}
// #define NSEL_L	{DIS_NIRQ_INT; GPIOA->BRR = 1<<15;}
// #define SDN_H 	(GPIOB->BSRR = 1<<7)
// #define SDN_L		(GPIOB->BRR = 1<<7)



// #endif


typedef struct
{
  uint8_t NVIC_IRQChannel;             /*!< Specifies the IRQ channel to be enabled or disabled.
                                            This parameter can be a value of @ref IRQn_Type 
                                            (For the complete STM32 Devices IRQ Channels list, 
                                            please refer to stm32f0xx.h file) */

  uint8_t NVIC_IRQChannelPriority;     /*!< Specifies the priority level for the IRQ channel specified
                                            in NVIC_IRQChannel. This parameter can be a value
                                            between 0 and 3.  */

  FunctionalState NVIC_IRQChannelCmd;  /*!< Specifies whether the IRQ channel defined in NVIC_IRQChannel
                                            will be enabled or disabled. 
                                            This parameter can be set either to ENABLE or DISABLE */   
} NVIC_InitTypeDef;

void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);






void Si4438Gpio_Init(void);
void NIRQ_IntEN(void);
void SpiGpio_Init(void);
void SPI_Init(void);
//_Pragma("optimize=none") uint8_t SPI_RWbyte(uint8_t sdata);
uint8_t SPI_RWbyte(uint8_t sdata);

void Si4438_Delay_ms(uint16_t nms);







#endif


