/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user_config.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AJ_1_Pin LL_GPIO_PIN_13
#define AJ_1_GPIO_Port GPIOC
#define AJ_2_Pin LL_GPIO_PIN_14
#define AJ_2_GPIO_Port GPIOC
#define AJ_3_Pin LL_GPIO_PIN_15
#define AJ_3_GPIO_Port GPIOC
#define AJ_4_Pin LL_GPIO_PIN_0
#define AJ_4_GPIO_Port GPIOF
#define ZW_PIN_Pin LL_GPIO_PIN_0
#define ZW_PIN_GPIO_Port GPIOA
#define BT_TX_Pin LL_GPIO_PIN_2
#define BT_TX_GPIO_Port GPIOA
#define BT_RX_Pin LL_GPIO_PIN_3
#define BT_RX_GPIO_Port GPIOA
#define D1_Pin LL_GPIO_PIN_4
#define D1_GPIO_Port GPIOA
#define D2_Pin LL_GPIO_PIN_5
#define D2_GPIO_Port GPIOA
#define D3_Pin LL_GPIO_PIN_6
#define D3_GPIO_Port GPIOA
#define RZCGQ_Pin LL_GPIO_PIN_7
#define RZCGQ_GPIO_Port GPIOA
#define XBJT_Pin LL_GPIO_PIN_0
#define XBJT_GPIO_Port GPIOB
#define MCF_2_Pin LL_GPIO_PIN_1
#define MCF_2_GPIO_Port GPIOB
#define MCF_1_Pin LL_GPIO_PIN_2
#define MCF_1_GPIO_Port GPIOB
#define MCF_4_Pin LL_GPIO_PIN_10
#define MCF_4_GPIO_Port GPIOB
#define MCF_3_Pin LL_GPIO_PIN_11
#define MCF_3_GPIO_Port GPIOB
#define DCS_Pin LL_GPIO_PIN_12
#define DCS_GPIO_Port GPIOB
#define HG_JRS_Pin LL_GPIO_PIN_13
#define HG_JRS_GPIO_Port GPIOB
#define ZQ_JRS_Pin LL_GPIO_PIN_14
#define ZQ_JRS_GPIO_Port GPIOB
#define QB_Pin LL_GPIO_PIN_15
#define QB_GPIO_Port GPIOB
#define DCF_Pin LL_GPIO_PIN_8
#define DCF_GPIO_Port GPIOA
#define JRS_TX_PIN_Pin LL_GPIO_PIN_9
#define JRS_TX_PIN_GPIO_Port GPIOA
#define JRS_RX_PIN_Pin LL_GPIO_PIN_10
#define JRS_RX_PIN_GPIO_Port GPIOA
#define CC_Pin LL_GPIO_PIN_11
#define CC_GPIO_Port GPIOA
#define HG_FAN_Pin LL_GPIO_PIN_12
#define HG_FAN_GPIO_Port GPIOA
#define ZM_Pin LL_GPIO_PIN_6
#define ZM_GPIO_Port GPIOF
#define FMQ_Pin LL_GPIO_PIN_7
#define FMQ_GPIO_Port GPIOF
#define BJJ_8_Pin LL_GPIO_PIN_15
#define BJJ_8_GPIO_Port GPIOA
#define BJJ_7_Pin LL_GPIO_PIN_3
#define BJJ_7_GPIO_Port GPIOB
#define BJJ_6_Pin LL_GPIO_PIN_4
#define BJJ_6_GPIO_Port GPIOB
#define BJJ_5_Pin LL_GPIO_PIN_5
#define BJJ_5_GPIO_Port GPIOB
#define BJJ_4_Pin LL_GPIO_PIN_6
#define BJJ_4_GPIO_Port GPIOB
#define BJJ_3_Pin LL_GPIO_PIN_7
#define BJJ_3_GPIO_Port GPIOB
#define BJJ_2_Pin LL_GPIO_PIN_8
#define BJJ_2_GPIO_Port GPIOB
#define BJJ_1_Pin LL_GPIO_PIN_9
#define BJJ_1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
