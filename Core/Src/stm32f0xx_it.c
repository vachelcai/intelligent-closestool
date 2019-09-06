/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "stepper.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern osSemaphoreId RCTigHandle;
 uint8_t remoteCtrlData[10];
 uint8_t remoteCtrlPos=0;

//蜂鸣器部分
extern bool _fmqBusy;
 
 //步进机部分
extern stepperStr _pj;
extern stepperStr _fsf;
extern osMessageQId sCtrlQueHandle;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t usart1_last=0;
uint8_t usart1RxPos=0;
extern uint8_t EIWHReceiveData[8];
extern _Bool hasReceive;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim17;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	if(LL_TIM_IsActiveFlag_UPDATE(TIM3)){
		LL_TIM_ClearFlag_UPDATE(TIM3);
		LL_GPIO_ResetOutputPin(FMQ_GPIO_Port,FMQ_Pin);
		_fmqBusy=false;
	}
  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */
	if(LL_TIM_IsActiveFlag_UPDATE(TIM6)){
		LL_TIM_ClearFlag_UPDATE(TIM6);
		if(PJStep()){
			LL_TIM_DisableCounter(TIM6);
			
		}
	}
  /* USER CODE END TIM6_IRQn 0 */
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */
	if(LL_TIM_IsActiveFlag_UPDATE(TIM14)){
		LL_TIM_ClearFlag_UPDATE(TIM14);
		if(FSFStep()){
			LL_TIM_DisableCounter(TIM14);
		}
	}
  /* USER CODE END TIM14_IRQn 0 */
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */
		if(LL_TIM_IsActiveFlag_UPDATE(TIM16)){
			LL_TIM_ClearFlag_UPDATE(TIM16);
			osMessagePut(sCtrlQueHandle,0x66,0);
			LL_TIM_EnableCounter(TIM16);
		}
  /* USER CODE END TIM16_IRQn 0 */
  /* USER CODE BEGIN TIM16_IRQn 1 */

  /* USER CODE END TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM17 global interrupt.
  */
void TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM17_IRQn 0 */

  /* USER CODE END TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM17_IRQn 1 */

  /* USER CODE END TIM17_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
		if(LL_USART_IsActiveFlag_TC(USART1)){	//发送dma
        LL_USART_DisableIT_TC(USART1);
        LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_2);

    }
		if(LL_USART_IsActiveFlag_RXNE(USART1)){
			if(osKernelSysTick()-usart1_last>1000) usart1RxPos=0;
			EIWHReceiveData[usart1RxPos++]=LL_USART_ReceiveData8(USART1);
			if(usart1RxPos==8){
				hasReceive=1;
			}
		}
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	//if(huart2.Instance->
	if((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE){
				//USART2->ISR=~USART_ISR_RXNE;
        remoteCtrlData[remoteCtrlPos]= LL_USART_ReceiveData8(USART2);
        if(remoteCtrlPos==0 && remoteCtrlData[0]!=0x00){
            return;
        }
        if(remoteCtrlPos==9) {
            if(remoteCtrlData[9]!=0x00){
                remoteCtrlPos=0;
                return;
            }else{
                remoteCtrlPos=0;
                osSemaphoreRelease(RCTigHandle);
                return;
            }
        }
        remoteCtrlPos++;
        return;
    }
		
		if(LL_USART_IsActiveFlag_TC(USART2)){
        LL_USART_DisableIT_TC(USART2);
        LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_4);

    }
  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
