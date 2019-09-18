/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "seat_heat.h"
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
_Bool	is_noAC =pdFALSE;
uint8_t clearState=0;
extern uint32_t jkListRead;
extern uint32_t jkListWrite;
uint32_t * timeStamp;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

osThreadId defaultTaskHandle;
osThreadId RCTaskHandle;
osThreadId closestoolTaskHandle;
osThreadId ctRecordHandle;
osMessageQId clearQueHandle;
osMessageQId ctrlQueHandle;
osMessageQId sCtrlQueHandle;
osMessageQId jkQueHandle;
osTimerId buttonTimerHandle;
osTimerId ledTimerHandle;
osTimerId hotwaterHandle;
osTimerId sitTimeHandle;
osTimerId hasSitHandle;
osSemaphoreId RCTigHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);
extern void StartRemoteControl(void const * argument);
extern void Startclosestool(void const * argument);
void StartCTR(void const * argument);
extern void buttonLoop(void const * argument);
extern void ledLoop(void const * argument);
extern void hotWaterLoop(void const * argument);
extern void sitLoop(void const * argument);
extern void startIsSit(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//printf("AD value = 0x%04X\r\n", 1234);

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	LL_GPIO_SetOutputPin(ZQ_JRS_GPIO_Port,ZQ_JRS_Pin);
	LL_GPIO_SetOutputPin(HG_JRS_GPIO_Port,HG_JRS_Pin);
	
	is_noAC=!LL_GPIO_IsInputPinSet(AJ_1_GPIO_Port,AJ_1_Pin);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of RCTig */
  osSemaphoreDef(RCTig);
  RCTigHandle = osSemaphoreCreate(osSemaphore(RCTig), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of buttonTimer */
  osTimerDef(buttonTimer, buttonLoop);
  buttonTimerHandle = osTimerCreate(osTimer(buttonTimer), osTimerPeriodic, NULL);

  /* definition and creation of ledTimer */
  osTimerDef(ledTimer, ledLoop);
  ledTimerHandle = osTimerCreate(osTimer(ledTimer), osTimerPeriodic, NULL);

  /* definition and creation of hotwater */
  osTimerDef(hotwater, hotWaterLoop);
  hotwaterHandle = osTimerCreate(osTimer(hotwater), osTimerPeriodic, NULL);

  /* definition and creation of sitTime */
  osTimerDef(sitTime, sitLoop);
  sitTimeHandle = osTimerCreate(osTimer(sitTime), osTimerPeriodic, NULL);

  /* definition and creation of hasSit */
  osTimerDef(hasSit, startIsSit);
  hasSitHandle = osTimerCreate(osTimer(hasSit), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of clearQue */
  osMessageQDef(clearQue, 2, uint8_t);
  clearQueHandle = osMessageCreate(osMessageQ(clearQue), NULL);

  /* definition and creation of ctrlQue */
  osMessageQDef(ctrlQue, 6, uint8_t);
  ctrlQueHandle = osMessageCreate(osMessageQ(ctrlQue), NULL);

  /* definition and creation of sCtrlQue */
  osMessageQDef(sCtrlQue, 3, uint8_t);
  sCtrlQueHandle = osMessageCreate(osMessageQ(sCtrlQue), NULL);

  /* definition and creation of jkQue */
  osMessageQDef(jkQue, 16, uint8_t);
  jkQueHandle = osMessageCreate(osMessageQ(jkQue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 100);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of RCTask */
  osThreadDef(RCTask, StartRemoteControl, osPriorityHigh, 0, 64);
  RCTaskHandle = osThreadCreate(osThread(RCTask), NULL);

  /* definition and creation of closestoolTask */
  osThreadDef(closestoolTask, Startclosestool, osPriorityNormal, 0, 64);
  closestoolTaskHandle = osThreadCreate(osThread(closestoolTask), NULL);

  /* definition and creation of ctRecord */
  osThreadDef(ctRecord, StartCTR, osPriorityLow, 0, 64);
  ctRecordHandle = osThreadCreate(osThread(ctRecord), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	if(is_noAC){
		osThreadSuspend(RCTaskHandle);
	}
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* TIM3 interrupt Init */
  NVIC_SetPriority(TIM3_IRQn, 2);
  NVIC_EnableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 23999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 299;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetOnePulseMode(TIM3, LL_TIM_ONEPULSEMODE_SINGLE);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

  /* TIM6 interrupt Init */
  NVIC_SetPriority(TIM6_IRQn, 2);
  NVIC_EnableIRQ(TIM6_IRQn);

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  TIM_InitStruct.Prescaler = 7999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 6;
  LL_TIM_Init(TIM6, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM6);
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);

  /* TIM14 interrupt Init */
  NVIC_SetPriority(TIM14_IRQn, 2);
  NVIC_EnableIRQ(TIM14_IRQn);

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  TIM_InitStruct.Prescaler = 7999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 6;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM14, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM14);
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM16);

  /* TIM16 interrupt Init */
  NVIC_SetPriority(TIM16_IRQn, 2);
  NVIC_EnableIRQ(TIM16_IRQn);

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  TIM_InitStruct.Prescaler = 24000;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM16, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM16);
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration  
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = JRS_TX_PIN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(JRS_TX_PIN_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = JRS_RX_PIN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(JRS_RX_PIN_GPIO_Port, &GPIO_InitStruct);

  /* USART1 DMA Init */
  
  /* USART1_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 3);
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 2400;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_8;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration  
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX 
  */
  GPIO_InitStruct.Pin = BT_TX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(BT_TX_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BT_RX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(BT_RX_GPIO_Port, &GPIO_InitStruct);

  /* USART2 DMA Init */
  
  /* USART2_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, 3);
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART2);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */
	LL_USART_EnableIT_RXNE(USART2);
  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 2);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel4_5_IRQn, 2);
  NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(D1_GPIO_Port, D1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(D2_GPIO_Port, D2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(D3_GPIO_Port, D3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(MCF_2_GPIO_Port, MCF_2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(MCF_1_GPIO_Port, MCF_1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(MCF_4_GPIO_Port, MCF_4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(MCF_3_GPIO_Port, MCF_3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(DCS_GPIO_Port, DCS_Pin);

  /**/
  LL_GPIO_ResetOutputPin(HG_JRS_GPIO_Port, HG_JRS_Pin);

  /**/
  LL_GPIO_ResetOutputPin(ZQ_JRS_GPIO_Port, ZQ_JRS_Pin);

  /**/
  LL_GPIO_ResetOutputPin(QB_GPIO_Port, QB_Pin);

  /**/
  LL_GPIO_ResetOutputPin(DCF_GPIO_Port, DCF_Pin);

  /**/
  LL_GPIO_ResetOutputPin(CC_GPIO_Port, CC_Pin);

  /**/
  LL_GPIO_ResetOutputPin(HG_FAN_GPIO_Port, HG_FAN_Pin);

  /**/
  LL_GPIO_ResetOutputPin(ZM_GPIO_Port, ZM_Pin);

  /**/
  LL_GPIO_ResetOutputPin(FMQ_GPIO_Port, FMQ_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BJJ_8_GPIO_Port, BJJ_8_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BJJ_7_GPIO_Port, BJJ_7_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BJJ_6_GPIO_Port, BJJ_6_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BJJ_5_GPIO_Port, BJJ_5_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BJJ_4_GPIO_Port, BJJ_4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BJJ_3_GPIO_Port, BJJ_3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BJJ_2_GPIO_Port, BJJ_2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BJJ_1_GPIO_Port, BJJ_1_Pin);

  /**/
  GPIO_InitStruct.Pin = AJ_1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(AJ_1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = AJ_2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(AJ_2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = AJ_3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(AJ_3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = AJ_4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(AJ_4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = D1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(D1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = D2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(D2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = D3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(D3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = RZCGQ_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(RZCGQ_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = XBJT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(XBJT_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MCF_2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(MCF_2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MCF_1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(MCF_1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MCF_4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(MCF_4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MCF_3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(MCF_3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DCS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(DCS_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = HG_JRS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(HG_JRS_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ZQ_JRS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(ZQ_JRS_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = QB_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(QB_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DCF_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(DCF_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = CC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(CC_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = HG_FAN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(HG_FAN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ZM_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(ZM_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = FMQ_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(FMQ_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BJJ_8_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BJJ_8_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BJJ_7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BJJ_7_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BJJ_6_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BJJ_6_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BJJ_5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BJJ_5_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BJJ_4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BJJ_4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BJJ_3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BJJ_3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BJJ_2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BJJ_2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BJJ_1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BJJ_1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    
    
    
    
    
    

  /* USER CODE BEGIN 5 */
	/*
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,0x800fc40,0x12345668);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,0x800fc44,0x5678);
	HAL_FLASH_Lock();
	*/
	
	//如果是停电,那么起来就直接冲水,然后结束
	if(is_noAC){
		osMessagePut(clearQueHandle,0x56,0);
		
	}else{
		//读取设置
		osMessageGet(clearQueHandle,osWaitForever);
		osTimerStart(hasSitHandle,1);
		//启动各种时钟
		osTimerStart(buttonTimerHandle,1);
	}
  /* Infinite loop */
  for(;;)
  {
		osEvent rec=	osMessageGet(clearQueHandle,osWaitForever);
		if(rec.status==osEventMessage){
			clearState=rec.value.v;
			LL_GPIO_SetOutputPin(DCS_GPIO_Port,DCS_Pin);
			switch(rec.value.v){
				case 0x34:	//润壁
					LL_GPIO_SetOutputPin(MCF_1_GPIO_Port,MCF_1_Pin);
					osDelay(MCF_PULE_MS);
					LL_GPIO_ResetOutputPin(MCF_1_GPIO_Port,MCF_1_Pin);
					osDelay(MCF_RB_MS);
					LL_GPIO_SetOutputPin(MCF_2_GPIO_Port,MCF_2_Pin);
					osDelay(MCF_PULE_MS);
					LL_GPIO_ResetOutputPin(MCF_2_GPIO_Port,MCF_2_Pin);
				
					break;
				case 0x45:	//小冲
					LL_GPIO_SetOutputPin(MCF_1_GPIO_Port,MCF_1_Pin);
					osDelay(MCF_PULE_MS);
					LL_GPIO_ResetOutputPin(MCF_1_GPIO_Port,MCF_1_Pin);
					osDelay(MCF_XC_U1_MS);
					LL_GPIO_SetOutputPin(MCF_2_GPIO_Port,MCF_2_Pin);
					osDelay(MCF_PULE_MS);
					LL_GPIO_ResetOutputPin(MCF_2_GPIO_Port,MCF_2_Pin);
					//下冲
					LL_GPIO_SetOutputPin(MCF_3_GPIO_Port,MCF_3_Pin);
					osDelay(MCF_PULE_MS);
					LL_GPIO_ResetOutputPin(MCF_3_GPIO_Port,MCF_3_Pin);
					osDelay(MCF_XC_D2_MS);
					LL_GPIO_SetOutputPin(MCF_4_GPIO_Port,MCF_4_Pin);
					osDelay(MCF_PULE_MS);
					LL_GPIO_ResetOutputPin(MCF_4_GPIO_Port,MCF_4_Pin);
					//上冲
					LL_GPIO_SetOutputPin(MCF_1_GPIO_Port,MCF_1_Pin);
					osDelay(MCF_PULE_MS);
					LL_GPIO_ResetOutputPin(MCF_1_GPIO_Port,MCF_1_Pin);
					osDelay(MCF_XC_U3_MS);
					LL_GPIO_SetOutputPin(MCF_2_GPIO_Port,MCF_2_Pin);
					osDelay(MCF_PULE_MS);
					LL_GPIO_ResetOutputPin(MCF_2_GPIO_Port,MCF_2_Pin);

					break;
				case 0x56:
					LL_GPIO_SetOutputPin(MCF_1_GPIO_Port,MCF_1_Pin);
					osDelay(MCF_PULE_MS);
					LL_GPIO_ResetOutputPin(MCF_1_GPIO_Port,MCF_1_Pin);
					osDelay(MCF_DC_U1_MS);
					LL_GPIO_SetOutputPin(MCF_2_GPIO_Port,MCF_2_Pin);
					osDelay(MCF_PULE_MS);
					LL_GPIO_ResetOutputPin(MCF_2_GPIO_Port,MCF_2_Pin);
					//下冲
					LL_GPIO_SetOutputPin(MCF_3_GPIO_Port,MCF_3_Pin);
					osDelay(MCF_PULE_MS);
					LL_GPIO_ResetOutputPin(MCF_3_GPIO_Port,MCF_3_Pin);
					osDelay(MCF_DC_D2_MS);
					LL_GPIO_SetOutputPin(MCF_4_GPIO_Port,MCF_4_Pin);
					osDelay(MCF_PULE_MS);
					LL_GPIO_ResetOutputPin(MCF_4_GPIO_Port,MCF_4_Pin);
				//osDelay(300);
					//上冲
					LL_GPIO_SetOutputPin(MCF_1_GPIO_Port,MCF_1_Pin);
					osDelay(MCF_PULE_MS);
					LL_GPIO_ResetOutputPin(MCF_1_GPIO_Port,MCF_1_Pin);
					osDelay(MCF_DC_U3_MS);
					LL_GPIO_SetOutputPin(MCF_2_GPIO_Port,MCF_2_Pin);
					osDelay(MCF_PULE_MS);
					LL_GPIO_ResetOutputPin(MCF_2_GPIO_Port,MCF_2_Pin);
					
					break;
			}
						LL_GPIO_ResetOutputPin(DCS_GPIO_Port,DCS_Pin);
			clearState=0;
		}
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartCTR */
/* USER CODE END Header_StartCTR */
void StartCTR(void const * argument)
{
  /* USER CODE BEGIN StartCTR */
	uint32_t toWriteData[8]={0,0,0,0,0,0,0,0};
	uint32_t * beginTick=&toWriteData[2], * endTick=&toWriteData[5],*doTick= &toWriteData[3],*hgTick=&toWriteData[4];
	uint32_t * doType=&toWriteData[1];
	timeStamp=&toWriteData[6];
	
	jkListRead =  *(__IO uint32_t *)JK_READ_NUM;
	jkListWrite=*(__IO uint32_t *)JK_WRITE_NUM;
	while(1){
		osEvent oet= osMessageGet(jkQueHandle,osWaitForever);
		if(oet.status== osEventMessage){
			switch(oet.value.v){
				case 0:
					*beginTick=osKernelSysTick();
				*endTick=0;
				*doTick=0;
				*hgTick=0;
				*doType=0;
					break;
				case 2:	//女洗
					*doTick=osKernelSysTick();
				*doType=2;
				break;
				case 3:
					*doTick=osKernelSysTick();
				*doType=3;
				break;
				case 4:
					*hgTick=osKernelSysTick();
				break;
				case 0xff:
					//写入记录
				*endTick=osKernelSysTick();
				if(*doType==0 && *hgTick==0) break;	//只是坐坐不记录
				if((jkListWrite<jkListRead && (jkListRead/32 == jkListWrite/32))|| (jkListRead==0 && jkListWrite==JK_MAX_NUM-1)){
					//也满了,因为部分数据有用,不能擦写掉
					//数据满了
					break;
				}
				//如果刚好是一页的开始,擦掉这一页数据
				if(jkListWrite%32==0){
					HAL_FLASH_Unlock();
					FLASH_EraseInitTypeDef fei;
					fei.PageAddress=JK_LIST_BEGIN_ADD+1024*(jkListWrite/32);
					fei.NbPages=1;
					fei.TypeErase=FLASH_TYPEERASE_PAGES;
					uint32_t err=0;
					HAL_FLASHEx_Erase(&fei,&err);
					//HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,0x800fc40,0x12345668);
					//HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,0x800fc44,0x5678);
					//HAL_FLASH_Lock();
				}else{
					HAL_FLASH_Unlock();
				}
				//开始写入
				//toWriteData[6]=tim
				for(int i=0;i<8;i++){
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,JK_LIST_BEGIN_ADD+32*jkListWrite+4*i,toWriteData[i]);
				}
				HAL_FLASH_Lock();
				if(++jkListWrite==JK_MAX_NUM) jkListWrite=0;
					break;
			}
		}
	}
  /* USER CODE END StartCTR */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
