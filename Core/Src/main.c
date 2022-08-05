/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gps.h"
#include "FreeRTOS.h"
#include "task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/*Messages Structs*/
send_longitude long_send;
receive_longitude long_receive;
send_lattitude lat_send;
receive_lattitude lat_receive;
send_speed speed_send;
receive_speed speed_receive;
/************************************************************/
/*Global Variables to test the execution time of GPS Task*/
uint16_t GPS_start=0;
uint16_t GPS_end=0;
uint16_t GPS_execution_time=0;
uint8_t GPS_time[100];
/*Global Variables to test the execution time of CAN Task*/
uint16_t CAN_start = 0;
uint16_t CAN_end = 0;
uint16_t CAN_execution_time = 0;
uint8_t CAN_time[100];
uint8_t new_line [2] = "\r\n";
/************************************************************/
UART_HandleTypeDef GPS_UART_HANDLER;
UART_HandleTypeDef DEBUG_UART_HANDLER;
I2C_HandleTypeDef I2C_HANDLER;
/*CAN Messages Handlers*/
CAN_RxHeaderTypeDef longitudeRx;
CAN_TxHeaderTypeDef longitudeTx;
CAN_RxHeaderTypeDef lattitudeRx;
CAN_TxHeaderTypeDef lattitudeTx;
CAN_RxHeaderTypeDef speedRx;
CAN_TxHeaderTypeDef speedTx;
CAN_FilterTypeDef filter;
uint32_t mailbox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void CAN_Config_filter()
{
	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterMaskIdLow = 0;
	filter.FilterMaskIdHigh = 0;
	filter.FilterIdHigh = 0;
	filter.FilterIdLow = 0;
	filter.FilterBank = 0;
	HAL_CAN_ConfigFilter( &hcan , &filter );
}
void Longitude_Tx(void)
{
		longitudeTx.StdId = 0x10;
		longitudeTx.ExtId = 0;
		longitudeTx.DLC = 8;
		longitudeTx.IDE = CAN_ID_STD;
		longitudeTx.RTR = CAN_RTR_DATA;
		longitudeTx.TransmitGlobalTime = DISABLE;
		HAL_CAN_AddTxMessage( &hcan , &longitudeTx , long_send.send_longitude_arr , &mailbox );
		/*DeComment if using Polling*/
		//while(HAL_CAN_IsTxMessagePending(&hcan,mailbox));
}
void Longitude_Rx()
{		
	/*DeComment if using Polling*/
	//while( ! ( HAL_CAN_GetRxFifoFillLevel(&hcan,CAN_RX_FIFO0)));		
	HAL_CAN_GetRxMessage( &hcan,CAN_RX_FIFO0 ,&longitudeRx ,long_receive.receive_longitude_arr );
	HAL_GPIO_TogglePin( GPIOC, GPIO_PIN_13 );

}
void Lattitude_Tx(void)
{
	lattitudeTx.StdId = 0x11;
	lattitudeTx.ExtId = 0;
	lattitudeTx.DLC = 8;
	lattitudeTx.IDE = CAN_ID_STD;
	lattitudeTx.RTR = CAN_RTR_DATA;
	lattitudeTx.TransmitGlobalTime = DISABLE;
	HAL_CAN_AddTxMessage( &hcan , &lattitudeTx ,(lat_send.send_lattitude_arr) , &mailbox );
	/*DeComment if using Polling*/
	//while(HAL_CAN_IsTxMessagePending(&hcan,mailbox));
}
void Lattitude_Rx()
{		
	/*DeComment if using Polling*/
	//while( ! ( HAL_CAN_GetRxFifoFillLevel(&hcan,CAN_RX_FIFO0)));		
	HAL_CAN_GetRxMessage( &hcan ,CAN_RX_FIFO0 ,&lattitudeRx ,lat_receive.receive_lattitude_arr );
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

}
void Speed_Tx(void)
{
	speedTx.StdId = 0x12;
	speedTx.ExtId = 0;
	speedTx.DLC = 8;
	speedTx.IDE = CAN_ID_STD;
	speedTx.RTR = CAN_RTR_DATA;
	speedTx.TransmitGlobalTime = DISABLE;
	HAL_CAN_AddTxMessage( &hcan , &speedTx ,(speed_send.send_speed_arr) , &mailbox );
	/*DeComment if using Polling*/
	//while(HAL_CAN_IsTxMessagePending(&hcan,mailbox));
}
void Speed_Rx()
{	
	/*DeComment if using Polling*/	
	//while( ! ( HAL_CAN_GetRxFifoFillLevel(&hcan,CAN_RX_FIFO0)));		
	HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&speedRx,speed_receive.receive_speed_arr);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
void GPS_Task(void *pv){
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;
	xLastWakeTime = xTaskGetTickCount ();
	HAL_TIM_Base_Start_IT(&htim2);
	while(1){
		GPS_start=__HAL_TIM_GET_COUNTER(&htim2);
		GPS_I2C_receive();
		GPS_end=__HAL_TIM_GET_COUNTER(&htim2);
		HAL_TIM_Base_Stop_IT(&htim2);
		GPS_execution_time=GPS_end-GPS_start;
		/*Print Task Execution Time*/
		HAL_UART_Transmit(&DEBUG_UART_HANDLER,GPS_time, sprintf((char *)GPS_time, "%d", GPS_execution_time),HAL_MAX_DELAY);
		HAL_UART_Transmit(&DEBUG_UART_HANDLER,new_line,2,HAL_MAX_DELAY);
		xTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}
void CAN_Task(void *pv){
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;
	xLastWakeTime = xTaskGetTickCount ();
	HAL_TIM_Base_Start_IT(&htim2);
  	while(1)
		{
			CAN_start=__HAL_TIM_GET_COUNTER(&htim2);
			Longitude_Tx();
			Lattitude_Tx();
			Speed_Tx();
			CAN_end=__HAL_TIM_GET_COUNTER(&htim2);
			HAL_TIM_Base_Stop_IT(&htim2);
			CAN_execution_time=CAN_end-CAN_start;
			/*Print Task Execution Time*/
			HAL_UART_Transmit(&DEBUG_UART_HANDLER,CAN_time, sprintf((char *)CAN_time, "%d", CAN_execution_time),HAL_MAX_DELAY);
			HAL_UART_Transmit(&DEBUG_UART_HANDLER,new_line,2,HAL_MAX_DELAY);		
		  xTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
	uint8_t arr[15]="Message 1 sent ";
	HAL_UART_Transmit(&huart1,arr,15,HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
	uint8_t arr[15]="Message 2 sent ";
	HAL_UART_Transmit(&huart1,arr,15,HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
	uint8_t arr[15]="Message 3 sent ";
	HAL_UART_Transmit(&huart1,arr,15,HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

/*DeComment at the receiver's side*/
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
//{
//	Longitude_Rx();
//	Lattitude_Rx();	
//	Speed_Rx();
//	uint8_t arr[16]="Message received";
//	HAL_UART_Transmit(&huart1,arr,16,HAL_MAX_DELAY);	
//}


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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan,CAN_IT_TX_MAILBOX_EMPTY);
	
	
	TaskHandle_t Task1_handler;
	TaskHandle_t Task2_handler;

	xTaskCreate(GPS_Task,"GPS_Task",configMINIMAL_STACK_SIZE,NULL,1,&Task1_handler);
	xTaskCreate(CAN_Task,"CAN_Task",configMINIMAL_STACK_SIZE,NULL,2,&Task2_handler);
	vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_SILENT_LOOPBACK;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
	__disable_irq();
	while (1)
	{
	}
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
