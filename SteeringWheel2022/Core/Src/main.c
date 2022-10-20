/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24.h"
#include <string.h>
#include "Nextion.h"

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

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId defaultTaskHandle;
osThreadId RadioHandle;
osThreadId DisplayHandle;
/* USER CODE BEGIN PV */
char str1[20]={0};
uint8_t recieve1=0x20;
uint8_t UART_RX[20]={0};
uint8_t buf1[1]={0};
uint8_t buf2[1]={0};
uint8_t dt_reg=0;
char Autonomius_mission[15]="Error";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
void StartDefaultTask(void const * argument);
void Radio_send_task(void const * argument);
void Display_task(void const * argument);

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
	uint8_t retr_cnt, dt;
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
  MX_SPI2_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	CSN1_OFF;
	NRF24_ini();
	dt_reg = NRF24_ReadReg(CONFIG);
	HAL_Delay(200);
				NRF24L01_Send(buf1);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Radio */
  osThreadDef(Radio, Radio_send_task, osPriorityNormal, 0, 128);
  RadioHandle = osThreadCreate(osThread(Radio), NULL);

  /* definition and creation of Display */
  osThreadDef(Display, Display_task, osPriorityNormal, 0, 128);
  DisplayHandle = osThreadCreate(osThread(Display), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
						HAL_Delay(30);

//		NRF24L01_Receive();
//		retr_cnt = dt & 0xF;
//		i++;
//		retr_cnt_full += retr_cnt;
		dt_reg = NRF24_ReadReg(CONFIG);

		sprintf(str1,"CONFIG: 0x%02X\r\n",dt_reg);

//		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

		dt_reg = NRF24_ReadReg(EN_AA);

		sprintf(str1,"EN_AA: 0x%02X\r\n",dt_reg);

//		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

		dt_reg = NRF24_ReadReg(EN_RXADDR);

		sprintf(str1,"EN_RXADDR: 0x%02X\r\n",dt_reg);

//		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

		dt_reg = NRF24_ReadReg(STATUS);

//		sprintf(str1,"STATUS: 0x%02X\r\n",dt_reg);

//		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

		dt_reg = NRF24_ReadReg(RF_SETUP);
//		sprintf(str1,"RF_SETUP: 0x%02X\r\n",dt_reg);

//		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

//		NRF24_Read_Buf(TX_ADDR,buf1,3);

//		sprintf(str1,"TX_ADDR: 0x%02X, 0x%02X, 0x%02X\r\n",buf1[0],buf1[1],buf1[2]);

//		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

//		NRF24_Read_Buf(RX_ADDR_P0,buf1,3);

//		sprintf(str1,"RX_ADDR: 0x%02X, 0x%02X, 0x%02X\r\n",buf1[0],buf1[1],buf1[2]);

//		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
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
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CE1_GPIO_Port, CE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CSN1_Pin */
  GPIO_InitStruct.Pin = CSN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CSN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LEDG_Pin */
  GPIO_InitStruct.Pin = LEDG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CE1_Pin */
  GPIO_InitStruct.Pin = CE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CE1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IRQ1_Pin RadioBTN_Pin */
  GPIO_InitStruct.Pin = IRQ1_Pin|RadioBTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER  CODE BEGIN 4 */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *haurt)
{
	HAL_UART_Receive_DMA (&huart1, UART_RX, sizeof(UART_RX));
}
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Radio_send_task */
/**
* @brief Function implementing the Radio thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Radio_send_task */
void Radio_send_task(void const * argument)
{
  /* USER CODE BEGIN Radio_send_task */
	uint8_t on=0x6F, off=0x85;
  /* Infinite loop */
  for(;;)
  {
    osDelay(30);
				if (RadioBTN)
		{
			memcpy(buf1, &on,1);
			NRF24L01_Send(buf1);
				LED_OFF;
		}
		else if (!RadioBTN)
		{
			memcpy(buf1, &off,1);
			NRF24L01_Send(buf1);
		LED_ON;
		}
				dt_reg = NRF24_ReadReg(CONFIG);

		sprintf(str1,"CONFIG: 0x%02X\r\n",dt_reg);

//		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

		dt_reg = NRF24_ReadReg(EN_AA);

		sprintf(str1,"EN_AA: 0x%02X\r\n",dt_reg);

//		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

		dt_reg = NRF24_ReadReg(EN_RXADDR);

		sprintf(str1,"EN_RXADDR: 0x%02X\r\n",dt_reg);

//		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

		dt_reg = NRF24_ReadReg(STATUS);

//		sprintf(str1,"STATUS: 0x%02X\r\n",dt_reg);

//		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

		dt_reg = NRF24_ReadReg(RF_SETUP);
  }
  /* USER CODE END Radio_send_task */
}

/* USER CODE BEGIN Header_Display_task */
/**
* @brief Function implementing the Display thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Display_task */
void Display_task(void const * argument)
{
  /* USER CODE BEGIN Display_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
		NEXTION_SendString("AMI", current_mission(recieve1));		
  }
  /* USER CODE END Display_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
