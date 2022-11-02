/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define MAX_BAT_LVL	4.15 // Volts - one battery max lvl
#define MIN_BAT_LVL	3.0 // Volts - one battery min lvl 
#define CELL_NUM	15 // number of cells in stack (18 is normal)
#define STACK_NUM	2 // number of stacks in HV battery (8 is normal)
#define HV_MAX_LVL MAX_BAT_LVL*CELL_NUM*STACK_NUM // Max Voltage of HV battery
#define HV_MIN_LVL MIN_BAT_LVL*CELL_NUM*STACK_NUM // Min Voltage of HV battery
#define STACK_MAX_LVL MAX_BAT_LVL*CELL_NUM // Max Voltage of HV battery
#define STACK_MIN_LVL MIN_BAT_LVL*CELL_NUM // Min Voltage of HV battery
#define GET_PERCENT(volt_lvl) (100.*(volt_lvl-STACK_MIN_LVL)/(STACK_MAX_LVL-STACK_MIN_LVL))

/******Nextion params*********/
#define N_TXT	"txt"
#define N_BCO	"bco"
#define N_VAL	"val"
#define N_PCO	"pco"
#define N_RADIO_GREEN	{0xF8, 0x00}
#define N_RADIO_RED	{0x07, 0xE0}

typedef struct
{
	uint32_t id;
	uint8_t data[8];
}	CAN_message_td;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern UART_HandleTypeDef huart1;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CSN1_Pin GPIO_PIN_13
#define CSN1_GPIO_Port GPIOC
#define LEDG_Pin GPIO_PIN_0
#define LEDG_GPIO_Port GPIOB
#define CE1_Pin GPIO_PIN_15
#define CE1_GPIO_Port GPIOA
#define IRQ1_Pin GPIO_PIN_8
#define IRQ1_GPIO_Port GPIOB
#define RadioBTN_Pin GPIO_PIN_9
#define RadioBTN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
