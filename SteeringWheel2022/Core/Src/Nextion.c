#include "Nextion.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include "NRF24.h"
uint8_t Cmd_End[3]={0xFF, 0xFF, 0xFF};//command end sequence
void NEXTION_SendString (char *id, char *param, char *string)
{
	char buf[50]= {0};
	int len= 0;
	if (strncmp(param, "txt", 3) == 0)
	{
		len = sprintf (buf, "%s.%s=\"%s\"", id, param, string);
	}
	else
	{
//		len = sprintf (buf, "%s.%s=%s", id, param, string);
	}
//	osDelay(30);
	HAL_UART_Transmit_DMA (&huart1, (uint8_t *)buf, len);
	osDelay(30);
	HAL_UART_Transmit_DMA (&huart1, Cmd_End, 3);
	osDelay(10);
//	HAL_UART_Transmit (&huart1, (uint8_t *)buf, len, 1000);
//	HAL_UART_Transmit (&huart1, Cmd_End, 3, 100);
}
void NEXTION_SendInt (char *id, char *param, int string)
{
	char buf[50]= {0};
	int len= 0;
	if (strncmp(param, "txt", 3) == 0)
	{
//		len = sprintf (buf, "%s.%s=\"%s\"", id, param, string);
	}
	else
	{
		len = sprintf (buf, "%s.%s=%d", id, param, string);
	}
//	osDelay(30);
	HAL_UART_Transmit_DMA (&huart1, (uint8_t *)buf, len);
  osDelay(20);
	HAL_UART_Transmit_DMA (&huart1, Cmd_End, 3);
	osDelay(10);
//	HAL_UART_Transmit (&huart1, (uint8_t *)buf, len, 1000);
//	HAL_UART_Transmit (&huart1, Cmd_End, 3, 100);
}
char* current_mission(int kok){
switch (kok)
		{
			case 0X20:
				return "ACCELE\r\nRATION";
					break;
			case 0x21: 
					return "SKIDPAD";
					break;
			case 0x22:
					return "AUTO\r\nCROSS";
					break;
			case 0x23: 
					return "TRACKDRIVE";
					break;
			case 0x24:
					return "EBS\r\nTEST";
					break;
			case 0x25: 
					return "INSPECTION";
					break;
			case 0x26: 
					return "MANUAL\r\nDRIVING";
					break;
			default:
					return "ERROR";
		}
}