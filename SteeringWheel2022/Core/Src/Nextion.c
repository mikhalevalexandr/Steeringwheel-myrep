#include "Nextion.h"
#include "main.h"
#include "cmsis_os.h"

uint8_t Cmd_End[3]={0xFF, 0xFF, 0xFF};//command end sequence
void NEXTION_SendString (char *ID, char *string)
{
	char buf[50]= {0};
	int len=sprintf (buf, "%s.txt=\"%s\"", ID, string);
	HAL_UART_Transmit_DMA (&huart1, (uint8_t *)buf, len);
	osDelay(30);
	HAL_UART_Transmit_DMA (&huart1, Cmd_End, 3);
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