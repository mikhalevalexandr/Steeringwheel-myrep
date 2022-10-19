#ifndef __NEXTION_H
#define __NEXTION_H
#include "stm32f1xx_hal.h"

void NEXTION_SendString (char *ID, char *string);
char* current_mission(int kok);

extern uint8_t Cmd_End[3];
#endif /* NEXTION_H_ */