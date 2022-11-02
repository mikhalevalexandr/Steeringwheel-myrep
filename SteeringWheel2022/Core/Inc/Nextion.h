#ifndef __NEXTION_H
#define __NEXTION_H
#include "stm32f1xx_hal.h"

void NEXTION_SendString (char *id, char *param, char *string);
void NEXTION_SendInt (char *id, char *param, int string);

char* current_mission(int kok);

extern uint8_t Cmd_End[3];
#endif /* NEXTION_H_ */