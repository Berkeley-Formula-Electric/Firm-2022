#include "FEB_log.h"

extern UART_HandleTypeDef huart2;

void FEB_log(char *module, char *level, char *msg) {
  char str[128];
  sprintf(str, "<%s> [%s] %s\r\n", module, level, msg);
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

