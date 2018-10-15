#ifndef __BSP_H_
#define __BSP_H_

#include <stdint.h>

#include "main.h"
#include "stm32f0xx_hal.h"

extern ADC_HandleTypeDef hadc;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;


#define SYS_LED_L() HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, GPIO_PIN_RESET)
#define SYS_LED_H() HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, GPIO_PIN_SET)

#define SI_ENP_L() HAL_GPIO_WritePin(SI_ENP_GPIO_Port, SI_ENP_Pin, GPIO_PIN_RESET)
#define SI_ENP_H() HAL_GPIO_WritePin(SI_ENP_GPIO_Port, SI_ENP_Pin, GPIO_PIN_SET)

#define SI_ENT_L() HAL_GPIO_WritePin(SI_ENT_GPIO_Port, SI_ENT_Pin, GPIO_PIN_RESET)
#define SI_ENT_H() HAL_GPIO_WritePin(SI_ENT_GPIO_Port, SI_ENT_Pin, GPIO_PIN_SET)


uint8_t SPI_RW(uint8_t input);

#endif
