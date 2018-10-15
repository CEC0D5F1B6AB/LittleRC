
#include "bsp.h"
#include "main.h"

#include "stm32f0xx_hal.h"

uint8_t SPI_RW(uint8_t input)
{
		uint8_t output;
    
    if (HAL_SPI_TransmitReceive(&hspi1, &input, &output, 1, 0xff) != HAL_OK) {

			    return 0xFF;
    }
    return output;
}
