/*
 * error.cpp
 *
 * Created : 1/10/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "error.h"
#include "gpio.h"
#include "defines.h"

void error(Error err)
{
#ifdef _USE_BOARD_LEDS_FOR_ERROR_

        uint8_t err_no = (uint8_t)err;

        if (err_no & 0x01) {
                HAL_GPIO_WritePin(B_GreenLED_GPIO_Port, B_GreenLED_Pin, GPIO_PIN_SET);
        }
        if (err_no & 0x02) {
                HAL_GPIO_WritePin(B_OrangeLED_GPIO_Port, B_OrangeLED_Pin, GPIO_PIN_SET);
        }
        if (err_no & 0x04) {
                HAL_GPIO_WritePin(B_RedLED_GPIO_Port, B_RedLED_Pin, GPIO_PIN_SET);
        }
        if (err_no & 0x08) {
                HAL_GPIO_WritePin(B_BlueLED_GPIO_Port, B_BlueLED_Pin, GPIO_PIN_SET);
        }

#endif // _USE_BOARD_LEDS_FOR_ERROR_
}

void clear_Errors()
{
#ifdef _USE_BOARD_LEDS_FOR_ERROR_
        HAL_GPIO_WritePin(B_GreenLED_GPIO_Port, B_GreenLED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(B_OrangeLED_GPIO_Port, B_OrangeLED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(B_RedLED_GPIO_Port, B_RedLED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(B_BlueLED_GPIO_Port, B_BlueLED_Pin, GPIO_PIN_RESET);
#endif // _USE_BOARD_LEDS_FOR_ERROR_
}
