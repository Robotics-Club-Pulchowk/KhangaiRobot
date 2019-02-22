/*
 * int_config.cpp
 * 
 * Created : 11/16/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "freewheel.h"
#include "arduino.h"
#include "joystick.h"

extern struct Enc gXEnc;
extern struct Enc gYEnc;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
        switch (GPIO_Pin) {
                // This interrupt pin is connected to gXEnc
                case GPIO_PIN_7 : {     // PD7
                        Enc_HandleCount(&gXEnc);
                } break;
                
                // This interrupt pin is connected to gYEnc
                case GPIO_PIN_4 : {    // PB4
                        Enc_HandleCount(&gYEnc);
                }
        }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
        if (huart->Instance == USART3) {
                Arduino_Handle_RxCplt();
        }
        else if (huart->Instance == USART2) {
                JoyStick_Handle_RxCplt();
        }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
        if (huart->Instance == USART3) {
                Arduino_Handle_TxCplt();
        }
}

void HAL_UART_IRQCallback(UART_HandleTypeDef *huart)
{
        if (huart->Instance == USART3) {
                printf("Here\n");
        }
}
