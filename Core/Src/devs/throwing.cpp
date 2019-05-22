/*
 * throwing.cpp
 * 
 * Created : 5/18/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "throwing.h"
#include "usart.h"
#include "defines.h"

#define THROWING_UART   (huart4)

#define MAX_PACKET_LENGTH       (2)
#define MAX_COMMAND_QUEUE       (2)

uint8_t gThrowing_TxData[MAX_PACKET_LENGTH];

Throwing& Throwing::get_Instance()
{
        static Throwing sThrowing_Instance;

        return sThrowing_Instance;
}

int Throwing::init()
{
        __HAL_UART_ENABLE_IT(&THROWING_UART, UART_IT_TC);

        gThrowing_TxData[0] = START_BYTE;

        return 0;
}

Queue<uint8_t, MAX_COMMAND_QUEUE> gThrowing_Commands;

uint8_t gThrowing_TxBuffer[MAX_PACKET_LENGTH];
bool gAll_Command_Sent = true;

int Throwing::write(uint8_t cmd)
{
        if (gAll_Command_Sent) {
                gAll_Command_Sent = false;

                gThrowing_TxData[1] = cmd;
                HAL_UART_Transmit_IT(&THROWING_UART, gThrowing_TxData, 2);
        }
        else {
                gThrowing_Commands.insert(cmd);
        }

        return 0;
}

void Throwing_Handle_TxCplt(void)
{
        if (!gThrowing_Commands.is_Empty()) {
                gThrowing_TxData[1] = gThrowing_Commands.lookup();
                HAL_UART_Transmit_IT(&THROWING_UART, gThrowing_TxData, 2);
        }
        else {
                gAll_Command_Sent = true;
        }
}
