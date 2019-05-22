/*
 * logger.cpp
 *
 * Created : 5/21/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "stm32f4xx_hal.h"
#include "logger.h"

Queue<uint8_t, LOG_BUFFER_SIZE> gLogging_Buffer;
static bool gLogDataSent = true;

void log_data()
{
        if (gLogDataSent && !gLogging_Buffer.is_Empty()) {
                gLogDataSent = false;
                uint8_t data = gLogging_Buffer.lookup();
                HAL_UART_Transmit_IT(&huart2, &data, 1);
                HAL_Delay(1);
        }
}

uint8_t gSent_Count = 0;
void Logging_Handle_TxCplt()
{
        gLogDataSent = true;
}
