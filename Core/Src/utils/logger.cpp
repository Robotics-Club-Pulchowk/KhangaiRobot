
#include "stm32f4xx_hal.h"
#include "queue_custom.h"
#include "usart.h"

extern "C" int ITM_SendString(char *data, int len);

uint8_t gPrintfData[2][16*1024];
uint32_t gPrintfDataIndex[2] = { 0 };

uint8_t gSending_Index = 0;
volatile bool gPrintfDataSent = true;

int ITM_SendString(char *data, int len)
{
        int i = 0;
        for (; i < len; ++i)
        {
                // uint8_t buf_index = 1 - gSending_Index;
                // gPrintfData[buf_index][gPrintfDataIndex[buf_index]++] = data[i];
                ITM_SendChar(data[i]);
        }
        return i;
}

void log_data()
{
        if (gPrintfDataSent) {
                uint8_t send_index = 1 - gSending_Index;
                gSending_Index = 1 - gSending_Index;

                HAL_UART_Transmit_DMA(&huart2, gPrintfData[send_index], gPrintfDataIndex[send_index]);
                gPrintfDataIndex[send_index] = 0;
                gPrintfDataSent = false;
        }
}

void Logging_Handle_TxCplt()
{
        gPrintfDataSent = true;
}
