/*
 * printf_config.cpp
 * 
 * Created : 11/10/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "stm32f4xx_hal.h"

static volatile uint8_t gPrintfData[32*1024];
static volatile uint32_t gPrintfDataIndex = 0;

int ITM_SendString(char *data, int len)
{
        int i = 0;
        for (; i < len; ++i)
        {
                // gPrintfData[gPrintfDataIndex++] = data[i];
                ITM_SendChar(data[i]);
        }
        return i;
}

int _write(int file, char *data, int len)
{
        int sent = ITM_SendString(data, len);
        return sent;
}

void send_AllData()
{
        for (uint32_t i = 0; i < gPrintfDataIndex; ++i) {
                ITM_SendChar(gPrintfData[i]);
                asm volatile("nop");
                asm volatile("nop");
        }
}
