
#include "stm32f4xx_hal.h"
#include "queue_custom.h"

extern "C" int ITM_SendString(char *data, int len);

Queue<char, 32*1024> gPrintfData;

// volatile uint8_t gPrintfData[32*1024];
// volatile uint32_t gPrintfDataIndex = 0;

int ITM_SendString(char *data, int len)
{
        int i = 0;
        for (; i < len; ++i)
        {
                // gPrintfData[gPrintfDataIndex++] = data[i];
                ITM_SendChar(data[i]);
                // gPrintfData.insert(data[i]);
        }
        return i;
}

void send_AllData()
{
        // for (uint32_t i = 0; i < gPrintfDataIndex; ++i) {
        //         ITM_SendChar(gPrintfData[i]);
        //         asm volatile("nop");
        //         asm volatile("nop");
        // }
}
