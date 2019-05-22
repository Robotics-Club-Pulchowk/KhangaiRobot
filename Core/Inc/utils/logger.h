/*
 * logger.h
 *
 * Created : 5/21/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "queue_custom.h"
#include "usart.h"

#define LOG_BUFFER_SIZE     (16*1024)

extern Queue<uint8_t, LOG_BUFFER_SIZE> gLogging_Buffer;
