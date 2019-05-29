/*
 * logger.h
 *
 * Created : 5/21/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _LOGGER_H_
#define _LOGGER_H_


#include "queue_custom.h"
#include "usart.h"
#include "vec3.h"

#define LOG_BUFFER_SIZE     (16*1024)

extern Queue<uint8_t, LOG_BUFFER_SIZE> gLogging_Buffer;

void log_Angle(float psi, float rw);
void log_CompassOffsets(Vec3<float> offsets);


#endif // !_LOGGER_H_
