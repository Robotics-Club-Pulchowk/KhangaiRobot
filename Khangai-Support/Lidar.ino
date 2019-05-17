/*
 * lidar.ino
 *
 * Created : 1/15/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "main.h"

uint8_t gLidar_Buffer[2] = { 0 };

void send_LidarDataPack(unsigned long val)
{
        //* Send the lidar data in big endian format
        gLidar_Buffer[0] = (uint8_t)(val >> 8);
        gLidar_Buffer[1] = (uint8_t)val;

        send_DataPack(gLidar_Address, gLidar_Buffer, 2);

        Serial.println(val);
}

unsigned long read_Lidar()
{
        unsigned long pulseWidth;

        //* Count how long the pulse is high in microseconds
        pulseWidth = pulseIn(3, HIGH);

        return pulseWidth;      //* 1usec = 1 mm of distance
}
