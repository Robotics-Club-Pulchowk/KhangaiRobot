/*
 * lidar.ino
 *
 * Created : 1/15/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "main.h"

void send_LidarDataPack(uint8_t addr, unsigned long val)
{
        uint8_t buf[2];
        //* Send the lidar data in big endian format
        buf[0] = (uint8_t)(val >> 8);
        buf[1] = (uint8_t)val;

        send_DataPack(addr, buf, 2);

        Serial.println(val);
}
