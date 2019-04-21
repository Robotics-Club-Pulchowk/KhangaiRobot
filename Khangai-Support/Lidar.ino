#include "main.h"

uint8_t gLidar_Buffer[2] = { 0 };

void send_LidarDataPack(unsigned long val)
{
        gLidar_Buffer[0] = (uint8_t)(val >> 8);
        gLidar_Buffer[1] = (uint8_t)val;

        send_DataPack(gLidar_Address, gLidar_Buffer, 2);

        Serial.println(val);
}

unsigned long read_Lidar()
{
        unsigned long pulseWidth;
        pulseWidth = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds

        return pulseWidth;      // 1usec = 1 mm of distance
}
