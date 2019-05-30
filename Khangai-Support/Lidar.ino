/*
 * lidar.ino
 *
 * Created : 1/15/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "main.h"

//* Following variables are for timing purpose
const unsigned long gLidar_Read_Period = 10;
static unsigned long gXLidar_Read_Time = 0;
static unsigned long gYLidar_Read_Time = 0;
static bool gIs_First_Update = true;

void send_LidarDataPack(uint8_t addr, unsigned long val)
{
        uint8_t buf[2];
        //* Send the lidar data in big endian format
        buf[0] = (uint8_t)(val >> 8);
        buf[1] = (uint8_t)val;

        send_DataPack(addr, buf, 2);

#ifdef _DEBUG_MODE
        Serial.print(addr);
        Serial.print(" : ");
        Serial.println(val);
#endif
}

void update_Lidar()
{
        if (gIs_First_Update) {
                gXLidar_Read_Time = millis();
                delay(2);
                gYLidar_Read_Time = millis();
        }
        else {
                if (millis() - gXLidar_Read_Time > gLidar_Read_Period) {
                        gXLidar_Read_Time = millis();

                        //* Read and send XLidar data here
                }

                if (millis() - gYLidar_Read_Time > gLidar_Read_Period) {
                        gYLidar_Read_Time = millis();

                        //* Read and send YLidar data here
                }
        }
}
