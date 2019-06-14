/*
 * lidar.ino
 *
 * Created : 1/15/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "TFMini.h"
#include "main.h"

//* Following variables are for timing purpose
const unsigned long gLidar_Read_Period = 10;
static unsigned long gXLidar_Read_Time = 0;
static unsigned long gYLidar_Read_Time = 0;
static bool gIs_First_Update = true;

//* Biases of respective lidars
static unsigned long gXLidar_Bias = 0;
static unsigned long gYLidar_Bias = 90;

//* Construct the lidar's object
TFMini gXLidar;
TFMini gYLidar;

//* Initialize the lidars as shown in BasicReading.ino example of TFMini
void init_Lidars()
{
        XLIDAR_SERIAL.begin(TFMINI_BAUDRATE);
        YLIDAR_SERIAL.begin(TFMINI_BAUDRATE);

#ifdef _DEBUG_MODE
        Serial.println("Initializing Lidars...");
#endif

        gXLidar.begin(&XLIDAR_SERIAL);
        gYLidar.begin(&YLIDAR_SERIAL);
}

void send_LidarDataPack(uint8_t addr, unsigned long val)
{
        uint8_t buf[2];
        //* Send the lidar data in big endian format
        buf[0] = (uint8_t)(val >> 8);
        buf[1] = (uint8_t)val;

        send_DataPack(addr, buf, 2);

}

void update_Lidars()
{
        uint16_t dist;
        if (gIs_First_Update) {
                gXLidar_Read_Time = millis();
                delay(2);
                gYLidar_Read_Time = millis();

                gIs_First_Update = false;
        }
        else {
                if (millis() - gXLidar_Read_Time > gLidar_Read_Period) {
                        gXLidar_Read_Time = millis();

                        //* Read and send XLidar data here
                        dist = gXLidar.getDistance()*10 + gXLidar_Bias;
                        send_LidarDataPack(gXLidar_Address, dist);
                        
#ifdef _DEBUG_MODE
                        uint16_t strength = gXLidar.getRecentSignalStrength();
                        
                        // Display the measurement
                        Serial.print("XLidar(0x02) : ");
                        Serial.print(dist);
                        Serial.print(" mm      sigstr: ");
                        Serial.println(strength);
#endif

                }

                if (millis() - gYLidar_Read_Time > gLidar_Read_Period) {
                        gYLidar_Read_Time = millis();

                        //* Read and send YLidar data here
                        dist = gYLidar.getDistance()*10 + gYLidar_Bias;
                        send_LidarDataPack(gYLidar_Address, dist);
                        
#ifdef _DEBUG_MODE
                        uint16_t strength = gYLidar.getRecentSignalStrength();
                        
                        // Display the measurement
                        Serial.print("YLidar(0x03) : ");
                        Serial.print(dist);
                        Serial.print(" mm      sigstr: ");
                        Serial.println(strength);
#endif
                }
        }
}
