/*
 * parser.ino
 *
 * Created : 1/15/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "main.h"

// Following variables are used to communicate between stm-board
// and the arduino
extern bool gPing_Command;
extern bool gSend_Lidar_Data;

bool gStart_Byte_Received = false;
bool gDev_Addr_Received = false;
uint8_t gReceived_Device_Address = 0x00;

uint8_t gLED_Intensity_Value = 0;

uint8_t gNum_Bytes_Received = 0;

void parse_STMByte(uint8_t c)
{
        if (!gStart_Byte_Received) {
                if (c == START_BYTE) {
                        gStart_Byte_Received = true;
                }
        }
        else {
                if (!gDev_Addr_Received) {
                        gReceived_Device_Address = c;
                        gDev_Addr_Received = true;

                        // Check if the current data is a ping request
                        if (!c) {
                                gPing_Command = true;
                                gStart_Byte_Received = false;
                                gDev_Addr_Received = false;
                        }
                }
                else {
                        if (gReceived_Device_Address == gLED_Address) {
//                                Serial.println(c);
                                gLED_Intensity_Value = c;
                                gNum_Bytes_Received = 0;
                                gStart_Byte_Received = false;
                                gDev_Addr_Received = false;
                        }
                        else {
                                // Ignore all other devices for now
                        }
                }
        }
}
