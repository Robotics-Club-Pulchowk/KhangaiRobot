/*
 * main.h
 *
 * Created : 1/15/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _MAIN_H_
#define _MAIN_H_


#define START_BYTE      (0xA5)

#define STM_SERIAL      (Serial2)

extern uint8_t gArduino_Address;
extern uint8_t gLidar_Address;
extern uint8_t gPneumatic_Address;

extern uint8_t gLED_Intensity_Value;

// Function Prototypes
void send_DataPack(uint8_t addr, const uint8_t *buf, uint8_t len);

void send_LidarDataPack(unsigned long val);
unsigned long read_Lidar();
void parse_STMByte(uint8_t c);


#endif //! _MAIN_H_
