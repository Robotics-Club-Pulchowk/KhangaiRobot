/*
 * main.h
 *
 * Created : 1/15/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#include <SoftwareSerial.h>

#define _DEBUG_MODE

#define START_BYTE      (0xA5)

SoftwareSerial STM_Soft_Serial(A8, A9);

#define STM_SERIAL      (STM_Soft_Serial)

#define XLIDAR_R_SERIAL   (Serial1)
#define XLIDAR_B_SERIAL   (Serial2)
#define YLIDAR_SERIAL     (Serial3)

enum GameField {
        NONE,
        RED_F,
        BLUE_F
};

extern uint8_t gArduino_Address;
extern uint8_t gLED_Address;
extern uint8_t gXLidar_Address;
extern uint8_t gYLidar_Address;

extern uint8_t gLED_Intensity_Value;

extern GameField gCurrent_GameField;

// Function Prototypes
void send_DataPack(uint8_t addr, const uint8_t *buf, uint8_t len);

void send_LidarDataPack(uint8_t addr, unsigned long val);
unsigned long read_Lidar();
void parse_STMByte(uint8_t c);


#endif //! _MAIN_H_
