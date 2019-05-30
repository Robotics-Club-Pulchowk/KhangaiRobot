/*
 * Khangai-Support.ino
 *
 * Created : 1/15/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "main.h"

//* Function Prototypes
void send_PingReply();
void update_Lidar();

//* Following variables are for timing purpose
const unsigned long gLED_Intensity_Read_Period = 100;
unsigned long gLED_Intensity_Read_Time = 0;

//* Following are the addresses of the devices associated with the Arduino Mega
uint8_t gArduino_Address = 0x00;
uint8_t gLED_Address = 0x01;
uint8_t gXLidar_Address = 0x02;
uint8_t gYLidar_Address = 0x03;

//* Following variables are used to communicate between stm-board
//* and the arduino
bool gPing_Command = false;
bool gSend_Lidar_Data = false;

const uint8_t gRed_LED_Pin = 10;
const uint8_t gBlue_LED_Pin = 8;

void setup()
{
        //* Initialize LED Strip pins in pwm mode for intensity control
        pinMode(gRed_LED_Pin, OUTPUT);
        pinMode(gBlue_LED_Pin, OUTPUT);
        analogWrite(gRed_LED_Pin, 0);
        analogWrite(gBlue_LED_Pin, 0);

        //* Initialize the Serials
        (Serial).begin(115200);
        STM_SERIAL.begin(9600);

        Serial.println("Hello World!!");

        //* Store current time for periodic update
        gLED_Intensity_Read_Time = millis();
}

void loop()
{
        //* Read any available bytes and parse it to obtain any useful data
        if (STM_SERIAL.available()) {
                uint8_t c = STM_SERIAL.read();
                parse_STMByte(c);
        }

        //* Update lidar values to the stm if the timing constraint is fulfilled
        update_Lidar();

        if (millis() - gLED_Intensity_Read_Time > gLED_Intensity_Read_Period) {
                gLED_Intensity_Read_Time = millis();

                //* We expect the intensity byte to contain data in the form :
                //* (blue red)
                //* blue and red are of a nibble width each
                uint8_t red = gLED_Intensity_Value & 0x0f;
                uint8_t blue = (gLED_Intensity_Value & 0xf0) >> 4;

                //* Map the nibble data to byte data
                red = map(red, 0, 15, 0, 255);
                blue = map(blue, 0, 15, 0, 255);

                //* Analogwrite the LED intensity value
                analogWrite(gRed_LED_Pin, red);
                analogWrite(gBlue_LED_Pin, blue);
        }

        //* If ping command is obtained, send ok status
        if (gPing_Command) {
                gPing_Command = false;
                send_PingReply();
        }
}

void send_DataPack(uint8_t addr, const uint8_t *buf, uint8_t len)
{
        //* Send the data in format: 0xA5 ADDR DATA
        STM_SERIAL.write(START_BYTE);
        STM_SERIAL.write(addr);
        for (uint8_t i = 0; i < len; ++i) {
                STM_SERIAL.write(buf[i]);
        }
}

void send_PingReply()
{
        uint8_t reply = 0x5A;
        send_DataPack(gArduino_Address, &reply, 1);
}
