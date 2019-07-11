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
void update_Lidars();
void init_Lidars();

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

const uint8_t gRed_LED_Pin = 11;
const uint8_t gBlue_LED_Pin = 5;

const uint8_t gRed_GameField_Pin = 23;
const uint8_t gBlue_GameField_Pin = 25;

GameField gCurrent_GameField;

void setup()
{
        //* Initialize the Serials
        (Serial).begin(115200);
        STM_SERIAL.begin(9600);
 
        //* Initialize LED Strip pins in pwm mode for intensity control
        pinMode(gRed_LED_Pin, OUTPUT);
        pinMode(gBlue_LED_Pin, OUTPUT);

        //* Initialize GameField Selecting Pins
        pinMode(gRed_GameField_Pin, INPUT);
        pinMode(gBlue_GameField_Pin, INPUT);

        int red_val = digitalRead(gRed_GameField_Pin);
        int blue_val = digitalRead(gBlue_GameField_Pin);

        digitalWrite(gRed_LED_Pin, red_val);
        digitalWrite(gBlue_LED_Pin, blue_val);

        if (red_val & !blue_val) {
                gCurrent_GameField = GameField::RED_F;
                Serial.println("Red Field Selected");
        }
        else if (blue_val & !red_val) {
                gCurrent_GameField = GameField::BLUE_F;
                Serial.println("Blue Field Selected");
        }
        else {
                gCurrent_GameField = GameField::NONE;
                Serial.println("None Field Selected");
        }

        Serial.println("Hello World!!");

        init_Lidars();

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
        update_Lidars();

        if (millis() - gLED_Intensity_Read_Time > gLED_Intensity_Read_Period) {
                gLED_Intensity_Read_Time = millis();
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
