/*
 * arduino.cpp
 * 
 * Created : 1/5/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "arduino.h"
#include "usart.h"

extern Arduino_Device gXLidar_Dev;

#define MAX_PACKET_LENGTH       (32)

#define ARDUINO_UART            (huart3)
#define ARDUINO_START_BYTE      (0xA5)

#define MAX_DEVICES             (3)
#define MAX_BYTES_PER_DEVICE    (8)

static uint8_t gBytes_Per_Device[MAX_DEVICES];
static uint8_t gRxBuffer[MAX_DEVICES][MAX_BYTES_PER_DEVICE];

struct Arduino_Packet
{
        uint8_t start_byte;
        uint8_t dev_id;
        uint8_t *buffer;
        uint16_t buf_len;
};

static Queue<Arduino_Packet, 8> gTxData;
static uint8_t gRxData;

Arduino_Device::Arduino_Device(uint8_t id, uint8_t num_bytes)
{
        if (id > MAX_DEVICES) {
                _Error_Handler(__FILE__, __LINE__);
        }

        id_ = id;
        gBytes_Per_Device[id_] = num_bytes;
}

bool Arduino_Device::initiated = false;
int Arduino_Device::init()
{
        if (!initiated) {
                // __HAL_UART_FLUSH_DRREGISTER(&ARDUINO_UART);
                __HAL_UART_ENABLE_IT(&ARDUINO_UART, UART_IT_TC);
                HAL_UART_Receive_DMA(&ARDUINO_UART, &gRxData, 1);

                initiated = true;
        }

        return 0;
}

float Arduino_Device::read()
{
        if (!rx_data_.is_Empty()) {
                return rx_data_.lookup();
        }
        return 0;
}

bool Arduino_Device::available()
{
        return !(rx_data_.is_Empty());
}

void Arduino_Device::denit()
{

}


uint8_t gTxBuffer[MAX_PACKET_LENGTH];
bool gSending_Packet = false;

static void copy_Bytes(const uint8_t *src, uint8_t *dest, uint16_t len)
{
        for (uint16_t i = 0; i < len; ++i) {
                dest[i] = src[i];
        }
}

int Arduino_Device::write(uint8_t *buf, uint16_t len)
{
        if (len > (MAX_PACKET_LENGTH - 2)) {
                _Error_Handler(__FILE__, __LINE__);
                return -1;
        }

        Arduino_Packet pack;
        pack.start_byte = 0xA5;
        pack.dev_id = id_;
        pack.buffer = buf;
        pack.buf_len = len;

        if (!gSending_Packet) {
                // gSending_Packet = true;

                gTxBuffer[0] = pack.start_byte;
                gTxBuffer[1] = pack.dev_id;
                copy_Bytes(pack.buffer, &gTxBuffer[2], pack.buf_len);
                // 2 bytes extra to hold start byte and device id
                uint16_t total_size = pack.buf_len + 2;

                HAL_UART_Transmit_IT(&ARDUINO_UART, gTxBuffer, total_size);
        }
        else {
                gTxData.insert(pack);
        }

        return len;
}

void Arduino_Handle_TxCplt(void)
{
        if (!gTxData.is_Empty()) {
                Arduino_Packet pack = gTxData.lookup();
                gTxBuffer[0] = pack.start_byte;
                gTxBuffer[1] = pack.dev_id;
                copy_Bytes(pack.buffer, &gTxBuffer[2], pack.buf_len);
                // 2 bytes extra to hold start byte and device id
                uint16_t total_size = pack.buf_len + 2;

                HAL_UART_Transmit_IT(&ARDUINO_UART, gTxBuffer, total_size);
        }
        else {
                gSending_Packet = false;
        }
}


static bool gStart_Byte_Received = false;
static uint16_t gReceived_Data_num = 0;
static uint8_t gCurrent_Device_ID = 0;

void Arduino_Handle_RxCplt(void)
{
        __HAL_UART_FLUSH_DRREGISTER(&ARDUINO_UART);
        // printf("%d  ", gRxData);

        if (!gStart_Byte_Received) {
                if (gRxData == ARDUINO_START_BYTE) {
                        ++gReceived_Data_num;
                        gStart_Byte_Received = true;
                }
        }
        else {
                if (gReceived_Data_num == 1) {
                        ++gReceived_Data_num;
                        gCurrent_Device_ID = gRxData;
                }
                else {
                        if (gReceived_Data_num < (gBytes_Per_Device[gCurrent_Device_ID] + 2)) {
                                gRxBuffer[gCurrent_Device_ID][gReceived_Data_num - 2] = gRxData;
                                ++gReceived_Data_num;

                                // These devices expect only float values
                                if (gBytes_Per_Device[gCurrent_Device_ID] == 2 && gReceived_Data_num == 4) {
                                        if (gCurrent_Device_ID == gXLidar_Dev.get_ID()) {
                                                int16_t val = (gRxBuffer[gCurrent_Device_ID][0]) << 8;
                                                val |= gRxBuffer[gCurrent_Device_ID][1];
                                                gXLidar_Dev.store((float)val);
                                                // printf("%d\n", val);
                                        }
                                }
                        }

                        if (gReceived_Data_num >= (gBytes_Per_Device[gCurrent_Device_ID] + 2)) {
                                gStart_Byte_Received = false;
                                gReceived_Data_num = 0;
                                gCurrent_Device_ID = 0x00;
                        }
                }
        }
}
