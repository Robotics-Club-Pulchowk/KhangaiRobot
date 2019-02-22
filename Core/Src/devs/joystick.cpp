/*
 * joystick.cpp
 *
 * Created : 02/19/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "joystick.h"
#include "defines.h"

#define JOYSTICK_START_BYTE     (0xA5)
#define NUM_JOYSTICK_BYTES      (7)

static JoyStick gJoyStick;
static JoyStick_Data gNull_JData;

static uint8_t gRx2Data;

static void fill_JoyData(JoyStick_Data *joy, uint8_t data[NUM_JOYSTICK_BYTES]);

void init_JoyStick(UART_HandleTypeDef *huart)
{
        gJoyStick.huart = huart;

        // Null JoyStick Data
        gNull_JData.button1 = 0;
        gNull_JData.lt = 0;
        gNull_JData.rt = 0;
        gNull_JData.l_hatx = 0;
        gNull_JData.l_haty = 0;
        gNull_JData.r_hatx = 0;
        gNull_JData.r_haty = 0;
        
        // __HAL_UART_ENABLE_IT(huart, UART_IT_TC);
        HAL_UART_Receive_DMA(huart, &gRx2Data, 1);
}

bool joy_Empty()
{
        return gJoyStick.data.is_Empty();
}

JoyStick_Data read_JoyStick()
{
        if (!joy_Empty()) {
                return gJoyStick.data.lookup();
        }
        return gNull_JData;
}

template<typename T, size_t N>
void fill_Array(T (&arr)[N], T num)
{
        for (size_t i = 0; i < N; ++i) {
                arr[i] = num;
        }
}


JoyStick_Data gJoy;
uint8_t gJoy_Data_Arr[NUM_JOYSTICK_BYTES];
static bool gStart_Byte_Rx2 = false;
static uint16_t gRx2_Data_num = 0;

void JoyStick_Handle_RxCplt()
{
        __HAL_UART_FLUSH_DRREGISTER(gJoyStick.huart);
        if (!gStart_Byte_Rx2) {
                if (gRx2Data == JOYSTICK_START_BYTE) {
                        gStart_Byte_Rx2 = true;
                }
        }
        else {
                if (gRx2_Data_num < NUM_JOYSTICK_BYTES) {
                        // printf("%x ", (int8_t)gRx2Data);
                        gJoy_Data_Arr[gRx2_Data_num] = gRx2Data;
                        ++gRx2_Data_num;
                }
                else {
                        gStart_Byte_Rx2 = false;
                        gRx2_Data_num = 0;
                        fill_JoyData(&gJoy, gJoy_Data_Arr);
                        fill_Array(gJoy_Data_Arr, (uint8_t)0);
                        gJoyStick.data.insert(gJoy);
                }
        }
}

static void fill_JoyData(JoyStick_Data *joy, uint8_t data[NUM_JOYSTICK_BYTES])
{
        joy->button1 = data[0];
        joy->lt = data[1];
        joy->rt = data[2];
        joy->l_hatx = data[3];
        joy->l_haty = data[4];
        joy->r_hatx = data[5];
        joy->r_haty = data[6];
}

#ifdef _USE_BOARD_LEDS_FOR_JOYSTICK
static void use_LEDs(Vec3<float> vels, float tol)
{
        float vx = vels.getX();
        float vy = vels.getY();

        // Right - Orange
        // Left - Blue
        // Ahead - Green
        // Back - Red
        if (vx > tol) {
                HAL_GPIO_WritePin(B_OrangeLED_GPIO_Port, B_OrangeLED_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(B_BlueLED_GPIO_Port, B_BlueLED_Pin, GPIO_PIN_RESET);
        }
        else if (vx < -tol) {
                HAL_GPIO_WritePin(B_OrangeLED_GPIO_Port, B_OrangeLED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(B_BlueLED_GPIO_Port, B_BlueLED_Pin, GPIO_PIN_SET);
        }
        else {
                HAL_GPIO_WritePin(B_OrangeLED_GPIO_Port, B_OrangeLED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(B_BlueLED_GPIO_Port, B_BlueLED_Pin, GPIO_PIN_RESET);
        }

        if (vy > tol) {
                HAL_GPIO_WritePin(B_GreenLED_GPIO_Port, B_GreenLED_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(B_RedLED_GPIO_Port, B_RedLED_Pin, GPIO_PIN_RESET);
        }
        else if (vy < -tol) {
                HAL_GPIO_WritePin(B_GreenLED_GPIO_Port, B_GreenLED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(B_RedLED_GPIO_Port, B_RedLED_Pin, GPIO_PIN_SET);
        }
        else {
                HAL_GPIO_WritePin(B_GreenLED_GPIO_Port, B_GreenLED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(B_RedLED_GPIO_Port, B_RedLED_Pin, GPIO_PIN_RESET);
        }
}
#endif

static Vec3<float> parse_JoyData(JoyStick_Data joy)
{
        int8_t lx = (int8_t)(joy.r_hatx);
        float vx = (float)(lx) / 128.0;
        int8_t ly = (int8_t)(joy.r_haty);
        float vy = (float)(ly) / 128.0;

        Vec3<float> vels(vx, vy, 0);

        #ifdef _USE_BOARD_LEDS_FOR_JOYSTICK

                use_LEDs(vels, 0.128);

        #endif

        return vels;
}

Vec3<float> parse_JoyStick()
{
        JoyStick_Data joydata = read_JoyStick();
        Vec3<float> vels = parse_JoyData(joydata);

        return vels;
}
