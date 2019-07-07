/*
 * joystick.cpp
 *
 * Created : 02/19/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "joystick.h"
#include "defines.h"
#include "array.h"

#include "crc_hash.h"
#include "logger.h"

#include <math.h>

#define JOYSTICK_START_BYTE     (START_BYTE)
#define NUM_JOYSTICK_BYTES      (8)

static JoyStick_Handle gJoyStick;
static JoyStick_Data gNull_JData;

static CRC_Hash gJoyStick_CRC(7);
static uint32_t gJoy_Err_Count = 0;

static uint8_t gRx2Data;

static void fill_JoyData(JoyStick_Data *joy, uint8_t data[NUM_JOYSTICK_BYTES]);

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
                        uint8_t rem = gRx2Data;

                        gStart_Byte_Rx2 = false;
                        gRx2_Data_num = 0;

                        uint8_t hash = gJoyStick_CRC.get_Hash(gJoy_Data_Arr, NUM_JOYSTICK_BYTES);
                        // arrPrint(gJoy_Data_Arr);
                        // printf("\t%d\t", hash);

                        if (hash == rem) {
                                fill_JoyData(&gJoy, gJoy_Data_Arr);
                                gJoyStick.data.insert(gJoy);

                                // printf("No Error!!");
                        }
                        else {
                                ++gJoy_Err_Count;
                                log_JoyStickError(gJoy_Err_Count);
                                // printf("%d, %d, %ld", rem, hash, gJoy_Err_Count);
                        }
                        // printf("\n");

                        arrFill(gJoy_Data_Arr, (uint8_t)0);
                }
        }
}

static void fill_JoyData(JoyStick_Data *joy, uint8_t data[NUM_JOYSTICK_BYTES])
{
        joy->button1 = data[0];
        joy->button2 = data[1];
        joy->lt = data[2];
        joy->rt = data[3];
        joy->l_hatx = data[4];
        joy->l_haty = data[5];
        joy->r_hatx = data[6];
        joy->r_haty = data[7];
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


JoyStick& JoyStick::get_Instance(UART_HandleTypeDef *huart)
{
        static JoyStick sJoy(huart);
        
        return sJoy;
}

int JoyStick::init()
{
        gJoyStick.huart = huart_;

        // Null JoyStick Data
        gNull_JData.button1 = 0;
        gNull_JData.button2 = 0;
        gNull_JData.lt = 0;
        gNull_JData.rt = 0;
        gNull_JData.l_hatx = 0;
        gNull_JData.l_haty = 0;
        gNull_JData.r_hatx = 0;
        gNull_JData.r_haty = 0;
        
        // __HAL_UART_ENABLE_IT(huart, UART_IT_TC);
        HAL_UART_Receive_DMA(huart_, &gRx2Data, 1);

        return 0;
}

bool JoyStick::is_Empty()
{
        return gJoyStick.data.is_Empty();
}

JoyStick_Data JoyStick::read()
{
        if (!is_Empty()) {
                return gJoyStick.data.lookup();
        }
        return gNull_JData;
}

JoyStick_Command& JoyStick::parse()
{
        JoyStick_Data joydata = read();
        parse_JoyData(joydata);

        return Joy_Command;
}

void JoyStick::parse_JoyData(JoyStick_Data joy)
{
        int8_t rx = (int8_t)(joy.r_hatx);
        // int8_t ry = (int8_t)(joy.r_haty);
        // float vy = (float)(ry) / 128.0;

        int8_t rotate_dir = 0;
        if (rx > 120) {
                rotate_dir = 1;
        }
        else if (rx < -120) {
                rotate_dir = -1;
        }

        int lx = (int8_t)(joy.l_hatx);
        float vx = (float)(lx) / 128.0;

        if (fabsf(vx) < 0.05) {
                vx = 0;
        }

        int ly = (int8_t)(joy.l_haty);
        float vy = (float)(ly) / 128.0;

        if (fabsf(vy) < 0.05) {
                vy = 0;
        }

        float rw = 0;
        // if (!((fabsf(x) < 1e-3f) && (fabsf(y) < 1e-3f))) {
        //         rw = atan2f(y,x);
        // }

        Vec3<float> vels(vx, vy, rw);

        #ifdef _USE_BOARD_LEDS_FOR_JOYSTICK

                use_LEDs(vels, 0.128);

        #endif

        Control_Mode mode = Control_Mode::NONE;
        uint8_t button = joy.button1;
        bool reset_pos = button & _BV(RESET_KEY);
        bool grip_shagai = button & _BV(SHAGAI_GRIP_KEY);
        bool throw_shagai = button & _BV(THROW_SHAGAI_KEY);
        bool actuate_arm = button & _BV(ACTUATE_ARM_KEY);

        float manual_stroke = button & _BV(MANUAL_KEY);
        float auto_stroke = button & _BV(AUTO_KEY);

        if (manual_stroke && auto_stroke) {
                mode = Control_Mode::MANUAL;       // Manual Mode
        }
        else {
                if (manual_stroke && !auto_stroke) {
                        mode = Control_Mode::MANUAL;       // Manual Mode
                }
                else if (auto_stroke && !manual_stroke) {
                        mode = Control_Mode::AUTO;       // Automatic Mode
                }
                else {
                        mode = Control_Mode::NONE;
                }
        }

        uint8_t brake = joy.rt;
        uint8_t accel = joy.lt;

        button = joy.button2;
        bool start_throw = button & _BV(START_THROW_KEY);
        
        taskENTER_CRITICAL();
        Joy_Command.mode = mode;
        Joy_Command.reset_pos = reset_pos;
        Joy_Command.vels = vels;
        Joy_Command.brake = brake;
        Joy_Command.accel = accel;
        Joy_Command.grip_shagai = grip_shagai;
        Joy_Command.throw_shagai = throw_shagai;
        Joy_Command.actuate_arm = actuate_arm;
        Joy_Command.start_throw = start_throw;
        Joy_Command.rotate_dir = rotate_dir;
        taskEXIT_CRITICAL();
}
