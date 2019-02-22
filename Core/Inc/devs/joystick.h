/*
 * joystick.h
 *
 * Created : 02/19/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _JOYSTICK_H_
#define _JOYSTICK_H_

#include "vec3.h"
#include "usart.h"
#include "queue_custom.h"

struct JoyStick_Data
{
        uint8_t button1;
        uint8_t lt;
        uint8_t rt;
        uint8_t l_hatx;
        uint8_t l_haty;
        uint8_t r_hatx;
        uint8_t r_haty;
};

struct JoyStick
{
        UART_HandleTypeDef *huart;
        Queue<JoyStick_Data, 2> data;
};

bool joy_Empty();
void init_JoyStick(UART_HandleTypeDef *huart);
JoyStick_Data read_JoyStick();
void JoyStick_Handle_RxCplt();

Vec3<float> parse_JoyStick();

#endif // !_JOYSTICK_H_
