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
#include "FreeRTOS.h"
#include "task.h"


// Button's bit position in the byte
#define B_X             (7)
#define B_Y             (6)
#define B_A             (5)
#define B_B             (4)
#define B_UP            (3)
#define B_DOWN          (2)
#define B_LB            (1)
#define B_RB            (0)

#define MANUAL_KEY              (B_LB)
#define AUTO_KEY                (B_A)
#define RESET_KEY               (B_RB)
#define SHAGAI_GRIP_KEY         (B_X)
#define THROW_SHAGAI_KEY        (B_Y)
#define ACTUATE_ARM_KEY         (B_B)

#ifndef _BV
        #define _BV(x)          (1 << x)
#endif

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

struct JoyStick_Handle
{
        UART_HandleTypeDef *huart;
        Queue<JoyStick_Data, 2> data;
};

enum class Control_Mode {
        MANUAL,
        AUTO,
        NONE
};

struct JoyStick_Command
{
        Control_Mode mode;
        bool reset_pos;
        Vec3<float> vels;
        uint8_t brake;
        uint8_t accel;
        bool grip_shagai;
        bool throw_shagai;
        bool actuate_arm;
};

class JoyStick
{
public:
        JoyStick(UART_HandleTypeDef *huart) {
                huart_ = huart;
        }
        JoyStick(JoyStick &&) = default;
        JoyStick(const JoyStick &) = default;
        JoyStick &operator=(JoyStick &&) = default;
        JoyStick &operator=(const JoyStick &) = default;
        ~JoyStick() { }

        static JoyStick& get_Instance(UART_HandleTypeDef *huart);
        int init();
        bool is_Empty();

        JoyStick_Command& parse();

private:
        UART_HandleTypeDef *huart_;
        struct JoyStick_Command Joy_Command;

        JoyStick_Data read();
        void parse_JoyData(JoyStick_Data joy);
};

void JoyStick_Handle_RxCplt();

#endif // !_JOYSTICK_H_
