/*
 * wheel.h
 *
 * Created : 10/1/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _WHEEL_H_
#define _WHEEL_H_

#include "stm32f4xx_hal.h"
#include "tim.h"

#include "defines.h"

#include "pid_algorithms.h"
#include "pid.h"

#include "logger.h"

enum Direction
{
        AHEAD,
        BACK,
        HALT
};

struct Wheel_Config
{
        uint8_t id;
        float radius;

        GPIO_TypeDef *in1_port;
        uint16_t in1_pin;
        GPIO_TypeDef *in2_port;
        uint16_t in2_pin;

        TIM_HandleTypeDef *htim;        // PWM Generating Timer
        uint32_t channel;               // PWM Channel

        TIM_HandleTypeDef *henc;
        
        float tolerance;                // Tolerance level for zero
        float max_omega;                // Max Omega Attained by the wheel(motor)
        uint32_t enc_ppr;

        PID *pid_controller;            // Pointer to the wheel's PID controller
};

class Wheel
{
public:
        Wheel() { wheel_ = 0; }
        Wheel(Wheel_Config *wheel) { wheel_ = wheel; }
        Wheel(Wheel &&) = default;
        Wheel(const Wheel &) = default;
        Wheel &operator=(Wheel &&) = default;
        Wheel &operator=(const Wheel &) = default;
        ~Wheel() { }
        
        void set_Config(Wheel_Config *wheel) { wheel_ = wheel; }
        void set_Direction(enum Direction d) { dir_ = d; }
        void set_Omega(float omega);
        void update() const;

        float get_Omega(uint32_t dt_millis);
        float get_MaxOmega() { return wheel_->max_omega; }

        void start_Periphs();

        void set_PIDController(PID *pid) { wheel_->pid_controller = pid; }
        PID * get_PIDController() {
                return wheel_->pid_controller;
        }

        void log(float omega, float new_omega) {
                // Motor Packet is of the following form
                // START_BYTE  MOTOR_PACKET_ID  MOTOR_ID SCALED_OMEGA  SCALED_NEW_OMEGA  TIME_HIGH_BYTE  TIME_LOW_BYTE
                // A total of 7 bytes including start byte
                // uint32_t curr_time = HAL_GetTick();

                gLogging_Buffer.insert((uint8_t)(START_BYTE));
                gLogging_Buffer.insert((uint8_t)(MOTOR_PACKET_ID));
                gLogging_Buffer.insert((uint8_t)(wheel_->id));
                float omga = (omega / (float)(MAX_OMEGA)) * 127.0;
                gLogging_Buffer.insert((uint8_t)(omga));
                float n_omga = (new_omega / (float)(MAX_OMEGA)) * 127.0;
                gLogging_Buffer.insert((uint8_t)(n_omga));

                // uint8_t time_h = (uint8_t)(curr_time >> (8+4));
                // uint8_t time_l = (uint8_t)(curr_time >> 4);
                // gLogging_Buffer.insert(time_h);
                // gLogging_Buffer.insert(time_l);

                // printf("%d, %d, %d, %d\n", (int8_t)(omga), (int8_t)(n_omga), time_h, time_l);
        }
private:
       Wheel_Config *wheel_;
       enum Direction dir_;
       float omega_;    // This value should always be positive
};


#endif // _WHEEL_H_
