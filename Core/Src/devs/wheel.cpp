/*
 * wheel.c
 *
 * Created : 10/1/2018
 */

#include "wheel.h"
#include <math.h>

static uint16_t time_period(uint16_t PWM_frequency)
{
        if (PWM_frequency < 2000) { //MIN PWM_FREQUENCY = 1281.738 for time period to be 16 bit
                _Error_Handler(__FILE__, __LINE__);
        }
        return ((PWM_TIMER_FREQUENCY / 2) / PWM_frequency) - 1; // In Center aligned mode period doubles hence we divide by 2
}

/* SET DUTYCYCLE primary function */
static void set_DutyCycle_Primary(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t dutyCycle)
{
        uint16_t mapped_value;
        mapped_value = (time_period(MOTOR_DRIVER_FREQUENCY) * dutyCycle) / 65535;
        __HAL_TIM_SET_COMPARE(htim, Channel, mapped_value);
}

static void set_DutyCycle(Wheel_Config *w, uint16_t dutyCycle)
{
        set_DutyCycle_Primary(w->htim, w->channel, dutyCycle);
}

static void set_WheelDirection(Wheel_Config *w, Direction d)
{
        if (d == Direction::AHEAD)       // AHEAD
        {
                HAL_GPIO_WritePin(w->in1_port, w->in1_pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(w->in2_port, w->in2_pin, GPIO_PIN_RESET);
        }
        else if (d == Direction::BACK)  // BACK
        {
                HAL_GPIO_WritePin(w->in1_port, w->in1_pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(w->in2_port, w->in2_pin, GPIO_PIN_SET);
        }
        else                    // HALT
        {
                HAL_GPIO_WritePin(w->in1_port, w->in1_pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(w->in2_port, w->in2_pin, GPIO_PIN_RESET);
        }
}

static void set_WheelOmega(Wheel_Config *w, float omega)
{
        uint16_t new_omega = (65535.0 / w->max_omega) * (omega);
        set_DutyCycle(w, new_omega);
}

void Wheel::start_Periphs()
{
        HAL_TIM_Base_Start(wheel_->htim);
        
        HAL_TIM_PWM_Start(wheel_->htim, wheel_->channel);
        HAL_TIM_Encoder_Start(wheel_->henc, TIM_CHANNEL_ALL);
}

void Wheel::set_Omega(float omega)
{
        if (omega < -wheel_->tolerance) {
                dir_ = Direction::BACK;
        }
        else if (omega > wheel_->tolerance) {
                dir_ = Direction::AHEAD;
        }
        else {
                dir_ = Direction::HALT;
                omega_ = 0;
        }
        omega_ = fabsf(omega);
}

void Wheel::update() const
{
        if (omega_ < 0) {
                _Error_Handler(__FILE__, __LINE__);
        }
        set_WheelDirection(wheel_, dir_);
        set_WheelOmega(wheel_, omega_);
}

// Calculating Omega Ratio When Encoder is used in TI12 mode
inline float omegaRatioTI12(uint32_t ppr)
{
        return (2.0 * M_PI / (4.0 * ppr));
}

float Wheel::get_Omega(uint32_t dt_millis)
{
        float dt = (float)dt_millis / 1000.0;
        // Get count and reset the counter
        int16_t counts = wheel_->henc->Instance->CNT;
        wheel_->henc->Instance->CNT = 0;

        // w = 2*pi*f
        // f = (counts / ppr) / dt;
        float omega = omegaRatioTI12(wheel_->enc_ppr) * counts / dt;

        return omega;
}