/*
 * a4988.h
 * 
 * Created : 11/14/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _A4988_H_
#define _A4988_H_

#include "periphs/tim.h"
#include "periphs/gpio.h"

struct A4988
{
        uint32_t steps_per_rev;
        GPIO_TypeDef *dir_port, *step_port;
        uint32_t dir_pin, step_pin;
        float omega;
};

void A4988_setDirection(A4988 *stepper, uint32_t dir);
void A4988_toggleDirection(A4988 *stepper);

void A4988_step(A4988 *stepper);

#endif // !_A4988_H_
