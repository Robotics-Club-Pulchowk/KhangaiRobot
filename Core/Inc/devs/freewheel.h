/*
 * freewheel.h
 * 
 * Created : 1/4/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _FREEWHEEL_H_
#define _FREEWHEEL_H_

#include "gpio.h"

struct Enc
{
        GPIO_TypeDef *chB_port;
        uint16_t chB_pin;
        int32_t count;
        float radius;
        float ppr;
        char id;
};

void Enc_HandleCount(struct Enc *enc);
float Enc_get_DeltaDist(struct Enc *enc);

#endif // !_FREEWHEEL_H_
