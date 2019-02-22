/*
 * hmc5883.c
 * 
 * Created : 11/13/2018
 *  Author : n-is & ane
 *   email : 073bex421.nischal@pcampus.edu.np
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _HMC5883_H_
#define _HMC5883_H_

#include "i2c.h"
#include "vec3.h"
#include "a4988.h"

struct HMC5883
{
        I2C_HandleTypeDef *hi2c;
        Vec3<float> raw_axis;
        uint8_t address;
        Vec3<float> hard_iron_offset;
};

int8_t HMC5883_Init(struct HMC5883 *compass);
int8_t HMC5883_Read(struct HMC5883 *compass);

void HMC5883_Calibrate(struct HMC5883 *hmc, struct A4988 *stpr, uint32_t n);

#endif // !_HMC5883_H_
