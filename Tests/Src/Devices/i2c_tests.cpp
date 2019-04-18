/*
 * i2c_tests.cpp
 *
 * Created : 4/17/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "i2c.h"

//* This function prints all the available addresses in the specified I2C Bus.
void print_I2C_Addresses(I2C_HandleTypeDef * hi2c)
{
        uint8_t devs_found = 0;
        printf("Scanning...\n");
        for (uint8_t i = 0; i < 255; ++i) {
                if (HAL_I2C_IsDeviceReady(hi2c, i, 2, 10) == HAL_OK) {
                        ++devs_found;
                        printf("Address : 0x%X", i);

                        if (i == 0x3c) {
                                printf("Compass (HMC5883)");
                        }
                        else if (i == 0xD0) {
                                printf("Accel & Gyro (MPU6050)");
                        }
                        else if (i == 0xEE) {
                                printf("Barometer");
                        }

                        ++i;
                        printf("\n");
                }
                HAL_Delay(10);
        }
        if (devs_found) {
                printf("%d devices detected", devs_found);
        }
        else {
                printf("No devices found");
        }
        printf(" on given I2C Bus\n");
}