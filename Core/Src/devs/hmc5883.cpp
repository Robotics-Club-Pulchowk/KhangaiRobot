/*
 * hmc5883.c
 * 
 * Created : 11/13/2018
 *  Author : n-is & ane
 *   email : 073bex421.nischal@pcampus.edu.np
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "hmc5883.h"
#include "mpu6050.h"

#include "utils.h"

static uint8_t buffer[8];

int8_t HMC5883_Init(struct HMC5883 *hmc)
{        
	buffer[0] = 0x00;
	buffer[1] = 0x78;       // Sampling Number, Output Rate
	HAL_StatusTypeDef h1 = HAL_I2C_Master_Transmit(hmc->hi2c, hmc->address, buffer, 2, I2C_TIMEOUT);
	
	buffer[0] = 0x01;
	buffer[1] = 0x60;       // Gain
	HAL_StatusTypeDef h2 = HAL_I2C_Master_Transmit(hmc->hi2c, hmc->address, buffer, 2, I2C_TIMEOUT);
	
	buffer[0] = 0x02;
	buffer[1] = 0x00;       // Measurement Mode
	HAL_StatusTypeDef h3 = HAL_I2C_Master_Transmit(hmc->hi2c, hmc->address, buffer, 2, I2C_TIMEOUT);

        return (h1 | h2 | h3);
}

int8_t HMC5883_Read(struct HMC5883 *hmc)
{        
        buffer[0] = 0x03;
        HAL_StatusTypeDef h1 = HAL_I2C_Master_Transmit(hmc->hi2c, hmc->address, buffer, 1, I2C_TIMEOUT);
        HAL_StatusTypeDef h2 = HAL_I2C_Master_Receive(hmc->hi2c, hmc->address, &buffer[1], 6, I2C_TIMEOUT);
        
        int16_t x = (buffer[1]<<8 | buffer[2]);
        int16_t y = (buffer[5]<<8 | buffer[6]);		
        int16_t z = (buffer[3]<<8 | buffer[4]);

        float bx = (float)x - hmc->hard_iron_offset.getX();
        float by = (float)y - hmc->hard_iron_offset.getY();
        float bz = (float)z - hmc->hard_iron_offset.getZ();

        (hmc->raw_axis).set_Values((float)bx,(float)by,(float)bz);

        return (h1 | h2);
}

void HMC5883_Calibrate(struct HMC5883 *hmc, struct A4988 *stpr, uint32_t n)
{
        Vec3<float> maxs, mins;
        printf("Start Rotating Compass\n");
        HMC5883_Read(hmc);
        maxs = hmc->raw_axis;
        mins = hmc->raw_axis;
        HAL_Delay(6);
        for (uint32_t i = 1; i < n; ++i) {
                HMC5883_Read(hmc);
                maxs.setX(max_val(maxs.getX(), hmc->raw_axis.getX()));
                maxs.setY(max_val(maxs.getY(), hmc->raw_axis.getY()));
                maxs.setZ(max_val(maxs.getZ(), hmc->raw_axis.getZ()));
                
                mins.setX(min_val(mins.getX(), hmc->raw_axis.getX()));
                mins.setY(min_val(mins.getY(), hmc->raw_axis.getY()));
                mins.setZ(min_val(mins.getZ(), hmc->raw_axis.getZ()));

                A4988_step(stpr);
                HAL_Delay(4);
        }

        HAL_Delay(50);
        A4988_toggleDirection(stpr);
        for (uint32_t i = 0; i < (n); ++i) {
                A4988_step(stpr);
                // for (uint32_t j = 0; j < 30000; ++j) {
                //         asm volatile("nop");
                // }
                HAL_Delay(1);
        }
        A4988_toggleDirection(stpr);

        printf("Stop Rotating Compass\n");

        hmc->hard_iron_offset = (maxs + mins).mult_EW(0.5);

        (hmc->hard_iron_offset).print();
        printf("\n");
}
