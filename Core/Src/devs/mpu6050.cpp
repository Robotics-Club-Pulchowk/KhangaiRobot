/*
 * mpu6050.c
 * 
 * Created : 9/27/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

// TODO: Test code in this file in actual hardware.


#include "mpu6050.h"
#include "smoothing_algorithm.h"

static uint8_t i2c_buffer[8];

static const uint16_t lsb_per_g[4] = { 16384, 8192, 4096, 2048 };
static const float lsb_per_dsp[4] = { 131.0, 65.5, 32.8, 16.4 };

int8_t MPU6050_Init(struct MPU6050 *mpu)
{
        i2c_buffer[0] = PWR_MGMT_1;        // PWR_MGMT_1
        i2c_buffer[1] = (0x00);
        HAL_StatusTypeDef h1 = HAL_I2C_Master_Transmit(mpu->hi2c, mpu->address, i2c_buffer, 2, 10);
        
        i2c_buffer[0] = SMPLRT_DIV;        // SMPLRT_DIV
        i2c_buffer[1] = 0x19;
        HAL_StatusTypeDef h2 = HAL_I2C_Master_Transmit(mpu->hi2c, mpu->address, i2c_buffer, 2, 10);
        
        i2c_buffer[0] = CONFIG;            // CONFIG
        i2c_buffer[1] = 0x00;
        HAL_I2C_Master_Transmit(mpu->hi2c, mpu->address, i2c_buffer, 2, 10);

        i2c_buffer[0] = GYRO_CONFIG;
        i2c_buffer[1] = (mpu->g_scale) << 3;
        HAL_StatusTypeDef h3 = HAL_I2C_Master_Transmit(mpu->hi2c, mpu->address, i2c_buffer, 2, I2C_TIMEOUT);

        i2c_buffer[0] = ACCEL_CONFIG;
        i2c_buffer[1] = (mpu->a_scale) << 3;
        HAL_StatusTypeDef h4 = HAL_I2C_Master_Transmit(mpu->hi2c, mpu->address, i2c_buffer, 2, I2C_TIMEOUT);

        return (h1 | h2 | h3 | h4);
}

int8_t MPU6050_Read_RawAccel(struct MPU6050 *mpu)
{
        i2c_buffer[0] = ACCEL_XOUT_H;   // Starting Address for Accelerometer data
        HAL_StatusTypeDef h1 = HAL_I2C_Master_Transmit(mpu->hi2c, mpu->address, i2c_buffer, 1, I2C_TIMEOUT);
        HAL_StatusTypeDef h2 = HAL_I2C_Master_Receive(mpu->hi2c, mpu->address, i2c_buffer, 6, I2C_TIMEOUT);
        int16_t x = (i2c_buffer[0] << 8) | i2c_buffer[1];
        int16_t y = (i2c_buffer[2] << 8) | i2c_buffer[3];
        int16_t z = (i2c_buffer[4] << 8) | i2c_buffer[5];

        mpu->raw_a_axis.set_Values(x,y,z);

        return (h1 | h2);
}

int8_t MPU6050_Read_RawGyro(struct MPU6050 *mpu)
{
        i2c_buffer[0] = GYRO_XOUT_H;   // Starting Address for gyro data
        HAL_StatusTypeDef h1 = HAL_I2C_Master_Transmit(mpu->hi2c, mpu->address, i2c_buffer, 1, I2C_TIMEOUT);
        HAL_StatusTypeDef h2 = HAL_I2C_Master_Receive(mpu->hi2c, mpu->address, i2c_buffer, 6, I2C_TIMEOUT);
        int16_t x = (i2c_buffer[0] << 8) | i2c_buffer[1];
        int16_t y = (i2c_buffer[2] << 8) | i2c_buffer[3];
        int16_t z = (i2c_buffer[4] << 8) | i2c_buffer[5];
        
        mpu->raw_g_axis.set_Values(x,y,z);
        
        return (h1 | h2);
}

int8_t MPU6050_Read_Temp(struct MPU6050 *mpu)
{
        int16_t tmpr;
        i2c_buffer[0] = TEMP_OUT_H;   // Starting Address for temperature data
        HAL_StatusTypeDef h1 = HAL_I2C_Master_Transmit(mpu->hi2c, mpu->address, i2c_buffer, 1, I2C_TIMEOUT);
        HAL_StatusTypeDef h2 = HAL_I2C_Master_Receive(mpu->hi2c, mpu->address, i2c_buffer, 2, I2C_TIMEOUT);
        
        tmpr = (i2c_buffer[0] << 8) | i2c_buffer[1];
        mpu->tmpr = (tmpr / 340.0) + 36.53;
        
        return (h1 | h2);
}

int8_t MPU6050_Read_RawAxes(struct MPU6050 *mpu)
{
        int8_t h1 = MPU6050_Read_RawAccel(mpu);
        int8_t h2 = MPU6050_Read_RawGyro(mpu);
        
        return (h1 | h2);
}

int8_t MPU6050_Read_NormAccel(struct MPU6050 *mpu)
{
        int8_t h1 = MPU6050_Read_RawAccel(mpu);
        // (mpu->raw_a_axis).print();
        // printf("\n");

        float scale = (float)lsb_per_g[mpu->a_scale];
        int16_t x = (mpu->raw_a_axis).getX();
        int16_t y = (mpu->raw_a_axis).getY();
        int16_t z = (mpu->raw_a_axis).getZ();

        float xf = float(x)/(float)scale;
        float yf = float(y)/(float)scale;
        float zf = float(z)/(float)scale;

        mpu->norm_a_axis.set_Values(xf, yf, zf);
        
        return h1;
}


int8_t MPU6050_Read_NormGyro(struct MPU6050 *mpu)
{
        int8_t h1 = MPU6050_Read_RawGyro(mpu);
        float scale = (float)lsb_per_dsp[mpu->g_scale];
        int16_t x = (mpu->raw_g_axis).getX();
        int16_t y = (mpu->raw_g_axis).getY();
        int16_t z = (mpu->raw_g_axis).getZ();

        float xf = float(x)/scale;
        float yf = float(y)/scale;
        float zf = float(z)/scale;

        mpu->norm_g_axis.set_Values(xf, yf, zf);
        
        return h1;
}

int8_t MPU6050_Read_NormAxes(struct MPU6050 *mpu)
{
        int8_t h1 = MPU6050_Read_NormAccel(mpu);
        int8_t h2 = MPU6050_Read_NormGyro(mpu);
        
        return (h1 | h2);
}

Vec3<float> MPU6050_Calc_OmegaBias(struct MPU6050 *mpu, uint32_t n)
{
        Vec3<float> bias;
        Vec3<float> gyro;
        float item_num;
        
        Exp_Smooth x_alpha35(0.35);
        Exp_Smooth y_alpha35(0.35);
        Exp_Smooth z_alpha35(0.35);

        float gx,gy,gz;

        for (uint32_t i = 0; i < n; ++i) {
                MPU6050_Read_NormGyro(mpu);
                gyro = mpu->norm_g_axis;

                gx = x_alpha35.smooth(gyro.getX());
                gy = y_alpha35.smooth(gyro.getY());
                gz = z_alpha35.smooth(gyro.getZ());

                gyro.set_Values(gx, gy, gz);

                item_num = 1.0 / (float)(i+1);
                bias += (gyro - bias).mult_EW(item_num);

                HAL_Delay(1);
        }
        return bias;
}
