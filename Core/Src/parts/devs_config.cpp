/*
 * devs_config.cpp
 * 
 * Created : 11/9/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

/**
 * @file  : devs_config.cpp
 * @brief : All the hardware specific configurations of all devices are done in
 *          this file, except for the wheels(Vietnamese Motors and Vietnamese 
 *          Motor Encoder), which is done in actuator.cpp file.
 */

#include "devs_config.h"

struct MPU6050 Body_IMU;
struct HMC5883 Body_HMC;
struct A4988 IMU_Stepper;

struct Enc gXEnc;
struct Enc gYEnc;

Arduino_Device gXLidar_Dev(0x02, 2);
Arduino_Device gYLidar_Dev(0x03, 2);
Arduino_Device gLED_Strip(0x01, 1);


Encoder gXEncoder(&gXEnc, SensorName::XEncoder);
Encoder gYEncoder(&gYEnc, SensorName::YEncoder);
Lidar   gXLidar(&gXLidar_Dev, SensorName::XLidar);
Lidar   gYLidar(&gYLidar_Dev, SensorName::YLidar);

void IMU_Init()
{
        Body_IMU.hi2c = &hi2c1;
        Body_IMU.address = 0xD0;
        Body_IMU.a_scale = Accel_Scale::SCALE_2G;
        Body_IMU.g_scale = Gyro_Scale::SCALE_250;

        Body_HMC.hi2c = &hi2c1;
        Body_HMC.address = 0x3C;
        Body_HMC.hard_iron_offset.set_Values(13.65, -62.21, -199.25);

        MPU6050_Init(&Body_IMU);
        HMC5883_Init(&Body_HMC);
}

void Stepper_Init()
{
        IMU_Stepper.steps_per_rev = 200;
        IMU_Stepper.dir_port = GPIOD;
        IMU_Stepper.dir_pin = GPIO_PIN_10;
        IMU_Stepper.step_port = GPIOB;
        IMU_Stepper.step_pin = GPIO_PIN_14;

        A4988_setDirection(&IMU_Stepper, 0);
}


void Encoders_Init()
{
        gXEnc.id = 'x';
        gXEnc.chB_port = GPIOC;
        gXEnc.chB_pin = GPIO_PIN_5;
        gXEnc.count = 0;
        gXEnc.radius = 29.5;
        gXEnc.ppr = 200;

        gYEnc.id = 'y';
        gYEnc.chB_port = GPIOC;
        gYEnc.chB_pin = GPIO_PIN_4;
        gYEnc.count = 0;
        gYEnc.radius = 29.5;
        gYEnc.ppr = 200;

        gXEncoder.init();
        gYEncoder.init();
}

void Lidars_Init()
{
        gXLidar.set_Outliers(Vec2<float>(12000, 10));
        gYLidar.set_Outliers(Vec2<float>(12000, 10));

        gXLidar.init();
        gYLidar.init();
}
