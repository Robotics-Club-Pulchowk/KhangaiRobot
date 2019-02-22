
/*
 * devs_config.h
 * 
 * Created : 11/9/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _DEVS_CONFIG_H_
#define _DEVS_CONFIG_H_

#include "a4988.h"
#include "arduino.h"
#include "freewheel.h"
#include "hmc5883.h"
#include "mpu6050.h"

#include "encoder.h"
#include "lidar.h"

extern const uint32_t gMax_Robo_Velocity;

// These are the main IMU and the Compass Of the Robot
extern struct MPU6050 Body_IMU;
extern struct HMC5883 Body_HMC;
extern struct A4988 IMU_Stepper;

extern struct Enc gXEnc;
extern struct Enc gYEnc;

extern Encoder gXEncoder;
extern Encoder gYEncoder;
extern Lidar   gXLidar;


void IMU_Init();
void Stepper_Init();
void Encoders_Init();
void Lidars_Init();

#endif // !_DEVS_CONFIG_H_
