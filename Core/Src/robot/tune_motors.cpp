/*
 * tune_motors.cpp
 *
 * Created : 12/31/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "pid_algorithms.h"
#include "pid.h"
#include <math.h>
#include "robot.h"

/**
 * @brief Function that calculates and sets new omegas for each wheel according
 *        to set points provided in order
 * @param set_points : An array of 4 elements. This array holds the angular speed
 *                    to be reached by each wheel in order
 * @param dt_millis : The time period at which this function is called periodically
 * @retval None
 * 
 * @note This function must be called periodically for it to actually work properly
 * 
 * <pre>
 * Tasks To Be Done By tune function
 * 1) Measure omega of each wheels
 * 2) Compute error
 * 3) Compute PID using available PID_Algorithm
 * 4) set Omega of each wheel
 * 5) update new omegas of all wheels at once
 * </pre>
 */
void Robot::tune_motors(float set_points[4], uint32_t dt_millis)
{
        float omega[4];
        float error[4];
        float voltage[4];
        float new_omega[4];
        PID *pid;
        float max_voltage;
        float max_omega;

        for (uint8_t i = 0; i < 4; ++i) {
                omega[i] = gWheels[i].get_Omega(dt_millis);
                error[i] = set_points[i] - omega[i];
                pid = gWheels[i].get_PIDController();
                voltage[i] = pid->compute_PID(error[i], dt_millis);

                // Max Omega corresponds to the max voltage value
                max_voltage = fabsf(pid->get_Algorithm()->get_Upper());
                max_omega = gWheels[i].get_MaxOmega();
                new_omega[i] = voltage[i] * max_omega / max_voltage;

                gWheels[i].set_Omega(new_omega[i]);
                // printf("(%ld, %ld)  ", (int32_t)(omega[i]*1000), (int32_t)(new_omega[i]*1000));
        }
        // printf("\n");

        // We don't want to delete the poninter since it was not us who allocated it
        pid = 0;

        for (uint8_t i = 0; i < 4; ++i) {
                gWheels[i].update();
        }
}

void Robot::ramp_down(uint32_t dt_millis)
{
        float omega[4];
        float error[4];
        float voltage[4];
        float new_omega[4];
        PID *pid;
        float max_voltage;
        float max_omega;

        for (uint8_t j = 0; j < 10; ++j) {
                
                for (uint8_t i = 0; i < 4; ++i) {
                        omega[i] = gWheels[i].get_Omega(dt_millis);
                        if (fabsf(omega[i]) < 5) {
                                gWheels[i].set_Omega(0);
                        }else {
                                pid = gWheels[i].get_PIDController();
                                error[i] = -(omega[i] / 2.0);
                                voltage[i] = pid->compute_PID(error[i], dt_millis);

                                // Max Omega corresponds to the max voltage value
                                max_voltage = fabsf(pid->get_Algorithm()->get_Upper());
                                max_omega = gWheels[i].get_MaxOmega();
                                new_omega[i] = voltage[i] * max_omega / max_voltage;

                                gWheels[i].set_Omega(new_omega[i]);
                        }
                }

                for (uint8_t i = 0; i < 4; ++i) {
                        gWheels[i].update();
                }
                HAL_Delay(10);
        }
}
