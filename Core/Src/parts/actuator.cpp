/*
 * actuator.cpp
 *
 * Created : 1/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include <math.h>

#include "actuator.h"
#include "pid_algorithms.h"
#include "pid.h"
#include "mat.h"

static Wheel_Config gWheel_Configurations[4];

static Discrete_PID gDisc_PID[4];
static PID gPID[4];

static float gI_Factor = 2;

// We should make sure that the Actuator ony have one instance and it is properly
// instantiated
Actuator& Actuator::get_Instance()
{
        static Actuator gBase_Instance;

        return gBase_Instance;
}


/**
 * @brief Function that initializes all the required components for the robot's
 *        actuator(omni-base)
 * @retval None
 * 
 * 
 * <pre>
 * Tasks performed by this function
 * 1) Call the devices initializations in correct order
 * 2) Call the software initializations for utilities like pid and filters
 * </pre>
 */
int Actuator::init()
{
        // Initialize all wheels of the robot
        wheels_Init();
        // Initializes PID parameters for all wheels
        pid_Init();

        return 0;
}


const float gCoupling_Array[4][3] = {{ 1,  1, 1 },
                                     { 1, -1, 1 },
                                     {-1, -1, 1 },
                                     {-1,  1, 1 }};

static const Mat gCoupling_Matrix(gCoupling_Array);

const float gInverse_Coupling_Array[3][4] = {{ 0.0,  0.5, 0.5, 0.0 },
                                             { 0.5, -0.5, 0.0, 0.0 },
                                             { 0.5,  0.0, 0.5, 0.0 }};

static const Mat gInverse_Coupling_Matrix(gInverse_Coupling_Array);
/**
 * @brief Function that actuate the robot's base(omni-base)
 * @param vel : An vector that holds vx, vy and omega
 * @param dt_millis : The time period at which this function is called periodically
 * @retval A vector that holds vx, vy and omega of the base in the last frame
 * 
 * 
 * <pre>
 * Tasks performed by this function
 * 1) Calculates omegas of each wheel to reach the given velocities using the
 *    Coupling Matrix
 * 2) Measure omega of each wheels
 * 3) Compute error
 * 4) Compute PID using available PID_Algorithm
 * 5) set Omega of each wheel
 * 6) update new omegas of all wheels at once
 * 7) Calculates vx, vy and rw of the base from the measured omegas of each
 *    wheel using the inverse Coupling Matrix
 * </pre>
 */
Vec3<float> Actuator::actuate(Vec3<float> vel, uint32_t dt_millis)
{
        Mat wheels_omegas = gCoupling_Matrix * vel;
        float set_points[4] = { wheels_omegas.at(0,0),
                                wheels_omegas.at(1,0),
                                wheels_omegas.at(2,0),
                                wheels_omegas.at(3,0) };

        // This is motor tuning part
        float omega[4][1];
        float error[4];
        float voltage[4];
        float new_omega[4];
        PID *pid;
        float max_voltage;
        float max_omega;

        for (uint8_t i = 0; i < 4; ++i) {
                omega[i][0] = wheels_[i].get_Omega(dt_millis);
                error[i] = set_points[i] - omega[i][0];
                pid = wheels_[i].get_PIDController();
                voltage[i] = pid->compute_PID(error[i], dt_millis);

                // Max Omega corresponds to the max voltage value
                max_voltage = fabsf(pid->get_Algorithm()->get_Upper());
                max_omega = wheels_[i].get_MaxOmega();
                new_omega[i] = voltage[i] * max_omega / max_voltage;

                wheels_[i].set_Omega(new_omega[i]);
                // printf("(%ld, %ld)  ", (int32_t)(omega[i][0]*1000), (int32_t)(new_omega[i]*1000));
        }
        // printf("\n");

        // We don't want to delete the poninter since it was not us who allocated it
        pid = 0;

        for (uint8_t i = 0; i < 4; ++i) {
                wheels_[i].update();
        }

        // We can measure the velocity of the robot by this method
        // We can also use this for wheel's slippage
        // !yet to implement
        Mat measured = gInverse_Coupling_Matrix * Mat(omega);

        Vec3<float> last_vel(measured.at(0,0),
                             measured.at(1,0),
                             measured.at(2,0));
        return last_vel;
}


uint32_t Actuator::stop(uint32_t dt_millis, float ramp_factor, uint32_t max_time)
{
        float omega[4];
        float error[4];
        float voltage[4];
        float new_omega[4];
        PID *pid;
        float max_voltage;
        float max_omega;

        bool break_loop = false;

        uint32_t tick_start = HAL_GetTick();
        uint32_t tick_end;

        for (uint8_t j = 0; j < 10; ++j) {
                
                for (uint8_t i = 0; i < 4; ++i) {
                        omega[i] = wheels_[i].get_Omega(dt_millis);

                        if (fabsf(omega[i]) < 5) {
                                wheels_[i].set_Omega(0);
                        }else {
                                pid = wheels_[i].get_PIDController();
                                error[i] = -(omega[i] / ramp_factor);
                                voltage[i] = pid->compute_PID(error[i], dt_millis);

                                // Max Omega corresponds to the max voltage value
                                max_voltage = fabsf(pid->get_Algorithm()->get_Upper());
                                max_omega = wheels_[i].get_MaxOmega();
                                new_omega[i] = voltage[i] * max_omega / max_voltage;

                                wheels_[i].set_Omega(new_omega[i]);
                        }

                        if (HAL_GetTick() - tick_start > max_time) {
                                wheels_[i].set_Omega(0);
                                break_loop = true;
                        }
                }
                pid = 0;

                for (uint8_t i = 0; i < 4; ++i) {
                        wheels_[i].update();
                }
                
                if (break_loop) {
                        break;
                }
                HAL_Delay(10);
        }
        tick_end = HAL_GetTick();
        return (tick_end - tick_start);
}


/**
 * @brief Function that initializes all the required components for the wheels
 *        of the robot
 * @retval None
 * 
 * 
 * <pre>
 * Tasks performed by this function
 * 1) Assigns appropriate IDs to each wheel
 * 2) Give each wheel the respective radius
 * 3) Assign a timer and a channel to each wheel for it's motor to run in PWM
 *    mode. A timer is shared among all the available wheels since we have
 *    four wheels.
 * 4) Assigns the ppr of each wheel's encoder
 * 5) Assigns the direction pins for each motor's direction
 * 6) Assigns timer for each wheel's encoder
 * 7) Starts the timers to run each wheel in respective configured modes like PWM
 *    mode and the Encoder mode
 * </pre>
 */
void Actuator::wheels_Init(void)
{
        int i;
        for (i = 0; i < 4; i++)
        {
                gWheel_Configurations[i].id = i;
                gWheel_Configurations[i].radius = 0.067;
                // All motors are connected to same timer : TIM8
                gWheel_Configurations[i].htim = &htim8;
                //! Need to re-calculate this value
                gWheel_Configurations[i].max_omega = 70;
                //! Need to make sure the following value is correct
                gWheel_Configurations[i].enc_ppr = 249.6;
        }
        gWheel_Configurations[0].in1_port = GPIOD;
        gWheel_Configurations[0].in1_pin = GPIO_PIN_3; 
        gWheel_Configurations[0].in2_port = GPIOD;
        gWheel_Configurations[0].in2_pin = GPIO_PIN_1;
        gWheel_Configurations[0].channel = TIM_CHANNEL_1;
        gWheel_Configurations[0].henc = &htim2;

        gWheel_Configurations[1].in1_port = GPIOD;
        gWheel_Configurations[1].in1_pin = GPIO_PIN_14;
        gWheel_Configurations[1].in2_port = GPIOD;
        gWheel_Configurations[1].in2_pin = GPIO_PIN_5;
        gWheel_Configurations[1].channel = TIM_CHANNEL_2;
        gWheel_Configurations[1].henc = &htim3;
        
        gWheel_Configurations[2].in1_port = GPIOD;
        gWheel_Configurations[2].in1_pin = GPIO_PIN_7;
        gWheel_Configurations[2].in2_port = GPIOD;
        gWheel_Configurations[2].in2_pin = GPIO_PIN_15;
        gWheel_Configurations[2].channel = TIM_CHANNEL_3;
        gWheel_Configurations[2].henc = &htim1;

        gWheel_Configurations[3].in1_port = GPIOC;
        gWheel_Configurations[3].in1_pin = GPIO_PIN_12;
        gWheel_Configurations[3].in2_port = GPIOC;
        gWheel_Configurations[3].in2_pin = GPIO_PIN_10;
        gWheel_Configurations[3].channel = TIM_CHANNEL_4;
        gWheel_Configurations[3].henc = &htim4;

        for (uint8_t i = 0; i < 4; ++i) {
                wheels_[i].set_Config(&gWheel_Configurations[i]);
                wheels_[i].start_Periphs();
        }
}

/**
 * @brief Function that initializes all the required components for the wheel's
 *        pid controller
 * @retval None
 * 
 * 
 * <pre>
 * Tasks performed by this function
 * 1) Assigns gains for PID controller of each wheel
 * 2) Assigns separate PID controllers for each wheel
 * </pre>
 */
void Actuator::pid_Init()
{
        gDisc_PID[0].set_PID(0.6336, gI_Factor*10.05, 0);
        gDisc_PID[0].set_Limits(24, -24);
        gDisc_PID[1].set_PID(0.5262, gI_Factor*9.528, 0);
        gDisc_PID[1].set_Limits(24, -24);
        gDisc_PID[2].set_PID(0.7336, gI_Factor*10.05, 0);
        gDisc_PID[2].set_Limits(24, -24);
        gDisc_PID[3].set_PID(0.49, gI_Factor*7.615, 0);
        gDisc_PID[3].set_Limits(24, -24);

        for (uint8_t i = 0; i < 4; ++i) {
                gPID[i].set_Algorithm(&gDisc_PID[i]);
                wheels_[i].set_PIDController(&gPID[i]);
        }
}