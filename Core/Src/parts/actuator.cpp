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

#include "logger.h"

#define WHEEL_RADIUS    (0.0675)


// We should make sure that the Actuator ony have one instance and it is properly
// instantiated
Actuator& Actuator::get_Instance()
{
        static Actuator sBase_Instance;

        return sBase_Instance;
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
 * 2) Call the software initializations for utilities like pid for wheels
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

const float gInverse_Coupling_Array[3][4] = {{ 0.25,  0.25, -0.25, -0.25 },
                                             { 0.25, -0.25, -0.25,  0.25 },
                                             { 0.25,  0.25,  0.25,  0.25 }};

static const Mat gInverse_Coupling_Matrix(gInverse_Coupling_Array);
/**
 * @brief Function that actuate the robot's base(omni-base)
 * @param vel : A vector that holds vx, vy and omega
 * @param psis : A vector that holds psi_target, psi and psi_dot
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

Vec3<float> Actuator::actuate(Vec3<float> vel, Vec3<float> psis, uint32_t dt_millis, int8_t test)
{
        static uint32_t gMotor_On_Period = HAL_GetTick();
        uint32_t current_time = HAL_GetTick();


        float rw = 0;
        float t_psi = 0;
        float psi = 0;

        if (test > 0) {
                rw = 0.5;
        }
        else if (test < 0) {
                rw = -0.5;
        }
        else {
                if (current_time - gMotor_On_Period < 4000) {
                        vel.set_Values(0, 0.4, 0);
                        t_psi = M_PI / 2.0;
                }
                else if (current_time - gMotor_On_Period < 6000) {
                        vel.set_Values(0, 0, 0);
                        t_psi = M_PI / 2.0;
                }
                else if (current_time - gMotor_On_Period < 10000) {
                        vel.set_Values(0, -0.4, 0);
                        t_psi = 0;
                }
                else {
                        gMotor_On_Period = HAL_GetTick();
                }
        

                // Calculate rw
                psi = psis.getY() / 57.3;
                float err_psi = t_psi - psi;
                rw = -angle_pid_->compute_PID(err_psi, dt_millis);
        }

        // printf("%ld, %ld\n", (int32_t)(rw*1000), (int32_t)(psi*1000));

        // vel.set_Values(0,0,0);

        vel.set_Values(0, 0, rw);

        log_Angle(psi, rw);

        // psi /= 57.3;

        // float Rot[3][3] = { { cos(psi), -sin(psi), 0 },
        //                     { sin(psi),  cos(psi), 0 },
        //                     {        0,         0, 1} };

        // Mat Rm(Rot);
        // Mat v = Rm * vel;

        Mat wheels_omegas = gCoupling_Matrix * vel;
        // w = v / r
        float set_points[4] = { wheels_omegas.at(0,0) / (float)(WHEEL_RADIUS),
                                wheels_omegas.at(1,0) / (float)(WHEEL_RADIUS),
                                wheels_omegas.at(2,0) / (float)(WHEEL_RADIUS),
                                wheels_omegas.at(3,0) / (float)(WHEEL_RADIUS) };

        for (uint8_t i = 0; i < 4; ++i) {
                if (set_points[i] > 15) {
                        set_points[i] = 15;
                }
                else if (set_points[i] < -15) {
                        set_points[i] = -15;
                }
        }

        // This is motor tuning part
        // Omega is created as 2D array with single column so that we can easily
        // convert it to matrix form later on
        float omega[4];
        float vels[4][1];
        float error[4];
        float voltage[4];
        float new_omega[4];
        PID *pid;
        float max_voltage;
        float max_omega;

        // printf("%ld   ", HAL_GetTick());
        for (uint8_t i = 0; i < 4; ++i) {
                // if (i != 2)
                //         set_points[i] = 0;

                omega[i] = wheels_[i].get_Omega(dt_millis);
                error[i] = set_points[i] - omega[i];
                pid = wheels_[i].get_PIDController();
                // The controller's output is voltage
                voltage[i] = pid->compute_PID(error[i], dt_millis);

                // Max Omega corresponds to the max voltage value
                max_voltage = fabsf(pid->get_Algorithm()->get_Upper());
                max_omega = wheels_[i].get_MaxOmega();
                // Controller's output voltage is converted to the corresponding
                // omega according to linear relation since we will just be
                // output-ting voltage and this is just a abstraction of the
                // motor driver
                new_omega[i] = voltage[i] * max_omega / max_voltage;

                wheels_[i].set_Omega(new_omega[i]);
                // printf("%ld   %ld   %ld   ", (int32_t)(set_points[i]*1000), (int32_t)(omega[i]*1000), (int32_t)(new_omega[i]*1000));
                // wheels_[i].log(omega[i], new_omega[i]);

                vels[i][0] = omega[i] * (float)(WHEEL_RADIUS);
        }
        // printf("\n");

        // We don't want to delete the poninter since it was not us who allocated it
        pid = 0;

        for (uint8_t i = 0; i < 4; ++i) {
                wheels_[i].update();
        }

        // We can measure the velocity of the robot by this method
        // We can also use this for determining wheel's slippage
        // !yet to implement
        Mat measured = gInverse_Coupling_Matrix * Mat(vels);

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

void Actuator::clear()
{
        for (uint8_t i = 0; i < 4; ++i) {
                wheels_[i].clear();
        }
        angle_pid_->clear();
}


static Wheel_Config gWheel_Configurations[4];
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
                gWheel_Configurations[i].id = i + 1;
                gWheel_Configurations[i].radius = 0.0675;
                // All motors are connected to same timer : TIM8
                gWheel_Configurations[i].htim = &htim8;
                //! Need to make sure the following value is correct
                gWheel_Configurations[i].enc_ppr = 249.6;
        }
        gWheel_Configurations[0].in2_port = GPIOD;
        gWheel_Configurations[0].in2_pin = GPIO_PIN_0;
        gWheel_Configurations[0].in1_port = GPIOE;
        gWheel_Configurations[0].in1_pin = GPIO_PIN_5; 
        gWheel_Configurations[0].channel = TIM_CHANNEL_1;
        gWheel_Configurations[0].henc = &htim4;
        gWheel_Configurations[0].max_omega = 65.19;

        gWheel_Configurations[1].in2_port = GPIOA;
        gWheel_Configurations[1].in2_pin = GPIO_PIN_10;
        gWheel_Configurations[1].in1_port = GPIOB;
        gWheel_Configurations[1].in1_pin = GPIO_PIN_5;
        gWheel_Configurations[1].channel = TIM_CHANNEL_2;
        gWheel_Configurations[1].henc = &htim2;
        gWheel_Configurations[1].max_omega = 63.86;
        
        gWheel_Configurations[2].in2_port = GPIOE;
        gWheel_Configurations[2].in2_pin = GPIO_PIN_3;
        gWheel_Configurations[2].in1_port = GPIOD;
        gWheel_Configurations[2].in1_pin = GPIO_PIN_2;
        gWheel_Configurations[2].channel = TIM_CHANNEL_3;
        gWheel_Configurations[2].henc = &htim3;
        gWheel_Configurations[2].max_omega = 71.507;

        gWheel_Configurations[3].in2_port = GPIOC;
        gWheel_Configurations[3].in2_pin = GPIO_PIN_13;
        gWheel_Configurations[3].in1_port = GPIOE;
        gWheel_Configurations[3].in1_pin = GPIO_PIN_1;
        gWheel_Configurations[3].channel = TIM_CHANNEL_4;
        gWheel_Configurations[3].henc = &htim1;
        gWheel_Configurations[3].max_omega = 68.041;

        for (uint8_t i = 0; i < 4; ++i) {
                wheels_[i].set_Config(&gWheel_Configurations[i]);
                wheels_[i].start_Periphs();
        }
}


static Discrete_PID gDisc_PID[4];
static PID gPID[4];

// static float gI_Factor = 3.125;
// static float gP_Factor = 1;

//* Robot's ANgle Control parameters
static Discrete_PID gAng_PID;
static PID gRobo_PID;
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
        gDisc_PID[0].set_PID(0.2732, 1.662, 0.0005221);
        gDisc_PID[0].set_Limits(24, -24);
        gDisc_PID[1].set_PID(0.2789, 2.644, 0.000342);
        gDisc_PID[1].set_Limits(24, -24);
        gDisc_PID[2].set_PID(0.2491, 1.458, 0.0004947);
        gDisc_PID[2].set_Limits(24, -24);
        gDisc_PID[3].set_PID(0.2618, 2.347, 0.0003395);
        gDisc_PID[3].set_Limits(24, -24);

        gAng_PID.set_PID(0.2, 0, 0);
        gAng_PID.set_Limits(5, -5);
        gRobo_PID.set_Algorithm(&gAng_PID);
        set_AnglePID(&gRobo_PID);

        for (uint8_t i = 0; i < 4; ++i) {
                gPID[i].set_Algorithm(&gDisc_PID[i]);
                wheels_[i].set_PIDController(&gPID[i]);
        }
}

void Actuator::profile(Vec3<float> vel, uint32_t dt_millis)
{
        // Takes a total of about 5 seconds to profile actuator
        printf("Actuation Command : ");
        vel.print();
        printf("\n");

        Vec3<float> v;

        uint32_t n = (uint32_t)(4000.0 / (float)dt_millis);

        uint32_t curr_time = HAL_GetTick();
        for (uint32_t i = 0; i < n; ++i) {
                if (HAL_GetTick() - curr_time > dt_millis) {
                        curr_time = HAL_GetTick();

                        printf("Actuated : ");
                        // v = actuate(vel, dt_millis);
                        v = v.mult_EW(1000);
                        v.print();
                        printf("\n");
                }
                
        }

        stop(dt_millis, 2);
}

void Actuator::check()
{
        for (uint8_t i = 0; i < 4; ++i) {
                wheels_[i].set_Omega(4*i);
                wheels_[i].update();
        }
        uint32_t curr_time = HAL_GetTick();
        uint32_t sample_time = curr_time;
        while (HAL_GetTick() - curr_time < 5000) {
                if (HAL_GetTick() - sample_time > 10) {
                        sample_time = HAL_GetTick();
                        for (uint8_t i = 0; i < 4; ++i) {
                                printf("%d  ", (int16_t)(wheels_[i].get_Omega(10)*1000));
                        }
                        printf("\n");
                }
        }
        for (uint8_t i = 0; i < 4; ++i) {
                wheels_[i].set_Omega(0);
                wheels_[i].update();
        }
        HAL_Delay(500);
        for (uint8_t i = 0; i < 4; ++i) {
                wheels_[i].set_Omega(-10);
                wheels_[i].update();
        }
        
        sample_time = HAL_GetTick();
        curr_time = sample_time;
        while (HAL_GetTick() - curr_time < 5000) {
                if (HAL_GetTick() - sample_time > 10) {
                        sample_time = HAL_GetTick();
                        for (uint8_t i = 0; i < 4; ++i) {
                                printf("%d  ", (int16_t)(wheels_[i].get_Omega(10)*1000));
                        }
                        printf("\n");
                }
        }
        
        for (uint8_t i = 0; i < 4; ++i) {
                wheels_[i].set_Omega(0);
                wheels_[i].update();
        }
        HAL_Delay(500);
}

// *** EOF ***
