/*
 * robo_init.c
 *
 * Created : 10/1/2018
 */

#include "robo_init.h"
#include "pid_algorithms.h"
#include "pid.h"

static Wheel_Config gWheel_Configurations[4];
Wheel gWheels[4];

static Discrete_PID gDisc_PID[4];
static PID gPID[4];

static float gI_Factor = 2;

static void wheel_init(void);
static void pid_Init();

/**
 * @brief Function that initializes all the required components for the robot
 *        to run
 * @retval None
 * 
 * Tasks performed by this function
 * 1) Call the devices initializations in correct order
 * 2) Call the software initializations for utilities like pid and filters
 * 3) Starts the timers to run them in respective configured modes like PWM
 *    mode and the Encoder mode
 */
void robo_Init()
{
        wheel_init();

        HAL_TIM_Base_Start(&htim8);

        HAL_TIM_PWM_Start(gWheel_Configurations[0].htim, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(gWheel_Configurations[1].htim, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(gWheel_Configurations[2].htim, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(gWheel_Configurations[3].htim, TIM_CHANNEL_4);

        HAL_TIM_Encoder_Start(gWheel_Configurations[0].henc, TIM_CHANNEL_ALL);
        HAL_TIM_Encoder_Start(gWheel_Configurations[1].henc, TIM_CHANNEL_ALL);
        HAL_TIM_Encoder_Start(gWheel_Configurations[2].henc, TIM_CHANNEL_ALL);
        HAL_TIM_Encoder_Start(gWheel_Configurations[3].henc, TIM_CHANNEL_ALL);
}


/**
 * @brief Function that initializes all the required components for the wheels
 *        of the robot
 * @retval None
 * 
 * Tasks performed by this function
 * 1) Assigns appropriate IDs to each wheel
 * 2) Give each wheel the respective radius
 * 3) Assign a timer and a channel to each wheel for it's motor to run in PWM
 *    mode. A timer is shared among all the available wheels since we have
 *    four wheels.
 * 4) Assigns the ppr of each wheel's encoder
 * 5) Assigns the direction pins for each motor's direction
 * 6) Assigns timer for each wheel's encoder
 */
static void wheel_init(void)
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
                gWheels[i].set_Config(&gWheel_Configurations[i]);
        }
        // Initializes PID parameters for all wheels
        pid_Init();
}

/**
 * @brief Function that initializes all the required components for the wheel's
 *        pid controller
 * @retval None
 * 
 * Tasks performed by this function
 * 1) Assigns gains for PID controller of each wheel
 * 2) Assigns separate PID controllers for each wheel
 */
static void pid_Init()
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
                gWheels[i].set_PIDController(&gPID[i]);
        }
}
