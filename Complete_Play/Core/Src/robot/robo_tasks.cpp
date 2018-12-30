/*
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Export Functions Used in C */
extern "C" void StartDefaultTask(void const *argument);
extern "C" void RobotThread(void const *argument);
extern "C" void LoggingThread(void const *argument);

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
        /* Init code for USB_DEVICE */
        MX_USB_DEVICE_Init();

        /* USER CODE BEGIN StartDefaultTask */
        /* Infinite loop */
        for (;;)
        {
                printf("Task : StartDefaultTask\n");
                osDelay(1000);
        }
        /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_RobotThread */
/**
 * @brief Function implementing the RoboSequence thread.
 * @param argument: Not used
 * @retval None
 * 
 * @note This thread is responsible for reading the current state of the robot.
 * 
 *
 * <pre>
 * Tasks performed by this thread :
 * 1) Read orientation given by the OrientationSensor based on the State the
 *    robot is in.
 * 2) Read position given by the PositionSensor based on the State the robot is
 *    in.
 * 3) Filter Orientation data.
 *      3.1) Filter roll
 *      3.2) Filter roll compensated pitch
 *      3.3) Filter tilt compensated pitch
 * 4) Rotate the measured position from body frame to the field frame
 * 5) Filter Position Data
 * 
 * 6) Find the robot's state based on it's current position
 * 7) Calculate the robot's angle of attack to reach the next state
 * 8) Calculate the robot's velocity according to it's state
 * 9) Calculate the correction angular velocity of the robot
 * 10) Calculate the omegas of each wheel
 * 
 * 11) Measure omega of each wheels
 * 12) Compute error
 * 13) Compute PID using available PID_Algorithm
 * 14) set Omega of each wheel
 * 15) update new omegas of all wheels at once
 * </pre>
 */
/* USER CODE END Header_RobotThread */
void RobotThread(void const *argument)
{
        /* USER CODE BEGIN RobotThread */
        uint32_t sample_period = 10;

        uint32_t dt = HAL_GetTick();
        uint32_t dt_tmp = HAL_GetTick();

        osDelay(sample_period);
        /* Infinite loop */
        for (;;)
        {
                // Since this is the highest priority task, we can be sure that
                // another task won't start when this task is running
                dt_tmp = HAL_GetTick();
                dt = dt_tmp - dt;
                // read_States(dt);
                // play_Game(dt);
                // tune_Motors(dt);
                dt = HAL_GetTick();
                dt_tmp = dt - dt_tmp;
                // Sleep for remaining time of the sampling period if there is
                // time left
                if (dt_tmp < sample_period) {
                        osDelay(sample_period - dt_tmp);
                }
        }
        /* USER CODE END RobotThread */
}

/* USER CODE BEGIN Header_LoggingThread */
/**
* @brief Function implementing the logging thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LoggingThread */
void LoggingThread(void const *argument)
{
        /* USER CODE BEGIN LoggingThread */
        /* Infinite loop */
        for (;;)
        {
                osDelay(1);
        }
        /* USER CODE END LoggingThread */
}
