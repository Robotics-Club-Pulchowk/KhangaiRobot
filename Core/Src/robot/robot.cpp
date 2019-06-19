/*
 * robot.cpp
 *
 * Created : 12/31/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "robot.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "joystick.h"
#include "array.h"
#include "devs_config.h"

#include "logger.h"
#include "utils.h"

#include "defines.h"

extern State_Vars gStateA_Data;
extern State_Vars gStateO_Data;

void update_GameField(GameField field);

// We should make sure that the robot ony have one instance and it is properly
// instantiated
Robot& Robot::get_Instance()
{
        static Robot sRobo_Instance;

        // Since all the parts follow singleton pattern, we should create them
        // when the robot is instantiated
        State_Sensor &stsn = State_Sensor::get_Instance();
        sRobo_Instance.sensor_ = &stsn;

        Processor &proc = Processor::get_Instance(sRobo_Instance.sensor_);
        sRobo_Instance.cpu_ = &proc;

        Actuator &act = Actuator::get_Instance();
        sRobo_Instance.base_ = &act;
        
        return sRobo_Instance;
}

void Robot::read_Field()
{
        // Read Field Value Here
        gCurrent_Field = GameField::BLUE;

        // Update Gamefield after reading current field
        update_GameField(gCurrent_Field);
}

int Robot::init(uint32_t dt_millis)
{
        // Initialize base as it's initialization doesn't depend upon any other
        // parts
        int base_status = base_->init();
        int sensor_status = sensor_->init(dt_millis);
        int cpu_status = cpu_->init(dt_millis);

        //* Calibrate the compass after all the devices have been initialized

        int status = (base_status | sensor_status | cpu_status);
        
        HAL_GPIO_WritePin(B_GreenLED_GPIO_Port, B_GreenLED_Pin, GPIO_PIN_SET);
        HAL_Delay(1000);
        HAL_GPIO_WritePin(B_GreenLED_GPIO_Port, B_GreenLED_Pin, GPIO_PIN_RESET);

        if (HAL_GPIO_ReadPin(B_PushButton_GPIO_Port, B_PushButton_Pin) == GPIO_PIN_SET) {
                HAL_GPIO_WritePin(B_RedLED_GPIO_Port, B_RedLED_Pin, GPIO_PIN_SET);

                HAL_Delay(500);
                Vec3<float> zero_vec(0,0,0);
                struct HMC5883 *hmc = &Body_HMC;
                
                Vec3<float> maxs, mins;
                HMC5883_Read(hmc);
                maxs = hmc->raw_axis;
                mins = hmc->raw_axis;

                uint32_t sample_period = 10;
                uint32_t i = 0;
                uint32_t total_counts = 1000;
                uint32_t curr_time = HAL_GetTick();
                while(1) {
                        if (i >= total_counts) {
                                break;
                        }
                        if (HAL_GetTick() - curr_time > sample_period) {
                                curr_time = HAL_GetTick();
                                ++i;
                                if ( i <= total_counts/2 - 25) {
                                        base_->actuate(zero_vec, zero_vec, sample_period, 1);
                                }
                                else if ( i < total_counts/2 + 15) {
                                        base_->actuate(zero_vec, zero_vec, sample_period, 0);
                                }
                                else if ( i < total_counts) {
                                        base_->actuate(zero_vec, zero_vec, sample_period, -1);
                                }
                                else {
                                        base_->actuate(zero_vec, zero_vec, sample_period, 0);
                                }
                                
                                HMC5883_Read(hmc);
                                maxs.setX(max_val(maxs.getX(), hmc->raw_axis.getX()));
                                maxs.setY(max_val(maxs.getY(), hmc->raw_axis.getY()));
                                maxs.setZ(max_val(maxs.getZ(), hmc->raw_axis.getZ()));
                                
                                mins.setX(min_val(mins.getX(), hmc->raw_axis.getX()));
                                mins.setY(min_val(mins.getY(), hmc->raw_axis.getY()));
                                mins.setZ(min_val(mins.getZ(), hmc->raw_axis.getZ()));

                        }
                }
                base_->actuate(zero_vec, zero_vec, sample_period, 0);

                hmc->hard_iron_offset = (maxs + mins).mult_EW(0.5);
                
                log_CompassOffsets(hmc->hard_iron_offset);
                printf("Compass offsets : ");
                (hmc->hard_iron_offset).print();
                printf("\n");
                base_->clear();
                HAL_GPIO_WritePin(B_RedLED_GPIO_Port, B_RedLED_Pin, GPIO_PIN_RESET);
        }

        // Start from field A
        robot_state_vars_ = &gStateA_Data;
        velocities_.set_Values(0,0,0);

        initiated_ = true;

        return status;
}

// This function is called by the RobotThread
void Robot::update(uint32_t dt_millis)
{
        state_ = sensor_->read_State(state_from_base_,robot_state_vars_, dt_millis);

        Vec3<float> vels = cpu_->control(state_,
                                         state_from_base_,
                                         velocities_,
                                         robot_state_vars_,
                                         dt_millis);

        state_.print();
        printf("\n");

        // Reflection about Y-axis
        // vels.setX(-vels.getX());
        // vels.setY(-vels.getY());

        taskENTER_CRITICAL();
        velocities_ = vels;
        psis_.set_Values(vels.getZ(), state_.getZ(), 0);
        taskEXIT_CRITICAL();
}

// This function is called by the MotorThread
void Robot::run(uint32_t dt_millis)
{
        velocities_.setZ(0);
        // (velocities_.mult_EW(1000)).print();
        // printf("\n");
        state_from_base_ = base_->actuate(velocities_, psis_, dt_millis, 0);
}

bool Robot::is_Initiated() const
{
        return initiated_;
}

void Robot::profile_Actuators(Vec3<float> vel, uint32_t dt_millis)
{
        base_->profile(vel, dt_millis);
}

void Robot::check_Actuators()
{
        base_->check();
}