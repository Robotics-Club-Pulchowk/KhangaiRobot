/*
 * robot.cpp
 *
 * Created : 12/31/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "robot.h"
#include "main.h"


// We should make sure that the robot ony have one instance and it is properly
// instantiated
Robot& Robot::get_Instance()
{
        static Robot gRobo_Instance;

        // Since all the parts follow singleton pattern, we should create them
        // when the robot is instantiated
        State_Sensor &stsn = State_Sensor::get_Instance();
        gRobo_Instance.sensor_ = &stsn;

        Actuator &act = Actuator::get_Instance();
        gRobo_Instance.base_ = &act;
        
        return gRobo_Instance;
}

int Robot::init()
{
        int base_status = base_->init();
        int sensor_status = sensor_->init();

        int status = (base_status | sensor_status);

        return status;
}

void Robot::run(uint32_t dt_millis)
{
        state_ = sensor_->read_State(state_from_base_);
        // TODO: Process state_
        state_from_base_ = base_->actuate(state_, dt_millis);
}
