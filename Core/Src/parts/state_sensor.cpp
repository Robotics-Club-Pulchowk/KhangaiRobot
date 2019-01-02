/*
 * state_sensor.cpp
 *
 * Created : 1/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "state_sensor.h"


State_Sensor& State_Sensor::get_Instance()
{
        static State_Sensor gState_Sensor_Instance;

        return gState_Sensor_Instance;
}


/**
 * @brief Function that initializes all the required components for the robot's
 *        State Sensor(x, y and yaw)
 * @retval 0 if inititialized properly
 * 
 * 
 * <pre>
 * Tasks performed by this function
 * 1) Call the devices initializations in correct order
 * 2) Call the software initializations for utilities like smoothies and filters
 * </pre>
 */
//! Not yet implemented
int State_Sensor::init()
{
        return 0;
}


/**
 * @brief Function that provides the state of the robot
 * @param base_state This parameter is considered as one of the possible state
 *                   of robot. This will be passed to both Orientation_Sensor
 *                   and the Position_Sensor for further use.
 * @retval state of the robot
 * 
 * 
 * <pre>
 * Tasks performed by this function
 * 1) Read Orientation of the robot with Orientation_Sensor
 * 2) Read Position of the robot with Position_Sensor
 * </pre>
 */
//! Not yet implemented
Vec3<float> State_Sensor::read_State(Vec3<float> base_state)
{
        Vec3<float> state;
        return state;
}
