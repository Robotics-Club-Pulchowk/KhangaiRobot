/*
 * state_sensor.h
 *
 * Created : 1/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _STATE_SENSOR_H_
#define _STATE_SENSOR_H_

#include "vec3.h"

class State_Sensor final
{
public:
        State_Sensor(State_Sensor &&) = default;
        State_Sensor(const State_Sensor &) = default;
        State_Sensor &operator=(State_Sensor &&) = default;
        State_Sensor &operator=(const State_Sensor &) = default;
        ~State_Sensor() { }

        static State_Sensor& get_Instance();

        int init();
        Vec3<float> read_State(Vec3<float> base_state);

private:
        
        State_Sensor() { }
};

#endif // !_STATE_SENSOR_H_
