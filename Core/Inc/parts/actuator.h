/*
 * actuator.h
 *
 * Created : 1/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

#include "wheel.h"
#include "vec3.h"

class Actuator
{
public:
        Actuator(Actuator &&) = default;
        Actuator(const Actuator &) = default;
        Actuator &operator=(Actuator &&) = default;
        Actuator &operator=(const Actuator &) = default;
        ~Actuator() { }

        static Actuator& get_Instance();

        int init();
        Vec3<float> actuate(Vec3<float> vel, uint32_t dt_millis);
        uint32_t stop(uint32_t dt_millis, float ramp_factor = 2.0, uint32_t max_time = 1000);

private:
        Wheel wheels_[4];

        Actuator() { }
        void wheels_Init();
        void pid_Init();
};

#endif // !_ACTUATOR_H_
