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

class Actuator final
{
public:
        Actuator(Actuator &&) = default;
        Actuator(const Actuator &) = default;
        Actuator &operator=(Actuator &&) = default;
        Actuator &operator=(const Actuator &) = default;
        ~Actuator() { }

        static Actuator& get_Instance();

        int init();
        Vec3<float> actuate(Vec3<float> vel, Vec3<float> psis, uint32_t dt_millis, int8_t test = 0);
        uint32_t stop(uint32_t dt_millis, float ramp_factor = 2.0, uint32_t max_time = 1000);
        void clear();

        void profile(Vec3<float> vel, uint32_t dt_millis);
        void check();

private:
        Wheel wheels_[4];
        PID* angle_pid_;

        Actuator() { }
        void wheels_Init();
        void pid_Init();
        void set_AnglePID(PID *pid) { angle_pid_ = pid; }
};

#endif // !_ACTUATOR_H_
