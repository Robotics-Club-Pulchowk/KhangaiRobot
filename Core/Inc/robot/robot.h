/*
 * robot.h
 *
 * Created : 12/31/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "wheel.h"


class Robot final
{
public:
        Robot(Robot &&) = default;
        Robot(const Robot &) = default;
        Robot &operator=(Robot &&) = default;
        Robot &operator=(const Robot &) = default;

        ~Robot() { }
        static Robot& get_Instance();

        int init();
        void tune_motors(float set_points[4], uint32_t dt_millis);
        void ramp_down(uint32_t dt_millis);

private:
        Wheel gWheels[4];       //< Our robot has exactly four wheels

        Robot() { }
        void wheels_Init();
        void pid_Init();
};

#endif // !_ROBOT_H_
