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
#include "vec3.h"

#include "actuator.h"
#include "processor.h"
#include "state_sensor.h"

/**
 * @class Robot
 * @brief A Singleton class that handles all the robot's sequence
 * 
 * Robot class is responsible for handling all the sequence to be taken by the
 * <b>Khangai Robot</b> in the game field.
 * <pre>
 * Tasks that can be delegated to the Robot class :
 * 1) Read the robot's state, i.e, the position and orientation of the robot
 *    using the available OrientationSensor and PositionSensor.
 * 2) Determine the next action to be taken by the robot.
 * 3) Give motor commands and tune the motors.
 * 4) Stop the robot using the appropriate ramping.
 * </pre>
 */
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
        void run(uint32_t dt_millis);

private:
        Actuator *base_;
        Vec3<float> state_;

        Robot() { }
};

#endif // !_ROBOT_H_
