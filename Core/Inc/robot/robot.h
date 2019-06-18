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

        void read_Field();
        int init(uint32_t dt_millis);
        void update(uint32_t dt_millis);
        void run(uint32_t dt_millis);
        bool is_Initiated() const;

        void profile_Actuators(Vec3<float> vel, uint32_t dt_millis);
        void check_Actuators();

private:
        State_Sensor *sensor_;
        Processor *cpu_;
        Actuator *base_;

        // Robot class makes sure that this variable is not modified by any other
        // entity except the Processor
        State_Vars *robot_state_vars_;

        Vec3<float> state_;
        Vec3<float> state_from_base_;
        Vec3<float> velocities_;
        Vec3<float> psis_;

        bool initiated_;
        Robot() {
                initiated_ = false;
        }
};
  
#endif // !_ROBOT_H_
