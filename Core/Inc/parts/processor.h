/*
 * processor.h
 *
 * Created : 1/9/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _PROCESSOR_H_
#define _PROCESSOR_H_

#include "state_sensor.h"
#include "robo_states.h"
#include "joystick.h"

class Processor
{
public:
        Processor(Processor &&) = default;
        Processor(const Processor &) = default;
        Processor &operator=(Processor &&) = default;
        Processor &operator=(const Processor &) = default;
        ~Processor() { }

        static Processor& get_Instance(State_Sensor *sen);
        
        int init(uint32_t dt_millis);
        Vec3<float> control(Vec3<float> state,
                            Vec3<float> vel_from_base,
                            Vec3<float> last_vel,
                            State_Vars *&robot_state_vars,
                            uint32_t dt_millis);

private:
        Robo_States *curr_state_;
        State_Sensor *sensor_;
        JoyStick *joy_stick_;
        bool is_first_;
        Vec3<float> first_state_;

        Processor() { is_first_ = true; }
        void process(Vec3<float> state, State_Vars *&robot_state_vars);
        Vec3<float> auto_control(Vec3<float> state, Vec3<float> vel_from_base, uint32_t dt_millis);
        Vec3<float> manual_control(JoyStick_Command& joy_cmd);
        void reset_Position(State_Vars *&robot_state_vars);
        void throw_Shagai(bool throw_shagai);
        void extend_Arm();

        void update_State(uint8_t bounds);
};

#endif // !_PROCESSOR_H_
