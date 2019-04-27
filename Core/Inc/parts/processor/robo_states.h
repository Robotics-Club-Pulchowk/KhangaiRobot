/*
 * robo_states.h
 *
 * Created : 1/9/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _ROBO_STATES_H_
#define _ROBO_STATES_H_

#include "state_sensor.h"
#include "vec2.h"
#include "vec4.h"

enum class Field
{
        FIELD_A,
        FIELD_B,
        FIELD_C,
        FIELD_D,
        FIELD_E,
        FIELD_F,
        FIELD_G,
        FIELD_H,
        FIELD_I,
        FIELD_J,
        FIELD_K,
        FIELD_L,

        FIELD_O,
        FIELD_P,
        FIELD_Q,
        FIELD_Q1,
        FIELD_Q2,
        FIELD_R1,
        FIELD_R2,
        FIELD_R
};

struct State_Vars
{
        Field id;

        Vec2<float> centre;
        Vec2<float> upper_bounds;
        Vec2<float> lower_bounds;

        float last_limit;
        float ramping_factor;
        float first_limit;

        float max_vel;
        float rated_vel;
};

class Robo_States
{
public:
        Robo_States(State_Vars *sv, Robo_States *next);
        Robo_States(Robo_States &&) = default;

        Robo_States(const Robo_States &) = default;
        Robo_States &operator=(Robo_States &&) = default;
        Robo_States &operator=(const Robo_States &) = default;
        ~Robo_States() { }

        float calc_RoboVelocity(Vec3<float> state, uint32_t dt_millis);
        float calc_AngleOfAttack(Vec3<float> state, float v, uint32_t dt_millis);
        Vec2<float> calc_Velocity(Vec3<float> state, Vec3<float> vel_from_base, uint32_t dt_millis);
        bool nextStateReached(Vec3<float> state, uint8_t bounds);

        Robo_States* get_NextState() { return next_state_; }
        Field get_ID() { return sv_->id; }
        State_Vars* get_State() { return sv_; }
        void set_State(Robo_States *robo) {
                sv_ = robo->get_State();
                next_state_ = robo->get_NextState();
        }
        
private:
        State_Vars *sv_;
        Robo_States *next_state_;

        float ramped_;

        Vec4<float> cubicSpline(Vec2<float> curr, Vec2<float> nex, Vec2<float> nexnex);
        float quadTheta(Vec3<float> state, float v, uint32_t dt_millis);
        float linearTheta(Vec3<float> state, float v, uint32_t dt_millis);
};

#endif // !_ROBO_STATES_H_
