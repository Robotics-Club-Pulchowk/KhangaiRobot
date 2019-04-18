/*
 * processor.cpp
 *
 * Created : 1/9/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "processor.h"

extern State_Vars gStateA_Data;
extern State_Vars gStateB_Data;
extern State_Vars gStateC_Data;
extern State_Vars gStateD_Data;
extern State_Vars gStateE_Data;
extern State_Vars gStateF_Data;
extern State_Vars gStateG_Data;
extern State_Vars gStateH_Data;
extern State_Vars gStateI_Data;
extern State_Vars gStateJ_Data;
extern State_Vars gStateK_Data;
extern State_Vars gStateL_Data;
// extern State_Vars gStateM_Data;
// extern State_Vars gStateN_Data;

extern Robo_States gStateA;
extern Robo_States gStateB;
extern Robo_States gStateC;
extern Robo_States gStateD;
extern Robo_States gStateE;
extern Robo_States gStateF;
extern Robo_States gStateG;
extern Robo_States gStateH;
extern Robo_States gStateI;
extern Robo_States gStateJ;
extern Robo_States gStateK;
extern Robo_States gStateL;
// extern Robo_States gStateM;
// extern Robo_States gStateN;

Robo_States gStateA(&gStateA_Data, &gStateB);
Robo_States gStateB(&gStateB_Data, &gStateC);
Robo_States gStateC(&gStateC_Data, &gStateD);
Robo_States gStateD(&gStateD_Data, &gStateE);
Robo_States gStateE(&gStateE_Data, &gStateF);
Robo_States gStateF(&gStateF_Data, &gStateG);
Robo_States gStateG(&gStateG_Data, &gStateH);
Robo_States gStateH(&gStateH_Data, &gStateI);
Robo_States gStateI(&gStateI_Data, &gStateJ);

Robo_States gStateJ(&gStateJ_Data, &gStateK);
Robo_States gStateK(&gStateK_Data, &gStateL);
Robo_States gStateL(&gStateL_Data, &gStateL);

// Robo_States gStateM(&gStateM_Data, &gStateN);
// Robo_States gStateN(&gStateN_Data, &gStateN);

void init_GameField();

Processor& Processor::get_Instance(State_Sensor *sen)
{
        static Processor sRobo_CPU;
        
        init_GameField();

        sRobo_CPU.curr_state_ = &gStateJ;
        sRobo_CPU.sensor_ = sen;
        
        return sRobo_CPU;
}

int Processor::init(uint32_t dt_millis)
{
        sensor_->change_Sensors(curr_state_->get_ID());
        return 0;
}

Vec3<float> Processor::process(Vec3<float> state, Vec3<float> vel_from_base, State_Vars *&robot_state_vars_, uint32_t dt_millis)
{
        Vec3<float> vel(0, 0, 0);

        // This is the second algorithm used for moving the robot.
        // Algorithm Info:
        //      1) Minimum Acceleration Trajectory
        //      2) Smooth Transition

        uint8_t bounds = sensor_->get_Bounds();
        if (curr_state_->nextStateReached(state, bounds)) {
                update_State();
                sensor_->change_Sensors(curr_state_->get_ID());
        }
        
        // Get new velocity for the robot
        Vec2<float> v_polar = curr_state_->calc_Velocity(state, vel_from_base, dt_millis);
        float v = v_polar.getX();
        float theta = v_polar.getY();

        float phi = state.getZ();

        phi /= (float)57.3;     // to rads
        // Ensuring: -pi < phi <= pi
        // Since sin and cos gives value between -1 and 1
        phi = atan2(sin(phi), cos(phi));

        // ! Need to look here more
        float vx = v*sin(theta - phi);
        float vy = v*cos(theta - phi);
        float rw = (phi)*0.3;

                vel.set_Values(vx, vy, rw);

        robot_state_vars_ = curr_state_->get_State();

        return vel;
}

void Processor::update_State()
{
        curr_state_ = curr_state_->get_NextState();
}
