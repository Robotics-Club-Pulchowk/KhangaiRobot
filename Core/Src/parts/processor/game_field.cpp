/*
 * game_field.cpp
 *
 * Created : 1/9/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "robo_states.h"

State_Vars gStateA_Data;
State_Vars gStateB_Data;
State_Vars gStateC_Data;
State_Vars gStateD_Data;
State_Vars gStateE_Data;
State_Vars gStateF_Data;
State_Vars gStateG_Data;
State_Vars gStateH_Data;
State_Vars gStateI_Data;

State_Vars gStateJ_Data;
State_Vars gStateK_Data;
State_Vars gStateL_Data;

// State_Vars gStateM_Data;
// State_Vars gStateN_Data;


const uint32_t gMax_Robo_Velocity = 1000;        // mm/s
const uint32_t gRated_Robo_Velocity = 100;

void init_GameField()
{
        // Defining the first state : State A
        gStateA_Data.id = Field::FIELD_A;
        gStateA_Data.centre = Vec2<float>(0, 0);
        gStateA_Data.upper_bounds = Vec2<float>(100, 100);
        gStateA_Data.lower_bounds = Vec2<float>(100, 100);
        gStateA_Data.last_limit = 0.9;
        gStateA_Data.ramping_factor = 0.005;
        gStateA_Data.first_limit = 0.33;
        gStateA_Data.max_vel = gMax_Robo_Velocity;
        gStateA_Data.rated_vel = gRated_Robo_Velocity;

        // Defining the state : State B
        gStateB_Data.id = Field::FIELD_B;
        gStateB_Data.centre = Vec2<float>(1800, 1500);
        gStateB_Data.upper_bounds = Vec2<float>(2000, 1700);
        gStateB_Data.lower_bounds = Vec2<float>(1660, 1300);
        gStateB_Data.last_limit = 1;
        gStateB_Data.ramping_factor = 0.001;
        gStateB_Data.first_limit = gStateA_Data.last_limit;
        gStateB_Data.max_vel = gMax_Robo_Velocity;
        gStateB_Data.rated_vel = gRated_Robo_Velocity;

        // Defining the state : State C
        gStateC_Data.id = Field::FIELD_C;
        gStateC_Data.centre = Vec2<float>(1750, 2400);
        gStateC_Data.upper_bounds = Vec2<float>(2000, 2700);
        gStateC_Data.lower_bounds = Vec2<float>(1500, 2000);
        gStateC_Data.last_limit = 1;
        gStateC_Data.ramping_factor = 0.001;
        gStateC_Data.first_limit = gStateB_Data.last_limit;
        gStateC_Data.max_vel = gMax_Robo_Velocity;
        gStateC_Data.rated_vel = gRated_Robo_Velocity;

        // Defining the state : State D
        gStateD_Data.id = Field::FIELD_D;
        gStateD_Data.centre = Vec2<float>(800, 3000);
        gStateD_Data.upper_bounds = Vec2<float>(1100, 3300);
        gStateD_Data.lower_bounds = Vec2<float>(500, 2700);
        gStateD_Data.last_limit = 1;
        gStateD_Data.ramping_factor = 0.001;
        gStateD_Data.first_limit = gStateC_Data.last_limit;
        gStateD_Data.max_vel = gMax_Robo_Velocity;
        gStateD_Data.rated_vel = gRated_Robo_Velocity;

        // Defining the state : State E
        gStateE_Data.id = Field::FIELD_E;
        gStateE_Data.centre = Vec2<float>(900, 3800);
        gStateE_Data.upper_bounds = Vec2<float>(1050, 4100);
        gStateE_Data.lower_bounds = Vec2<float>(750, 3200);
        gStateE_Data.last_limit = 1;
        gStateE_Data.ramping_factor = 0.001;
        gStateE_Data.first_limit = gStateD_Data.last_limit;
        gStateE_Data.max_vel = gMax_Robo_Velocity;
        gStateE_Data.rated_vel = gRated_Robo_Velocity;

        // Defining the state : State F
        gStateF_Data.id = Field::FIELD_F;
        gStateF_Data.centre = Vec2<float>(1800, 4100);
        gStateF_Data.upper_bounds = Vec2<float>(2100, 4300);
        gStateF_Data.lower_bounds = Vec2<float>(1620, 3800);
        gStateF_Data.last_limit = 1;
        gStateF_Data.ramping_factor = 0.001;
        gStateF_Data.first_limit = gStateE_Data.last_limit;
        gStateF_Data.max_vel = gMax_Robo_Velocity;
        gStateF_Data.rated_vel = gRated_Robo_Velocity;

        // Defining the state : State G
        gStateG_Data.id = Field::FIELD_G;
        gStateG_Data.centre = Vec2<float>(1750, 5500);
        gStateG_Data.upper_bounds = Vec2<float>(1875, 5650);
        gStateG_Data.lower_bounds = Vec2<float>(1625, 5200);
        gStateG_Data.last_limit = 1;
        gStateG_Data.ramping_factor = 0.001;
        gStateG_Data.first_limit = 1;
        gStateG_Data.max_vel = gMax_Robo_Velocity;
        gStateG_Data.rated_vel = gRated_Robo_Velocity;
        
        // Defining the state : State H
        gStateH_Data.id = Field::FIELD_H;
        gStateH_Data.centre = Vec2<float>(1250, 6300);
        gStateH_Data.upper_bounds = Vec2<float>(1400, 6450);
        gStateH_Data.lower_bounds = Vec2<float>(1025, 5800);
        gStateH_Data.last_limit = 1;
        gStateH_Data.ramping_factor = 0.001;
        gStateH_Data.first_limit = 1;
        gStateH_Data.max_vel = gMax_Robo_Velocity;
        gStateH_Data.rated_vel = gRated_Robo_Velocity;

        // Defining the state : State I
        gStateI_Data.id = Field::FIELD_I;
        gStateI_Data.centre = Vec2<float>(1300, 8000);
        gStateI_Data.upper_bounds = Vec2<float>(1400, 9000);
        gStateI_Data.lower_bounds = Vec2<float>(1050, 8030);
        gStateI_Data.last_limit = 0;
        gStateI_Data.ramping_factor = -0.1;
        gStateI_Data.first_limit = 0;
        gStateI_Data.max_vel = gMax_Robo_Velocity;
        gStateI_Data.rated_vel = gRated_Robo_Velocity;

        // Defining the state : State J
        gStateJ_Data.id = Field::FIELD_J;
        gStateJ_Data.centre = Vec2<float>(5500, 8400);
        gStateJ_Data.upper_bounds = Vec2<float>(6500, 9000);
        gStateJ_Data.lower_bounds = Vec2<float>(4300, 7800);
        gStateJ_Data.last_limit = 1;
        gStateJ_Data.ramping_factor = 0.001;
        gStateJ_Data.first_limit = 1;
        gStateJ_Data.max_vel = 800;
        gStateJ_Data.rated_vel = 90;
        
        // Defining the state : State K
        gStateK_Data.id = Field::FIELD_K;
        gStateK_Data.centre = Vec2<float>(5500, 7500);
        gStateK_Data.upper_bounds = Vec2<float>(0, 0);
        gStateK_Data.lower_bounds = Vec2<float>(0, 0);
        gStateK_Data.last_limit = 1;
        gStateK_Data.ramping_factor = 0.001;
        gStateK_Data.first_limit = 1;
        gStateK_Data.max_vel = 400;
        gStateK_Data.rated_vel = 40;

        // Defining the state : State L
        gStateL_Data.id = Field::FIELD_L;
        gStateL_Data.centre = Vec2<float>(6500, 8250);
        gStateL_Data.upper_bounds = Vec2<float>(0, 0);
        gStateL_Data.lower_bounds = Vec2<float>(0, 0);
        gStateL_Data.last_limit = 1;
        gStateL_Data.ramping_factor = 0.001;
        gStateL_Data.first_limit = 1;
        gStateL_Data.max_vel = 0;
        gStateL_Data.rated_vel = 0;
        
        // // Defining the state : State M
        // gStateL_Data.id = Field::FIELD_M;
        // gStateL_Data.centre = Vec2<float>(4000, 8250);
        // gStateL_Data.upper_bounds = Vec2<float>(4200, 8450);
        // gStateL_Data.lower_bounds = Vec2<float>(3700, 8050);
        // gStateL_Data.last_limit = 0;
        // gStateL_Data.ramping_factor = -0.001;
        // gStateL_Data.first_limit = 1;
        
        // // Defining the state : State N
        // gStateL_Data.id = Field::FIELD_N;
        // gStateL_Data.centre = Vec2<float>(4000, 7500);
        // gStateL_Data.upper_bounds = Vec2<float>(4200, 7300);
        // gStateL_Data.lower_bounds = Vec2<float>(3700, 7700);
        // gStateL_Data.last_limit = 0;
        // gStateL_Data.ramping_factor = -0.001;
        // gStateL_Data.first_limit = 1;
}
