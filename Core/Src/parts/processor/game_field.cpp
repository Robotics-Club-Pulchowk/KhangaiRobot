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

        // Defining the state : State B
        gStateB_Data.id = Field::FIELD_B;
        gStateB_Data.centre = Vec2<float>(1800, 1500);
        gStateB_Data.upper_bounds = Vec2<float>(2000, 1700);
        gStateB_Data.lower_bounds = Vec2<float>(1660, 1300);
        gStateB_Data.last_limit = 1;
        gStateB_Data.ramping_factor = 0.001;
        gStateB_Data.first_limit = gStateA_Data.last_limit;

        // Defining the state : State C
        gStateC_Data.id = Field::FIELD_C;
        gStateC_Data.centre = Vec2<float>(1750, 2400);
        gStateC_Data.upper_bounds = Vec2<float>(2000, 2700);
        gStateC_Data.lower_bounds = Vec2<float>(1500, 2200);
        gStateC_Data.last_limit = 1;
        gStateC_Data.ramping_factor = 0.001;
        gStateC_Data.first_limit = gStateB_Data.last_limit;

        // Defining the state : State D
        gStateD_Data.id = Field::FIELD_D;
        gStateD_Data.centre = Vec2<float>(700, 2900);
        gStateD_Data.upper_bounds = Vec2<float>(1000, 3100);
        gStateD_Data.lower_bounds = Vec2<float>(500, 2700);
        gStateD_Data.last_limit = 1;
        gStateD_Data.ramping_factor = 0.001;
        gStateD_Data.first_limit = gStateC_Data.last_limit;

        // Defining the state : State E
        gStateE_Data.id = Field::FIELD_E;
        gStateE_Data.centre = Vec2<float>(900, 3700);
        gStateE_Data.upper_bounds = Vec2<float>(1050, 4100);
        gStateE_Data.lower_bounds = Vec2<float>(750, 3400);
        gStateE_Data.last_limit = 1;
        gStateE_Data.ramping_factor = 0.001;
        gStateE_Data.first_limit = gStateD_Data.last_limit;

        // Defining the state : State F
        gStateF_Data.id = Field::FIELD_F;
        gStateF_Data.centre = Vec2<float>(1800, 4100);
        gStateF_Data.upper_bounds = Vec2<float>(2100, 4300);
        gStateF_Data.lower_bounds = Vec2<float>(1620, 3800);
        gStateF_Data.last_limit = 1;
        gStateF_Data.ramping_factor = 0.001;
        gStateF_Data.first_limit = gStateE_Data.last_limit;

        // Defining the state : State G
        gStateG_Data.id = Field::FIELD_G;
        gStateG_Data.centre = Vec2<float>(1750, 5500);
        gStateG_Data.upper_bounds = Vec2<float>(1875, 5650);
        gStateG_Data.lower_bounds = Vec2<float>(1625, 5200);
        gStateG_Data.last_limit = 1;
        gStateG_Data.ramping_factor = 0.001;
        gStateG_Data.first_limit = 1;
        
        // Defining the state : State H
        gStateH_Data.id = Field::FIELD_H;
        gStateH_Data.centre = Vec2<float>(1200, 6300);
        gStateH_Data.upper_bounds = Vec2<float>(1400, 6450);
        gStateH_Data.lower_bounds = Vec2<float>(1025, 5800);
        gStateH_Data.last_limit = 1;
        gStateH_Data.ramping_factor = 0.001;
        gStateH_Data.first_limit = 1;

        // Defining the state : State I
        gStateI_Data.id = Field::FIELD_I;
        gStateI_Data.centre = Vec2<float>(1300, 8000);
        gStateI_Data.upper_bounds = Vec2<float>(1400, 9000);
        gStateI_Data.lower_bounds = Vec2<float>(1050, 8000);
        gStateI_Data.last_limit = 0;
        gStateI_Data.ramping_factor = -0.1;
        gStateI_Data.first_limit = 0;

        // Defining the state : State J
        gStateJ_Data.id = Field::FIELD_J;
        gStateJ_Data.centre = Vec2<float>(5000, 8700);
        gStateJ_Data.upper_bounds = Vec2<float>(6000, 9000);
        gStateJ_Data.lower_bounds = Vec2<float>(4700, 8500);
        gStateJ_Data.last_limit = 1;
        gStateJ_Data.ramping_factor = 0.001;
        gStateJ_Data.first_limit = 1;
        
        // Defining the state : State K
        gStateK_Data.id = Field::FIELD_K;
        gStateK_Data.centre = Vec2<float>(4000, 8700);
        gStateK_Data.upper_bounds = Vec2<float>(4500, 9000);
        gStateK_Data.lower_bounds = Vec2<float>(3500, 8500);
        gStateK_Data.last_limit = 1;
        gStateK_Data.ramping_factor = 0.001;
        gStateK_Data.first_limit = 1;
        
        // Defining the state : State L
        gStateL_Data.id = Field::FIELD_L;
        gStateL_Data.centre = Vec2<float>(4000, 5000);
        gStateL_Data.upper_bounds = Vec2<float>(4200, 5300);
        gStateL_Data.lower_bounds = Vec2<float>(3700, 4700);
        gStateL_Data.last_limit = 0;
        gStateL_Data.ramping_factor = -0.001;
        gStateL_Data.first_limit = 1;
}
