/*
 * game_field.cpp
 *
 * Created : 1/9/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "robo_states.h"
#include "defines.h"

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

State_Vars gStateO_Data;
State_Vars gStateP_Data;
State_Vars gStateQ_Data;
State_Vars gStateQ1_Data;
State_Vars gStateQ2_Data;
State_Vars gStateR_Data;
State_Vars gStateR1_Data;
State_Vars gStateR2_Data;
State_Vars gStateS_Data;
State_Vars gStateT_Data;


const uint32_t gMax_Robo_Velocity = 1666;        // mm/s
const uint32_t gRated_Robo_Velocity = 150;

uint32_t gMax_Robo_Manual_Velocity = 1800;        // mm/s

float gAuto_Ratio = 0.6;

static void init_RedGameField();
static void init_BlueGameField();

void init_GameField()
{
        init_RedGameField();
}

void update_GameField(GameField field)
{
        if (field == GameField::RED) {
                init_RedGameField();
        }
        else if (field == GameField::BLUE) {
                init_BlueGameField();
        }
}

static void init_RedGameField()
{
        // *** Red Game Field *** //
        // Defining the first state : State A
        gStateA_Data.id = Field::FIELD_A;
        gStateA_Data.centre = Vec2<float>(0, 0);
        gStateA_Data.upper_bounds = Vec2<float>(100, 100);
        gStateA_Data.lower_bounds = Vec2<float>(100, 100);
        gStateA_Data.last_limit = 1;
        gStateA_Data.ramping_factor = 0.01;
        gStateA_Data.first_limit = 0.6;
        gStateA_Data.max_vel = 1400;
        gStateA_Data.rated_vel = 140;
        gStateA_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State B
        gStateB_Data.id = Field::FIELD_B;
        gStateB_Data.centre = Vec2<float>(1500, 1500);
        gStateB_Data.upper_bounds = Vec2<float>(2500, 2000);
        gStateB_Data.lower_bounds = Vec2<float>(1200, 1000);
        gStateB_Data.last_limit = 1;
        gStateB_Data.ramping_factor = 0.001;
        gStateB_Data.first_limit = gStateA_Data.last_limit;
        gStateB_Data.max_vel = gMax_Robo_Velocity;
        gStateB_Data.rated_vel = gRated_Robo_Velocity;
        gStateB_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State C
        gStateC_Data.id = Field::FIELD_C;
        gStateC_Data.centre = Vec2<float>(1600, 2400);
        gStateC_Data.upper_bounds = Vec2<float>(2000, 2700);
        gStateC_Data.lower_bounds = Vec2<float>(1500, 1800);
        gStateC_Data.last_limit = 1;
        gStateC_Data.ramping_factor = 0.001;
        gStateC_Data.first_limit = gStateB_Data.last_limit;
        gStateC_Data.max_vel = gMax_Robo_Velocity;
        gStateC_Data.rated_vel = 120;
        gStateC_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State D
        gStateD_Data.id = Field::FIELD_D;
        gStateD_Data.centre = Vec2<float>(800, 3000);
        gStateD_Data.upper_bounds = Vec2<float>(1115, 3300);
        gStateD_Data.lower_bounds = Vec2<float>(500, 2700);
        gStateD_Data.last_limit = 1;
        gStateD_Data.ramping_factor = 0.001;
        gStateD_Data.first_limit = gStateC_Data.last_limit;
        gStateD_Data.max_vel = gMax_Robo_Velocity;
        gStateD_Data.rated_vel = gRated_Robo_Velocity;
        gStateD_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State E
        gStateE_Data.id = Field::FIELD_E;
        gStateE_Data.centre = Vec2<float>(550, 4000);
        gStateE_Data.upper_bounds = Vec2<float>(1050, 4100);
        gStateE_Data.lower_bounds = Vec2<float>(500, 3500);
        gStateE_Data.last_limit = 1;
        gStateE_Data.ramping_factor = 0.001;
        gStateE_Data.first_limit = gStateD_Data.last_limit;
        gStateE_Data.max_vel = gMax_Robo_Velocity;
        gStateE_Data.rated_vel = gRated_Robo_Velocity;
        gStateE_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State F
        gStateF_Data.id = Field::FIELD_F;
        gStateF_Data.centre = Vec2<float>(1400, 4600);
        gStateF_Data.upper_bounds = Vec2<float>(2100, 5100);
        gStateF_Data.lower_bounds = Vec2<float>(1150, 3800);
        gStateF_Data.last_limit = 1;
        gStateF_Data.ramping_factor = 0.001;
        gStateF_Data.first_limit = gStateE_Data.last_limit;
        gStateF_Data.max_vel = gMax_Robo_Velocity;
        gStateF_Data.rated_vel = gRated_Robo_Velocity;
        gStateF_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State G
        gStateG_Data.id = Field::FIELD_G;
        gStateG_Data.centre = Vec2<float>(1550, 5300);
        gStateG_Data.upper_bounds = Vec2<float>(1875, 5650);
        gStateG_Data.lower_bounds = Vec2<float>(1300, 5080);
        gStateG_Data.last_limit = 1;
        gStateG_Data.ramping_factor = 0.001;
        gStateG_Data.first_limit = 1;
        gStateG_Data.max_vel = gMax_Robo_Velocity;
        gStateG_Data.rated_vel = gRated_Robo_Velocity;
        gStateG_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State H
        gStateH_Data.id = Field::FIELD_H;
        gStateH_Data.centre = Vec2<float>(1150, 6000);
        gStateH_Data.upper_bounds = Vec2<float>(1300, 6450);
        gStateH_Data.lower_bounds = Vec2<float>(1025, 5500);
        gStateH_Data.last_limit = 1;
        gStateH_Data.ramping_factor = 0.001;
        gStateH_Data.first_limit = 1;
        gStateH_Data.max_vel = gMax_Robo_Velocity;
        gStateH_Data.rated_vel = gRated_Robo_Velocity;
        gStateH_Data.ang_offset = -3;

        // *** Red Game Field *** //
        // Defining the state : State I
        gStateI_Data.id = Field::FIELD_I;
        gStateI_Data.centre = Vec2<float>(1325, 8600);
        gStateI_Data.upper_bounds = Vec2<float>(1600, 9000);
        gStateI_Data.lower_bounds = Vec2<float>(1000, 8200);
        gStateI_Data.last_limit = 1;
        gStateI_Data.ramping_factor = 0.004;
        gStateI_Data.first_limit = 0.825;
        gStateI_Data.max_vel = gMax_Robo_Velocity;
        gStateI_Data.rated_vel = gRated_Robo_Velocity;
        gStateI_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State J
        gStateJ_Data.id = Field::FIELD_J;
        gStateJ_Data.centre = Vec2<float>(5300, 8700);
        gStateJ_Data.upper_bounds = Vec2<float>(6500, 9500);
        gStateJ_Data.lower_bounds = Vec2<float>(4300, 7800);
        gStateJ_Data.last_limit = 1;
        gStateJ_Data.ramping_factor = 0.001;
        gStateJ_Data.first_limit = 1;
        gStateJ_Data.max_vel = 600;
        gStateJ_Data.rated_vel = 50;
        gStateJ_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State K
        gStateK_Data.id = Field::FIELD_K;
        gStateK_Data.centre = Vec2<float>(5500, 6500);
        gStateK_Data.upper_bounds = Vec2<float>(0, 0);
        gStateK_Data.lower_bounds = Vec2<float>(0, 0);
        gStateK_Data.last_limit = 0.7;
        gStateK_Data.ramping_factor = -0.001;
        gStateK_Data.first_limit = 1;
        gStateK_Data.max_vel = 800;
        gStateK_Data.rated_vel = 90;
        gStateK_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State L
        gStateL_Data.id = Field::FIELD_L;
        gStateL_Data.centre = Vec2<float>(6200, 8250);
        gStateL_Data.upper_bounds = Vec2<float>(0, 0);
        gStateL_Data.lower_bounds = Vec2<float>(0, 0);
        gStateL_Data.last_limit = 1;
        gStateL_Data.ramping_factor = 0.001;
        gStateL_Data.first_limit = 1;
        gStateL_Data.max_vel = 400;
        gStateL_Data.rated_vel = 40;
        gStateL_Data.ang_offset = 0;
        

        // *** Red Game Field *** //
        // Defining the state : State O
        gStateO_Data.id = Field::FIELD_O;
        gStateO_Data.centre = Vec2<float>(3700, 8500);
        gStateO_Data.upper_bounds = Vec2<float>(4000, 9000);
        gStateO_Data.lower_bounds = Vec2<float>(3000, 8000);
        gStateO_Data.last_limit = 1;
        gStateO_Data.ramping_factor = 0.01;
        gStateO_Data.first_limit = 0.6;
        gStateO_Data.max_vel = gMax_Robo_Velocity;
        gStateO_Data.rated_vel = gRated_Robo_Velocity;
        gStateO_Data.ang_offset = -7;

        // *** Red Game Field *** //
        // Defining the state : State P
        gStateP_Data.id = Field::FIELD_P;
        gStateP_Data.centre = Vec2<float>(3700, 7420);
        gStateP_Data.upper_bounds = Vec2<float>(4500, 7700);
        gStateP_Data.lower_bounds = Vec2<float>(2500, 7040);
        gStateP_Data.last_limit = 1;
        gStateP_Data.ramping_factor = 0.001;
        gStateP_Data.first_limit = 1;
        gStateP_Data.max_vel = gMax_Robo_Velocity;
        gStateP_Data.rated_vel = gRated_Robo_Velocity;
        gStateP_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State Q
        gStateQ_Data.id = Field::FIELD_Q;
        gStateQ_Data.centre = Vec2<float>(3800, 4320);
        gStateQ_Data.upper_bounds = Vec2<float>(4500, 4900);
        gStateQ_Data.lower_bounds = Vec2<float>(3000, 4000);
        gStateQ_Data.last_limit = 1;
        gStateQ_Data.ramping_factor = 0.001;
        gStateQ_Data.first_limit = 1;
        gStateQ_Data.max_vel = 600;
        gStateQ_Data.rated_vel = 60;
        gStateQ_Data.ang_offset = 10;

        // *** Red Game Field *** //
        // Defining the state : State Q1
        gStateQ1_Data.id = Field::FIELD_Q1;
        gStateQ1_Data.centre = Vec2<float>(4200, 5000);
        gStateQ1_Data.upper_bounds = Vec2<float>(4500, 5400);
        gStateQ1_Data.lower_bounds = Vec2<float>(3500, 4600);
        gStateQ1_Data.last_limit = 1;
        gStateQ1_Data.ramping_factor = 0.001;
        gStateQ1_Data.first_limit = 1;
        gStateQ1_Data.max_vel = 400;
        gStateQ1_Data.rated_vel = 40;
        gStateQ1_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State Q2
        gStateQ2_Data.id = Field::FIELD_Q2;
        gStateQ2_Data.centre = Vec2<float>(4000, 5000);
        gStateQ2_Data.upper_bounds = Vec2<float>(4500, 5400);
        gStateQ2_Data.lower_bounds = Vec2<float>(3500, 4600);
        gStateQ2_Data.last_limit = 1;
        gStateQ2_Data.ramping_factor = 0.001;
        gStateQ2_Data.first_limit = 1;
        gStateQ2_Data.max_vel = 400;
        gStateQ2_Data.rated_vel = 40;
        gStateQ2_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State R
        gStateR_Data.id = Field::FIELD_R;
        gStateR_Data.centre = Vec2<float>(3800, 3500);
        gStateR_Data.upper_bounds = Vec2<float>(4100, 4000);
        gStateR_Data.lower_bounds = Vec2<float>(3900, 3000);
        gStateR_Data.last_limit = 1;
        gStateR_Data.ramping_factor = 0.001;
        gStateR_Data.first_limit = 1;
        gStateR_Data.max_vel = 0;
        gStateR_Data.rated_vel = 0;
        gStateR_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State R1
        gStateR1_Data.id = Field::FIELD_R1;
        gStateR1_Data.centre = Vec2<float>(4250, 6000);
        gStateR1_Data.upper_bounds = Vec2<float>(4350, 8600);
        gStateR1_Data.lower_bounds = Vec2<float>(3900, 8400);
        gStateR1_Data.last_limit = 1;
        gStateR1_Data.ramping_factor = 0.001;
        gStateR1_Data.first_limit = 1;
        gStateR1_Data.max_vel = 0;
        gStateR1_Data.rated_vel = 0;
        gStateR1_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State R2
        gStateR2_Data.id = Field::FIELD_R2;
        gStateR2_Data.centre = Vec2<float>(3500, 3500);
        gStateR2_Data.upper_bounds = Vec2<float>(4100, 8600);
        gStateR2_Data.lower_bounds = Vec2<float>(3900, 8400);
        gStateR2_Data.last_limit = 1;
        gStateR2_Data.ramping_factor = 0.001;
        gStateR2_Data.first_limit = 1;
        gStateR2_Data.max_vel = 0;
        gStateR2_Data.rated_vel = 0;
        gStateR2_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State S
        gStateS_Data.id = Field::FIELD_S;
        gStateS_Data.centre = Vec2<float>(3800, 3500);
        gStateS_Data.upper_bounds = Vec2<float>(4100, 4000);
        gStateS_Data.lower_bounds = Vec2<float>(3900, 3000);
        gStateS_Data.last_limit = 1;
        gStateS_Data.ramping_factor = 0.001;
        gStateS_Data.first_limit = 1;
        gStateS_Data.max_vel = gMax_Robo_Velocity;
        gStateS_Data.rated_vel = gRated_Robo_Velocity;
        gStateS_Data.ang_offset = 0;

        // *** Red Game Field *** //
        // Defining the state : State T
        gStateT_Data.id = Field::FIELD_T;
        gStateT_Data.centre = Vec2<float>(4000, 8500);
        gStateT_Data.upper_bounds = Vec2<float>(4500, 9000);
        gStateT_Data.lower_bounds = Vec2<float>(3500, 8000);
        gStateT_Data.last_limit = 1;
        gStateT_Data.ramping_factor = 0.001;
        gStateT_Data.first_limit = 1;
        gStateT_Data.max_vel = 400;
        gStateT_Data.rated_vel = 40;
        gStateT_Data.ang_offset = 0;
}


static void init_BlueGameField()
{
        // *** Blue Game Field *** //
        // Defining the first state : State A
        gStateA_Data.id = Field::FIELD_A;
        gStateA_Data.centre = Vec2<float>(0, 0);
        gStateA_Data.upper_bounds = Vec2<float>(100, 100);
        gStateA_Data.lower_bounds = Vec2<float>(100, 100);
        gStateA_Data.last_limit = 1;
        gStateA_Data.ramping_factor = 0.01;
        gStateA_Data.first_limit = 0.6;
        gStateA_Data.max_vel = 1400;
        gStateA_Data.rated_vel = 140;
        gStateA_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State B
        gStateB_Data.id = Field::FIELD_B;
        gStateB_Data.centre = Vec2<float>(1500, 1500);
        gStateB_Data.upper_bounds = Vec2<float>(2500, 2000);
        gStateB_Data.lower_bounds = Vec2<float>(1200, 1000);
        gStateB_Data.last_limit = 1;
        gStateB_Data.ramping_factor = 0.001;
        gStateB_Data.first_limit = gStateA_Data.last_limit;
        gStateB_Data.max_vel = gMax_Robo_Velocity;
        gStateB_Data.rated_vel = gRated_Robo_Velocity;
        gStateB_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State C
        gStateC_Data.id = Field::FIELD_C;
        gStateC_Data.centre = Vec2<float>(1600, 2400);
        gStateC_Data.upper_bounds = Vec2<float>(2000, 2700);
        gStateC_Data.lower_bounds = Vec2<float>(1500, 1800);
        gStateC_Data.last_limit = 1;
        gStateC_Data.ramping_factor = 0.001;
        gStateC_Data.first_limit = gStateB_Data.last_limit;
        gStateC_Data.max_vel = gMax_Robo_Velocity;
        gStateC_Data.rated_vel = 120;
        gStateC_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State D
        gStateD_Data.id = Field::FIELD_D;
        gStateD_Data.centre = Vec2<float>(800, 2900);
        gStateD_Data.upper_bounds = Vec2<float>(1115, 3300);
        gStateD_Data.lower_bounds = Vec2<float>(500, 2700);
        gStateD_Data.last_limit = 1;
        gStateD_Data.ramping_factor = 0.001;
        gStateD_Data.first_limit = gStateC_Data.last_limit;
        gStateD_Data.max_vel = gMax_Robo_Velocity;
        gStateD_Data.rated_vel = gRated_Robo_Velocity;
        gStateD_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State E
        gStateE_Data.id = Field::FIELD_E;
        gStateE_Data.centre = Vec2<float>(550, 4000);
        gStateE_Data.upper_bounds = Vec2<float>(1050, 4100);
        gStateE_Data.lower_bounds = Vec2<float>(500, 3500);
        gStateE_Data.last_limit = 1;
        gStateE_Data.ramping_factor = 0.001;
        gStateE_Data.first_limit = gStateD_Data.last_limit;
        gStateE_Data.max_vel = gMax_Robo_Velocity;
        gStateE_Data.rated_vel = gRated_Robo_Velocity;
        gStateE_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State F
        gStateF_Data.id = Field::FIELD_F;
        gStateF_Data.centre = Vec2<float>(1400, 4600);
        gStateF_Data.upper_bounds = Vec2<float>(2100, 5100);
        gStateF_Data.lower_bounds = Vec2<float>(1150, 3800);
        gStateF_Data.last_limit = 1;
        gStateF_Data.ramping_factor = 0.001;
        gStateF_Data.first_limit = gStateE_Data.last_limit;
        gStateF_Data.max_vel = gMax_Robo_Velocity;
        gStateF_Data.rated_vel = gRated_Robo_Velocity;
        gStateF_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State G
        gStateG_Data.id = Field::FIELD_G;
        gStateG_Data.centre = Vec2<float>(1550, 5300);
        gStateG_Data.upper_bounds = Vec2<float>(1875, 5650);
        gStateG_Data.lower_bounds = Vec2<float>(1300, 5080);
        gStateG_Data.last_limit = 1;
        gStateG_Data.ramping_factor = 0.001;
        gStateG_Data.first_limit = 1;
        gStateG_Data.max_vel = gMax_Robo_Velocity;
        gStateG_Data.rated_vel = gRated_Robo_Velocity;
        gStateG_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State H
        gStateH_Data.id = Field::FIELD_H;
        gStateH_Data.centre = Vec2<float>(1150, 6000);
        gStateH_Data.upper_bounds = Vec2<float>(1400, 6450);
        gStateH_Data.lower_bounds = Vec2<float>(1025, 5500);
        gStateH_Data.last_limit = 1;
        gStateH_Data.ramping_factor = 0.001;
        gStateH_Data.first_limit = 1;
        gStateH_Data.max_vel = gMax_Robo_Velocity;
        gStateH_Data.rated_vel = gRated_Robo_Velocity;
        gStateH_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State I
        gStateI_Data.id = Field::FIELD_I;
        gStateI_Data.centre = Vec2<float>(1350, 8600);
        gStateI_Data.upper_bounds = Vec2<float>(1600, 9000);
        gStateI_Data.lower_bounds = Vec2<float>(1000, 8200);
        gStateI_Data.last_limit = 1;
        gStateI_Data.ramping_factor = 0.004;
        gStateI_Data.first_limit = 0.825;
        gStateI_Data.max_vel = gMax_Robo_Velocity;
        gStateI_Data.rated_vel = gRated_Robo_Velocity;
        gStateI_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State J
        gStateJ_Data.id = Field::FIELD_J;
        gStateJ_Data.centre = Vec2<float>(5300, 8700);
        gStateJ_Data.upper_bounds = Vec2<float>(6500, 9500);
        gStateJ_Data.lower_bounds = Vec2<float>(4300, 7800);
        gStateJ_Data.last_limit = 1;
        gStateJ_Data.ramping_factor = 0.001;
        gStateJ_Data.first_limit = 1;
        gStateJ_Data.max_vel = 600;
        gStateJ_Data.rated_vel = 50;
        gStateJ_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State K
        gStateK_Data.id = Field::FIELD_K;
        gStateK_Data.centre = Vec2<float>(5500, 6500);
        gStateK_Data.upper_bounds = Vec2<float>(0, 0);
        gStateK_Data.lower_bounds = Vec2<float>(0, 0);
        gStateK_Data.last_limit = 0.7;
        gStateK_Data.ramping_factor = -0.001;
        gStateK_Data.first_limit = 1;
        gStateK_Data.max_vel = 800;
        gStateK_Data.rated_vel = 90;
        gStateK_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State L
        gStateL_Data.id = Field::FIELD_L;
        gStateL_Data.centre = Vec2<float>(6200, 8250);
        gStateL_Data.upper_bounds = Vec2<float>(0, 0);
        gStateL_Data.lower_bounds = Vec2<float>(0, 0);
        gStateL_Data.last_limit = 1;
        gStateL_Data.ramping_factor = 0.001;
        gStateL_Data.first_limit = 1;
        gStateL_Data.max_vel = 400;
        gStateL_Data.rated_vel = 40;
        gStateL_Data.ang_offset = 0;
        

        // *** Blue Game Field *** //
        // Defining the state : State O
        gStateO_Data.id = Field::FIELD_O;
        gStateO_Data.centre = Vec2<float>(3700, 8500);
        gStateO_Data.upper_bounds = Vec2<float>(4000, 9000);
        gStateO_Data.lower_bounds = Vec2<float>(3000, 8000);
        gStateO_Data.last_limit = 1;
        gStateO_Data.ramping_factor = 0.01;
        gStateO_Data.first_limit = 0.6;
        gStateO_Data.max_vel = gMax_Robo_Velocity;
        gStateO_Data.rated_vel = gRated_Robo_Velocity;
        gStateO_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State P
        gStateP_Data.id = Field::FIELD_P;
        gStateP_Data.centre = Vec2<float>(3700, 7420);
        gStateP_Data.upper_bounds = Vec2<float>(4500, 7700);
        gStateP_Data.lower_bounds = Vec2<float>(2500, 7040);
        gStateP_Data.last_limit = 1;
        gStateP_Data.ramping_factor = 0.001;
        gStateP_Data.first_limit = 1;
        gStateP_Data.max_vel = gMax_Robo_Velocity;
        gStateP_Data.rated_vel = gRated_Robo_Velocity;
        gStateP_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State Q
        gStateQ_Data.id = Field::FIELD_Q;
        gStateQ_Data.centre = Vec2<float>(3700, 4320);
        gStateQ_Data.upper_bounds = Vec2<float>(4500, 4900);
        gStateQ_Data.lower_bounds = Vec2<float>(3000, 4000);
        gStateQ_Data.last_limit = 1;
        gStateQ_Data.ramping_factor = 0.001;
        gStateQ_Data.first_limit = 1;
        gStateQ_Data.max_vel = 600;
        gStateQ_Data.rated_vel = 60;
        gStateQ_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State Q1
        gStateQ1_Data.id = Field::FIELD_Q1;
        gStateQ1_Data.centre = Vec2<float>(4200, 5000);
        gStateQ1_Data.upper_bounds = Vec2<float>(4500, 5400);
        gStateQ1_Data.lower_bounds = Vec2<float>(3500, 4600);
        gStateQ1_Data.last_limit = 1;
        gStateQ1_Data.ramping_factor = 0.001;
        gStateQ1_Data.first_limit = 1;
        gStateQ1_Data.max_vel = 400;
        gStateQ1_Data.rated_vel = 40;
        gStateQ1_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State Q2
        gStateQ2_Data.id = Field::FIELD_Q2;
        gStateQ2_Data.centre = Vec2<float>(4000, 5000);
        gStateQ2_Data.upper_bounds = Vec2<float>(4500, 5400);
        gStateQ2_Data.lower_bounds = Vec2<float>(3500, 4600);
        gStateQ2_Data.last_limit = 1;
        gStateQ2_Data.ramping_factor = 0.001;
        gStateQ2_Data.first_limit = 1;
        gStateQ2_Data.max_vel = 400;
        gStateQ2_Data.rated_vel = 40;
        gStateQ2_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State R
        gStateR_Data.id = Field::FIELD_R;
        gStateR_Data.centre = Vec2<float>(3800, 3500);
        gStateR_Data.upper_bounds = Vec2<float>(4100, 4000);
        gStateR_Data.lower_bounds = Vec2<float>(3900, 3000);
        gStateR_Data.last_limit = 1;
        gStateR_Data.ramping_factor = 0.001;
        gStateR_Data.first_limit = 1;
        gStateR_Data.max_vel = 0;
        gStateR_Data.rated_vel = 0;
        gStateR_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State R1
        gStateR1_Data.id = Field::FIELD_R1;
        gStateR1_Data.centre = Vec2<float>(4250, 6000);
        gStateR1_Data.upper_bounds = Vec2<float>(4350, 8600);
        gStateR1_Data.lower_bounds = Vec2<float>(3900, 8400);
        gStateR1_Data.last_limit = 1;
        gStateR1_Data.ramping_factor = 0.001;
        gStateR1_Data.first_limit = 1;
        gStateR1_Data.max_vel = 0;
        gStateR1_Data.rated_vel = 0;
        gStateR1_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State R2
        gStateR2_Data.id = Field::FIELD_R2;
        gStateR2_Data.centre = Vec2<float>(3500, 3500);
        gStateR2_Data.upper_bounds = Vec2<float>(4100, 8600);
        gStateR2_Data.lower_bounds = Vec2<float>(3900, 8400);
        gStateR2_Data.last_limit = 1;
        gStateR2_Data.ramping_factor = 0.001;
        gStateR2_Data.first_limit = 1;
        gStateR2_Data.max_vel = 0;
        gStateR2_Data.rated_vel = 0;
        gStateR2_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State S
        gStateS_Data.id = Field::FIELD_S;
        gStateS_Data.centre = Vec2<float>(3800, 3500);
        gStateS_Data.upper_bounds = Vec2<float>(4100, 4000);
        gStateS_Data.lower_bounds = Vec2<float>(3900, 3000);
        gStateS_Data.last_limit = 1;
        gStateS_Data.ramping_factor = 0.001;
        gStateS_Data.first_limit = 1;
        gStateS_Data.max_vel = gMax_Robo_Velocity;
        gStateS_Data.rated_vel = gRated_Robo_Velocity;
        gStateS_Data.ang_offset = 0;

        // *** Blue Game Field *** //
        // Defining the state : State T
        gStateT_Data.id = Field::FIELD_T;
        gStateT_Data.centre = Vec2<float>(4000, 8500);
        gStateT_Data.upper_bounds = Vec2<float>(4500, 9000);
        gStateT_Data.lower_bounds = Vec2<float>(3500, 8000);
        gStateT_Data.last_limit = 1;
        gStateT_Data.ramping_factor = 0.001;
        gStateT_Data.first_limit = 1;
        gStateT_Data.max_vel = 400;
        gStateT_Data.rated_vel = 40;
        gStateT_Data.ang_offset = 0;
}
