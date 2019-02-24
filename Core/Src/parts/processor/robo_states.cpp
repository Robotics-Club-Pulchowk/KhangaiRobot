/*
 * robo_states.cpp
 *
 * Created : 1/9/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include <math.h>

#include "robo_states.h"
#include "devs_config.h"

#define CURVE_STEP_SIZE         (10.0)

void init_GameField();

static bool gGameField_Initiated = false;

Robo_States::Robo_States(State_Vars *sv, Robo_States *next)
{
        sv_ = sv;
        next_state_ = next;

        if (!gGameField_Initiated) {
                init_GameField();
                gGameField_Initiated = true;
        }

        ramped_ = sv_->first_limit;
}

float Robo_States::calc_RoboVelocity()
{
        // We are simply returning the maximum velocity of robot
        // We can probably use position PID here to obtain better robot control

        if (ramped_ < sv_->last_limit) {
                ramped_ += sv_->ramping_factor;
        }

        if (ramped_ > 1.0) {
                ramped_ = 1;
        }
        else if (ramped_ < 0) {
                ramped_ = 0;
        }

        // printf("%ld : %d\n", (HAL_GetTick()), (int16_t)(ramped_*1000));
        
        return ramped_ * (float)(gMax_Robo_Velocity);
}

float Robo_States::quadTheta(Vec3<float> state, float v, uint32_t dt_millis)
{
        float x1 = state.getX();
        float y1 = state.getY();
        Vec2<float> p2 = next_state_->sv_->centre;
        Vec2<float> p3 = next_state_->next_state_->sv_->centre;
        float x2 = p2.getX();
        float y2 = p2.getY();
        float x3 = p3.getX();
        float y3 = p3.getY();

        Vec3<float> X(x1, x2, x3);

        float Vandermonde[3][3] = {{ y1*y1, y1, 1 },
                                   { y2*y2, y2, 1 },
                                   { y3*y3, y3, 1 } };

        Mat V(Vandermonde);
        Mat coeffs = V.inv() * X;

        auto P = [&](float x) {
                float a = coeffs.at(0,0);
                float b = coeffs.at(1,0);
                float c = coeffs.at(2,0);

                return a*x*x + b*x + c;
        };

        float dt = (float)dt_millis / 1000.0;
        float r = v*dt;

        float dx(0), dy(0);

        while (((dx*dx) + (dy*dy)) < (r*r)) {
                dy += CURVE_STEP_SIZE;
                dx = P(y1 + dy) - x1;
        }

        // printf("%ld\n", HAL_GetTick());
        float theta = atan2f(dx,dy);
        printf("%ld\n", (int32_t)(theta*57.3));

        return theta;
}

float Robo_States::linearTheta(Vec3<float> state, float v, uint32_t dt_millis)
{
        Vec2<float> del = next_state_->sv_->centre - Vec2<float>(state);

        float dx = del.getX();
        float dy = del.getY();

        // printf("%d %d  :  ", (int16_t)dx, (int16_t)dy);
        float theta = atan2f(dx, dy);

        return theta;
}


float Robo_States::calc_AngleOfAttack(Vec3<float> state, float v, uint32_t dt_millis)
{
        // Let's use linear spline for first try then we should probably switch
        // to cubic spline later

        float theta = linearTheta(state, v, dt_millis);

        return atan2f(sin(theta), cos(theta));
}

Vec2<float> Robo_States::calc_Velocity(Vec3<float> state, uint32_t dt_millis)
{
        // Using Naive Approach
        float v = calc_RoboVelocity();
        float theta = calc_AngleOfAttack(state, v, dt_millis);

        Vec2<float> vel(v, theta);

        return vel;
}

bool Robo_States::nextStateReached(Vec3<float> state)
{
        // Since the axis of the rectangle are aligned with the axes, we can
        // easily calculate if a point is inside the rectange
        Vec2<float> upper = next_state_->sv_->upper_bounds;
        Vec2<float> lower = next_state_->sv_->lower_bounds;
        float mx, my, mnx, mny;

        // Correct the bounds for checking whether the robot has reached within
        // next state's zone
        if (upper.getX() > lower.getX()) {
                mx = upper.getX();
                mnx = lower.getX();
        }
        else {
                mnx = upper.getX();
                mx = lower.getX();
        }
        
        if (upper.getY() > lower.getY()) {
                my = upper.getY();
                mny = lower.getY();
        }
        else {
                mny = upper.getY();
                my = lower.getY();
        }

        float x = state.getX();
        float y = state.getY();

        // Check if x is bounded
        if (x > mx || x < mnx) {
                return false;
        }
        // Check if y is bounded
        if (y > my || y < mny) {
                return false;
        }

        return true;
}


