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
#include "min_jerk.h"
#include "calculus.h"
#include "polynomial.h"
#include "array.h"
#include "min_accel.h"

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

float Robo_States::calc_RoboVelocity(Vec3<float> state, uint32_t dt_millis)
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
        
        return ramped_ * (float)(sv_->max_vel);
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

// state -> mm mm deg
// vel_from_base -> m/s m/s rad/s
Vec2<float> Robo_States::calc_Velocity(Vec3<float> state, Vec3<float> vel_from_base, uint32_t dt_millis)
{
        Vec2<float> velocity;

        //** This Part contains minimum accelration implementation.

        float ma_polyX[4] = { 0 };
        float ma_polyY[4] = { 0 };
        float ma_polyR[4] = { 0 };

        // The centre value is in mm
        Vec2<float> centre = next_state_->sv_->centre;

        Vec2<float> pos(state.getX() / 1000.0, centre.getX() / 1000.0); // mm mm
        Vec2<float> vel(-vel_from_base.getX(), 0);       // m/s m/s

        // Get a new vector to the goal position
        Vec2<float> del = centre - Vec2<float>(centre.getX() / 1000.0, centre.getY() / 1000.0);
        Vec2<float> del_polar = del.polar();

        float rated_vel = (float)(sv_->rated_vel);       // m/s
        // New total distance to cover
        float dr = del_polar.getX() / 1000.0;   // m
        // Time to cover the new distance
        float Tp = dr / rated_vel;      // s

        // Convert the vx and vy obtained from robot base to polar form
        Vec2<float> r_v = Vec2<float>(vel_from_base).polar();
        // Start velocity of robot and the desired end velocity
        Vec2<float> r_vel(r_v.getX(), 0);
        // Start position of robot and the desired end position
        Vec2<float> r_pos(0, del_polar.getX());

        // Minimum Accleration Trajectory in distance
        min_accel(ma_polyR, r_pos, r_vel, Tp);

        // Minimum Accleration Trajectory in x-axis
        min_accel(ma_polyX, pos, vel, Tp);

        pos.set_Values(state.getY() / 1000.0, centre.getY() / 1000.0);
        vel.set_Values(vel_from_base.getY(), 0);

        // Minimum Accleration Trajectory in y-axis
        min_accel(ma_polyY, pos, vel, Tp);

        float ma_xdot[3] = { 0 };
        float ma_ydot[3] = { 0 };
        float ma_rdot[3] = { 0 };

        // Get dx/dt, dy/dt and dr/dt
        polyder(ma_xdot, ma_polyX);
        polyder(ma_ydot, ma_polyY);
        polyder(ma_rdot, ma_polyR);

        float dt = (float)dt_millis / 1000.0;
        float xdot = polyval(ma_xdot, dt);
        float ydot = polyval(ma_ydot, dt);
        float rdot = polyval(ma_rdot, dt);
        
        Vec2<float> pdot(ydot, xdot);
        pdot = pdot.polar();

        // Get robot's velocity from the trajectory obtained in distance and
        // the angle from x and y axes' trajectory
        velocity.set_Values(rdot, pdot.getY());

        // Clamp the velocity of the robot
        velocity.setX(velocity.getX() * 1000.0);
        if (velocity.getX() > sv_->max_vel) {
                velocity.setX((float)(sv_->max_vel));
        }

        return velocity;
}

bool Robo_States::nextStateReached(Vec3<float> state, uint8_t bounds)
{
        //* Special Case for Field K and L
        Field id = next_state_->get_ID();
        if (id == Field::FIELD_K) {
                if (bounds & 1 << (int)(Face::_6)) {
                        return true;
                }
                return false;
        }
        else if (id == Field::FIELD_L) {
                if ((bounds & (1 << (int)(Face::_6))) && (bounds & (1 << (int)(Face::_8)))) {
                        return true;
                }
                return false;

        }
        else if (id == Field::FIELD_Q1) {
                if (bounds & 1 << (int)(Face::_6)) {
                        return true;
                }
                return false;
        }
        else if (id == Field::FIELD_Q2) {
                if (bounds & 1 << (int)(Face::_7)) {
                        return true;
                }
        }
        else if (id == Field::FIELD_R || id == Field::FIELD_R1 || id == Field::FIELD_R2) {
                if ((bounds & 1 << (int)(Face::_6)) && (bounds & 1 << (int)(Face::_7))) {
                        return true;
                }
                return false;
        }
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
