/*
 * min_jerk.cpp
 *
 * Created : 23/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "min_jerk.h"
#include "mat.h"


void min_jerk(float (&poly)[6], Vec2<float> pos, Vec2<float> vel, Vec2<float> accel, float Tp)
{
        float x1 = pos.getX();
        float x2 = pos.getY();
        float x1_dot = vel.getX();
        float x2_dot = vel.getY();
        float x1_ddot = accel.getX();
        float x2_ddot = accel.getY();

        poly[0] = x1;
        poly[1] = x1_dot;
        poly[2] = x1_ddot / 2.0;

        Mat B(3,1);
        B.at(0,0) = x2 - x1 - x1_dot*Tp - x1_ddot*Tp*Tp/2.0;
        B.at(1,0) = x2_dot - x1_dot - x1_ddot*Tp;
        B.at(2,0) = x2_ddot - x1_ddot;

        float Tp2 = Tp*Tp;
        float Tp3 = Tp2*Tp;
        float Tp4 = Tp3*Tp;
        float Tp5 = Tp4*Tp;

        float T_arr[3][3] = { { Tp5,    Tp4,    Tp3   },
                              { 5*Tp4,  4*Tp3,  3*Tp2 },
                              { 20*Tp3, 12*Tp2, 6*Tp  } };

        Mat T(T_arr);
        Mat A = solve(T, B);

        poly[5] = A.at(0,0);
        poly[4] = A.at(1,0);
        poly[3] = A.at(2,0);
}