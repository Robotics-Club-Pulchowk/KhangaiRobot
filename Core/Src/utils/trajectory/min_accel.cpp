/*
 * min_accel.cpp
 *
 * Created : 23/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "min_accel.h"
#include "mat.h"
#include "array.h"


void min_accel(float (&poly)[4], Vec2<float> pos, Vec2<float> vel, float Tp)
{
        // Printing the parameters
        // printArr(poly);
        // printf(" [");
        // (pos.mult_EW(1000)).print();
        // printf("] [");
        // (vel.mult_EW(1000)).print();
        // printf("] [%d]\n", (int)(Tp*1000));
        //

        float x1 = pos.getX();
        float x2 = pos.getY();
        float x1_dot = vel.getX();
        float x2_dot = vel.getY();

        poly[0] = x1;
        poly[1] = x1_dot;

        float Tp2 = Tp*Tp;
        float Tp3 = Tp2*Tp;

        Mat B(2,1);
        B.at(0,0) = x2 - x1 - x1_dot*Tp;
        B.at(1,0) = x2_dot - x1_dot;

        float T_arr[2][2] = { { Tp3,   Tp2 },
                              { 3*Tp2, 2*Tp } };

        Mat T(T_arr);
        Mat A = solve(T, B);

        poly[3] = A.at(0,0);
        poly[2] = A.at(1,0);
}