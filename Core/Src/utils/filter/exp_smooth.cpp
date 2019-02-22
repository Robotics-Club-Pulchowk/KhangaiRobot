/*
 * exp_smooth.cpp
 * 
 * Created : 11/9/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "smoothing_algorithm.h"

//* Exponential Smoothing Implementation
Exp_Smooth::Exp_Smooth()
{
        alpha_ = 1;
        is_first_ = true;
}

Exp_Smooth::Exp_Smooth(float alpha)
{
        alpha_ = alpha;
        is_first_ = true;
}

float Exp_Smooth::smooth(const float &input)
{
        if (is_first_) {
                is_first_ = false;
                last_output_ = input;
        }
        else {
                float residue = input - last_output_;
                residue *= alpha_;
                last_output_ += residue;
        }
        return last_output_;
}

void Exp_Smooth::clear()
{
        is_first_ = true;
}
