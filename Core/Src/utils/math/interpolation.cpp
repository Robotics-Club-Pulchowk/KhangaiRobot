/*
 * interpolation.cpp
 *
 * Created : 4/14/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "interpolation.h"


float lerp(float x, float y, float t)
{
        return (1 - t)*x + t*y;
}

float cubic_herp(float x, float y, float t)
{
        float h_t = t*t*(-2*t + 3);
        return lerp(x, y, h_t);
}

Vec3<float> lerp(Vec3<float> x, Vec3<float> y, float t)
{
        Vec3<float> result = x.mult_EW(1-t) + y.mult_EW(t);
        return result;
}

Vec3<float> cubic_herp(Vec3<float> x, Vec3<float> y, float t)
{
        float h_t = t*t*(-2*t + 3);
        return lerp(x, y, h_t);
}
