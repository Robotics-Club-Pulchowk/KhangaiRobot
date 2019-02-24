/*
 * min_jerk.h
 *
 * Created : 23/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _MIN_JERK_H_
#define _MIN_JERK_H_

#include "vec2.h"

void min_jerk(float (&poly)[6], Vec2<float> pos, Vec2<float> vel, Vec2<float> accel, float Tp);

#endif // !_MIN_JERK_H_
