/*
 * bound_box.cpp
 * 
 * Created : 3/30/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */
/*

          ^        2
              __ __ __ __
         1   /           \  3
            /             \
           |               |
        8  |               |  4
           |               |
            \             /
         7   \__ __ __ __/  5

                   6
*/

#include "bound_box.h"

Bound_Box& Bound_Box::get_Instance()
{
        static Bound_Box sRobo_Bounds;

        return sRobo_Bounds;
}

int Bound_Box::init()
{
        //* For now there are no limit switches in bounds : 1 2 and 3

        //* Limit Switch Assignment for Bound 4
        // bounds_[3].add_LimitSwitch(GPIOA, GPIO_PIN_0);
        // bounds_[3].add_LimitSwitch(GPIOA, GPIO_PIN_1);

        //* Limit Switch Assignment for Bound 5
        // bounds_[4].add_LimitSwitch(GPIOA, GPIO_PIN_0);

        //* Limit Switch Assignment for Bound 6
        bounds_[5].add_LimitSwitch(FENCE_6A_GPIO_Port, FENCE_6A_Pin);
        bounds_[5].add_LimitSwitch(FENCE_6B_GPIO_Port, FENCE_6B_Pin);

        //* Limit Switch Assignment for Bound 7
        bounds_[6].add_LimitSwitch(FENCE_7_GPIO_Port, FENCE_7_Pin);

        //* Limit Switch Assignment for Bound 8
        bounds_[7].add_LimitSwitch(FENCE_8A_GPIO_Port, FENCE_8A_Pin);
        bounds_[7].add_LimitSwitch(FENCE_8B_GPIO_Port, FENCE_8B_Pin);

        return 0;
}
