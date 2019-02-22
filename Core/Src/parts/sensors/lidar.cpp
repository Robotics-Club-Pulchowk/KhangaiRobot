/*
 * lidar.cpp
 * 
 * Created : 1/7/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "lidar.h"

int Lidar::init()
{
        adev_->init();
        return 0;
}

float Lidar::read()
{
        float dist = adev_->read();

        // printf("%ld\n", (int32_t)(dist));

        float mx,mn;
        mx = outliers_.getX();
        mn = outliers_.getY();

        if (mx > mn) {
                if (dist > mx || dist < mn) {
                        dist = last_data_;
                }
        }
        else {
                if (dist > mn || dist < mx) {
                        dist = last_data_;
                }
        }
        last_data_ = dist;

        return dist;
}

bool Lidar::available()
{
        return adev_->available();
}

void Lidar::denit()
{
        
}
