/*
 * encoder.cpp
 * 
 * Created : 11/16/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "encoder.h"

int Encoder::init()
{
        return 0;
}

// static float gYDist = 0;

float Encoder::read()
{
        float dist = Enc_get_DeltaDist(enc_);
        // printf("%c : %ld\n", enc_->id, (int32_t)(dist*1000));
        
        
        // if (enc_->id == 'y') {
        //         gYDist += dist;
        //         printf("%c : %ld\n", enc_->id, (int32_t)(gYDist));
        // }

        return dist;
}

bool Encoder::available()
{
        return true;
}

void Encoder::denit()
{
        
}
