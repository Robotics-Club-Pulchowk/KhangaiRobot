/*
 * freewheel.c
 * 
 * Created : 1/4/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "freewheel.h"

#define PI                              (3.14159)
#define DISTANCE_RATIO(PPR, RADIUS)     (2.0*(PI)*(RADIUS)/((PPR)))

void Enc_HandleCount(struct Enc *enc)
{
        if (HAL_GPIO_ReadPin(enc->chB_port, enc->chB_pin) != GPIO_PIN_RESET) {
                ++enc->count;
        } else {
                --enc->count;
        }
}

int32_t gYCount = 0;
int32_t gXCount = 0;

float Enc_get_DeltaDist(struct Enc *enc)
{
        int32_t last_count = enc->count;
        enc->count = 0;
        
        // if (enc->id == 'y') {
        //         gYCount += last_count;
        //         printf("%c : %ld\n", enc->id, gYCount);
        // }
        
        // if (enc->id == 'x') {
        //         gXCount += last_count;
        //         printf("%c : %ld\n", enc->id, gXCount);
        // }

        return ((float)last_count * DISTANCE_RATIO(enc->ppr, enc->radius));
}
