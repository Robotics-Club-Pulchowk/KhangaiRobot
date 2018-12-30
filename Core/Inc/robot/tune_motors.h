#ifndef _TUNE_MOTORS_H_
#define _TUNE_MOTORS_H_

#include "robo_init.h"

void tune(float set_points[4], uint32_t dt_millis);
void ramp_down(uint32_t dt_millis);

#endif // !_TUNE_MOTORS_H_
