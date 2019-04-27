/*
 * bridge.h
 * 
 * Created : 4/3/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _BRIDGE_H_
#define _BRIDGE_H_

#include "moore.h"

//* According to the drawn state diagram, we need 9 states and there are
//* 2 possible inputs
const size_t gN_States = 9;
const size_t gN_Inputs = 2;

extern Moore_Machine<gN_States, gN_Inputs> gBridge_Machine;

//* The following global variable(s) are modified by the state machine
extern float gLast_YEncoderValue;

#endif // !_BRIDGE_H_
