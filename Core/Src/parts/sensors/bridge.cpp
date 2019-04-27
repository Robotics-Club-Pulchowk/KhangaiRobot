/*
 * bridge.cpp
 * 
 * Created : 4/3/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "bridge.h"

// *** Construction of Bridge Moore Machine ***

//* BPx_f => callback function if xth bridge pole reached
void BP1_f() { gLast_YEncoderValue = 6530; }
void BP2_f() { gLast_YEncoderValue = 7030; }
void BP3_f() { gLast_YEncoderValue = 7530; }
void BP4_f() { gLast_YEncoderValue = 8030; }


//* BPx => xth bridge pole
//* BRx => xth pole past (river) of bridge
static State B(0);
static State BP1(1, BP1_f);
static State BR1(2);
static State BP2(3, BP2_f);
static State BR2(4);
static State BP3(5, BP3_f);
static State BR3(6);
static State BP4(7, BP4_f);
static State BR4(8);

static State* gStates[gN_States] = { &B, &BP1, &BR1, &BP2, &BR2, &BP3, &BR3, &BP4, &BR4 };
static int gInputs[gN_Inputs] = { 0, 1 };
static size_t gDel[gN_States][gN_Inputs] = { { 1, 0 },
                                             { 1, 2 },
                                             { 3, 2 },
                                             { 3, 4 },
                                             { 5, 4 },
                                             { 5, 6 },
                                             { 7, 6 },
                                             { 7, 8 },
                                             { 8, 8 } };

Moore_Machine<gN_States, gN_Inputs> gBridge_Machine(gStates, gInputs, gDel);

// ***     End Of Bridge Moore Machine      ***
