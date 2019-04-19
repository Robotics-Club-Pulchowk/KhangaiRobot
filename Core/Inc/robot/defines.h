#ifndef _DEFINES_H_
#define _DEFINES_H_


#define MOTOR_DRIVER_FREQUENCY  (8000)
#define PWM_TIMER_FREQUENCY     (168000000)


#define MAX_OMEGA       70    // w = 2*pi*f

#define MAX_POSSIBLE_OMEGA	(70)    // Maximum value that may be computed


#define _USE_BOARD_LEDS_FOR_JOYSTICK

// #undef _USE_BOARD_LEDS_FOR_ERROR_
// #define _USE_mikroBUS_LEDS_FOR_ERROR_


#define _ENABLE_I2C_ERROR_DETECTION

#endif  // _DEFINES_H_
