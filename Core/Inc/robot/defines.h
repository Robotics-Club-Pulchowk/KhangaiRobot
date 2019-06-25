#ifndef _DEFINES_H_
#define _DEFINES_H_


#define MOTOR_DRIVER_FREQUENCY  (8000)
#define PWM_TIMER_FREQUENCY     (168000000)


#define MAX_OMEGA       70    // w = 2*pi*f

#define MAX_POSSIBLE_OMEGA	(70)    // Maximum value that may be computed

#define START_BYTE      (0xA5)

//* Packet IDs
#define MOTOR_PACKET_ID                 (0x01)
#define ANGLE_PACKET_ID                 (0x02)
#define COMPASS_PACKET_ID               (0x03)
#define JOYSTICK_ERROR_PACKET_ID        (0x04)
#define STATE_PACKET_ID                 (0x05)

#define STOP_PACKET_ID                  (0x5A)

// #define _USE_BOARD_LEDS_FOR_JOYSTICK

#undef _USE_BOARD_LEDS_FOR_ERROR_
// #define _USE_mikroBUS_LEDS_FOR_ERROR_


#define _ENABLE_I2C_ERROR_DETECTION

enum GameField {
        NONE,
        RED,
        BLUE
};

extern GameField gCurrent_Field;

#endif  // _DEFINES_H_
