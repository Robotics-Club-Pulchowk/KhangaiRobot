/*
 * throwing.h
 * 
 * Created : 5/18/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _THROWING_H_
#define _THROWING_H_

#include "queue_custom.h"


enum class Throwing_Commands
{
        GRIP = 0x01,
        ACTUATE = 0x02,
        EXTEND = 0x03,
        PASS_GEREGE = 0x04,
        THROW = 0x05,
        RETRIEVE = 0x06,
        MOVE_PLATFORM_LEFT = 0x07,
        MOVE_PLATFORM_RIGHT = 0x08
};

class Throwing
{
public:
        Throwing(Throwing &&) = default;
        Throwing(const Throwing &) = default;
        Throwing &operator=(Throwing &&) = default;
        Throwing &operator=(const Throwing &) = default;
        ~Throwing() { }
        
        static Throwing& get_Instance();
        
        int init();
        int write(uint8_t cmd);

private:
        Throwing() { }
};


void Throwing_Handle_TxCplt(void);


#endif //! _THROWING_H_
