/*
 * error.h
 *
 * Created : 1/10/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _ERROR_H_
#define _ERROR_H_

enum Error {
        NO_ERROR,

        PERIPHERAL_ERROR,
        DEVICE_ERROR
};

void error(Error err);

#endif // !_ERROR_H_

