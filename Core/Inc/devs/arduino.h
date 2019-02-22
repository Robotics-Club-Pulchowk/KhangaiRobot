/*
 * arduino.h
 * 
 * Created : 1/5/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _ARDUINO_H_
#define _ARDUINO_H_

#include "sensor.h"
#include "queue_custom.h"


class Arduino_Device
{
public:
        Arduino_Device(uint8_t id, uint8_t num_bytes);
        Arduino_Device(Arduino_Device &&) = default;
        Arduino_Device(const Arduino_Device &) = default;
        Arduino_Device &operator=(Arduino_Device &&) = default;
        Arduino_Device &operator=(const Arduino_Device &) = default;
        ~Arduino_Device() { }
        
        static int init();
        float read();
        bool available();
        void denit();

        int read(uint8_t *buf, uint16_t len);
        
        uint8_t get_ID() const { return id_; }
        int write(uint8_t *buf, uint16_t len);

        void store(float val) { rx_data_.insert(val); }
private:
        Queue<float, 2> rx_data_;
        static bool initiated;
        uint8_t id_;
};


void add_Device(Arduino_Device *adev);

void Arduino_Handle_RxCplt(void);
void Arduino_Handle_TxCplt(void);

#endif // !_ARDUINO_H_
