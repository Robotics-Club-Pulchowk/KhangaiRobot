/*
 * lidar.h
 * 
 * Created : 1/7/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _LIDAR_H_
#define _LIDAR_H_

#include "sensor.h"
#include "arduino.h"
#include "vec2.h"

class Lidar : public Sensor<float>
{
public:
        Lidar() = default;
        Lidar(Lidar &&) = default;
        Lidar(const Lidar &) = default;
        Lidar &operator=(Lidar &&) = default;
        Lidar &operator=(const Lidar &) = default;
        ~Lidar() { }
        
        Lidar(Arduino_Device *adev, SensorName name) {
                adev_ = adev;
                name_ = name;
        }
        
        int init() override;
        float read() override;
        bool available() override;
        void denit() override;

        void set_Outliers(Vec2<float> out) { outliers_ = out; }

private:
        Arduino_Device *adev_;
        Vec2<float> outliers_;
        float last_data_;
};

#endif // !_LIDAR_H_
