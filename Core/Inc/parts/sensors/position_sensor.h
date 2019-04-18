/*
 * position_sensor.h
 * 
 * Created : 11/29/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _POSITION_SENSOR_H_
#define _POSITION_SENSOR_H_

#include "vec3.h"
#include "sensor.h"
#include "kalman.h"

struct State_Vars;
enum class Field;

#define MAX_POSITION_SENSORS    (4)

extern Kalman_Vars gEncoders_KV;
extern Kalman_Vars gXLidarEncoder_KV;

int init_XLidarEncoderKalman(uint32_t dt_millis);
int init_EncodersKalman(uint32_t dt_millis);


class PositionSensor
{
public:
        PositionSensor(PositionSensor &&) = default;
        PositionSensor(const PositionSensor &) = default;
        PositionSensor &operator=(PositionSensor &&) = default;
        PositionSensor &operator=(const PositionSensor &) = default;
        ~PositionSensor() { }

        static PositionSensor& get_Instance();

        int init(uint32_t dt_millis);

        // Add sensor so that it can be used for calculating position
        void add_Sensor(Sensor<float> *sen) {
                for (uint8_t i = 0; i < sensor_count_; ++i) {
                        if (*p_sensors_[i] == *sen) {
                                return;
                        }
                }
                p_sensors_[sensor_count_++] = sen;
        }
        // Remove any Sensor from the next calculation
        void remove_Sensor(Sensor<float> *sen) {
                for (uint8_t i = 0; i < sensor_count_; ++i) {
                        if (*p_sensors_[i] == *sen) {
                                --sensor_count_;
                                p_sensors_[i] = p_sensors_[sensor_count_];
                                break;
                        }
                }
        }

        // Reads the position of the robot from the initial fences
        // using the available Sensors
        Vec3<float> read_Position(Vec3<float> ori, Vec3<float> base_state, const State_Vars *sv, uint32_t dt_millis);
        void update_State(Vec3<float> state);

private:
        Sensor<float> *p_sensors_[MAX_POSITION_SENSORS];
        uint8_t sensor_count_;

        Kalman_Filter enc_fuser_;
        Kalman_Filter xlidar_enc_fuser_;
        
        PositionSensor() :
        enc_fuser_(&gEncoders_KV, init_EncodersKalman), xlidar_enc_fuser_(&gXLidarEncoder_KV, init_XLidarEncoderKalman)
        {
                sensor_count_ = 0;
        }

        void process_LidarData(float (&lidar)[2], const State_Vars *sv);
};

#endif // !_POSITION_SENSOR_H_
