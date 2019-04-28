/*
 * state_sensor.h
 *
 * Created : 1/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _STATE_SENSOR_H_
#define _STATE_SENSOR_H_

#include "vec3.h"
#include "position_sensor.h"
#include "bound_box.h"

// SOme forward declarations
enum class Field;
struct State_Vars;

class State_Sensor final
{
public:
        State_Sensor(State_Sensor &&) = default;
        State_Sensor(const State_Sensor &) = default;
        State_Sensor &operator=(State_Sensor &&) = default;
        State_Sensor &operator=(const State_Sensor &) = default;
        ~State_Sensor() { }

        static State_Sensor& get_Instance();

        int init(uint32_t dt_millis);
        Vec3<float> read_State(Vec3<float> base_state, const State_Vars *sv, uint32_t dt_millis);

        
        void add_PSensor(Sensor<float> *sen) { p_sensor_->add_Sensor(sen); }
        void remove_PSensor(Sensor<float> *sen) { p_sensor_->remove_Sensor(sen); }

        void change_Sensors(Field field_id);
        uint8_t get_Bounds();

        void update_Position(Vec3<float> pos) { p_sensor_->update_State(pos); }

private:
        PositionSensor *p_sensor_;

        Bound_Box *bound_box_;
        uint8_t bounds_;

        bool is_first_ori_;
        Vec3<float> first_ori_;

        State_Sensor() { }
        Vec3<float> read_Orientation(Vec3<float> base_state, uint32_t dt_millis);
        Vec3<float> compensate_Bounds(Vec3<float> pos, Vec3<float> ori, const State_Vars *sv);
};

#endif // !_STATE_SENSOR_H_
